"""Observe SDK calls in the existing driver process, without a second CAN owner."""

import math
import threading
import time

import numpy as np

from data_collect.control.kinematics import pose_error
from data_collect.common.diagnostics import Timings

JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 7))


class TeleopTarget:
    """Aggregate the reference teleop's partial finite joint/gripper commands."""

    def __init__(self):
        self.enabled = False
        self.values = {}

    def set_enabled(self, enabled):
        self.enabled = bool(enabled)
        self.values.clear()

    def update(self, names, positions):
        if not self.enabled or len(names) != len(positions):
            return
        allowed = {*JOINT_NAMES, "gripper"}
        for name, position in zip(names, positions):
            if name in allowed and math.isfinite(position):
                self.values[name] = float(position)

    def snapshot(self):
        names = (*JOINT_NAMES, "gripper")
        if not self.enabled or any(name not in self.values for name in names):
            return None
        return [self.values[name] for name in names]


class RobotObserver:
    def __init__(self, manager, fk=None, tracking_latency=None):
        self.manager = manager
        self.fk = fk
        self.lock = threading.RLock()
        self.target_joint = None
        self.target_gripper = None
        self.following = False
        self.seq = 0
        self.packet_ts = {}
        self.available_ts = {}
        self.timings = Timings()
        self.tracking_latency = tracking_latency
        self.joint_command_begin = self.gripper_command_begin = None

    def command(self, kind, value, begin, end):
        if kind == "joint":
            self.target_joint = np.asarray(value, dtype=np.float32).copy()
            self.joint_command_begin = begin
        else:
            self.target_gripper = np.array([value], dtype=np.float32)
            self.gripper_command_begin = begin
        if self.target_joint is None or self.target_gripper is None:
            return
        self.seq += 1
        timing_fields = {}
        if self.tracking_latency:
            from data_collect.calibration.tracking import response_fields

            timing_fields = response_fields(self.joint_command_begin, self.gripper_command_begin, self.tracking_latency)
        self.manager.emit(
            "robot_command",
            {
                "target_joint_pos": self.target_joint.copy(),
                "target_gripper_pos": self.target_gripper.copy(),
                "cmd_call_begin_ns": np.int64(begin),
                "cmd_call_end_ns": np.int64(end),
                "cmd_seq": np.int64(self.seq),
                "following": np.bool_(self.following),
                **timing_fields,
            },
        )

    def mark_packet(self, key, packet, now):
        stamp = float(packet.timestamp)
        if not np.isfinite(stamp) or stamp <= 0:
            raise ValueError(f"{key} SDK 时间戳无效")
        changed = self.packet_ts.get(key) != stamp
        if changed:
            if key in self.packet_ts and stamp < self.packet_ts[key]:
                raise ValueError(f"{key} SDK 时间戳回退")
            self.packet_ts[key] = stamp
            self.available_ts[key] = now
        return changed

    def poll(self, arm, gripper):
        begin = time.monotonic_ns()
        with self.lock:
            acquired = time.monotonic_ns()
            joint = arm.get_joint_angles()
            jt = time.monotonic_ns()
            flange = arm.get_flange_pose()
            tt = time.monotonic_ns()
            grip = gripper.get_status() if gripper else None
            gt = time.monotonic_ns()
            if any(p is None or p.hz <= 0 for p in (joint, flange, grip)):
                return False
            changes = [
                self.mark_packet("joint", joint, jt),
                self.mark_packet("tcp", flange, tt),
                self.mark_packet("gripper", grip, gt),
            ]
            record = {
                "joint_pos": np.asarray(joint.msg, dtype=np.float32).copy(),
                "gripper_pos": np.array([grip.width], dtype=np.float32),
                "tcp_pose": np.asarray(
                    arm.get_flange2tcp_pose(flange.msg), dtype=np.float32
                ).copy(),
                "available_host_ts_ns": np.int64(gt),
                "joint_available_host_ts_ns": np.int64(self.available_ts["joint"]),
                "tcp_available_host_ts_ns": np.int64(self.available_ts["tcp"]),
                "gripper_available_host_ts_ns": np.int64(self.available_ts["gripper"]),
                "sdk_joint_ts": np.float64(joint.timestamp),
                "sdk_tcp_ts": np.float64(flange.timestamp),
                "sdk_gripper_ts": np.float64(grip.timestamp),
                "is_new_packet": np.bool_(any(changes)),
            }
        released = time.monotonic_ns()
        # Owned snapshots above allow CPU work without holding the SDK lock.
        if self.fk is not None:
            record["model_tcp_pose"] = self.fk(record["joint_pos"])
            position, rotation = pose_error(record["model_tcp_pose"], record["tcp_pose"])
            record["fk_position_error_m"] = np.float64(position)
            record["fk_rotation_error_rad"] = np.float64(rotation)
        computed = time.monotonic_ns()
        for name, duration in (("feedback_lock_wait", acquired - begin),
                               ("sdk_read_joint", jt - acquired),
                               ("sdk_read_tcp", tt - jt),
                               ("sdk_read_gripper", gt - tt),
                               ("feedback_lock_held", released - acquired),
                               ("fk_compute", computed - released)):
            self.timings.add(name, duration)
        self.manager.emit("robot_feedback", record)
        self.timings.add("feedback_poll", time.monotonic_ns() - begin)
        return True


class LockedProxy:
    """All public SDK access uses one reentrant lock; observe successful commands."""

    def __init__(self, target, observer, gripper=False, sdk_gripper=False):
        self._target = target
        self._observer = observer
        self._gripper = gripper
        self._sdk_gripper = sdk_gripper

    def __getattr__(self, name):
        value = getattr(self._target, name)
        # SDK enums/constants such as ARM_STATUS are classes and callable too.
        # Preserve their nested members used by the original driver's control gate.
        if not callable(value) or isinstance(value, type):
            return value

        def call(*args, **kwargs):
            obs = self._observer
            waiting = time.monotonic_ns()
            with obs.lock:
                begin = time.monotonic_ns()
                result = value(*args, **kwargs)
                end = time.monotonic_ns()
                if result is not False:
                    if not self._gripper and name in ("move_j", "move_js"):
                        joints = args[0] if args else kwargs["joints"]
                        obs.command("joint", joints, begin, end)
                    elif self._sdk_gripper and name == "move_gripper_m":
                        width = args[0] if args else kwargs["value"]
                        obs.command("gripper", width, begin, end)
                    elif not self._gripper and name in ("move_p", "move_l", "move_c", "move_mit"):
                        obs.manager.fail(
                            f"unsupported_collection_command: {name}; 采集仅支持关节目标"
                        )
            # Record after releasing this level of the SDK lock.
            obs.timings.add("sdk_lock_wait", begin - waiting)
            obs.timings.add("sdk_call", end - begin)
            return result

        return call
