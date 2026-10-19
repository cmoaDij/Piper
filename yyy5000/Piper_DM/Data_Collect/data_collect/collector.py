"""Collection-only Piper driver, sensor workers and recording lifecycle."""

import argparse
import json
import os
import sys
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from .adapters.robot import JOINT_NAMES, LockedProxy, RobotObserver, TeleopTarget
from .capture import CaptureRuntime
from .config import ROOT, fingerprint, load_config, local_path
from .kinematics import ForwardKinematics, check_kinematics
from .motion import MotionLimiter, check_feedback_motion
from .storage import EpisodeManager
from .diagnostics import feedback_ages, require_feedback_fresh


def validate_calibration(cfg):
    calibration = json.loads(local_path(cfg["calibration_file"]).read_text(encoding="utf-8"))
    if (
        not calibration.get("verified")
        or calibration.get("calibration_id") != cfg["calibration_id"]
    ):
        raise ValueError("请核验 calibration_file 并设置 verified=true")
    for category, field, id_key in (
        ("cameras", "camera_serials", "serial"),
        ("tactile", "tactile_ids", "dev_id"),
    ):
        options = cfg[category]
        ids = [str(value[id_key]) for value in options.values()]
        if len(ids) != 2 or len(set(ids)) != 2:
            raise ValueError(f"{category} 需要两个不同设备")
        for name, value in options.items():
            if str(calibration[field][name]) != str(value[id_key]):
                raise ValueError(f"{name} 与标定中的设备不符")
    if not np.allclose(calibration["tcp_offset"], cfg["tcp_offset"], atol=1e-9):
        raise ValueError("TCP 偏移与标定不符")
    if calibration.get("arm_type") != cfg["arm_type"]:
        raise ValueError("标定中的 arm_type 与 Piper-X 配置不符")
    cfg["calibration"] = calibration


class SensorRuntime:
    def __init__(self, cfg, manager, observer, arm, gripper):
        self.cfg, self.manager, self.observer = cfg, manager, observer
        self.arm, self.gripper = arm, gripper
        self.stop_event = threading.Event()
        self.threads = []
        self.capture = CaptureRuntime(cfg, manager)

    def _start(self, name, fn, *args):
        def guarded():
            try:
                fn(*args)
            except Exception as exc:
                self.manager.fail(f"{name}: {exc}")

        thread = threading.Thread(target=guarded, name=name, daemon=True)
        self.threads.append(thread)
        thread.start()

    def start(self):
        self.capture.start()
        self._start("robot-feedback", self._poll)

    def _poll(self):
        deadline, last_good = time.monotonic(), time.monotonic()
        previous_ns = None
        while not self.stop_event.is_set():
            now_ns = time.monotonic_ns()
            if previous_ns is not None:
                self.observer.timings.add("feedback_interval", now_ns - previous_ns)
            previous_ns = now_ns
            if self.observer.poll(self.arm, self.gripper):
                last_good = time.monotonic()
            if time.monotonic() - last_good > 2:
                raise RuntimeError("机器人反馈超过 2 秒未就绪")
            deadline += 1.0 / self.cfg["feedback_hz"]
            delay = deadline - time.monotonic()
            # Skip missed deadlines rather than burst-polling cached packets.
            if delay < 0:
                deadline = time.monotonic() + 1.0 / self.cfg["feedback_hz"]
                delay = deadline - time.monotonic()
            self.stop_event.wait(max(0, delay))

    def close(self):
        self.stop_event.set()
        for thread in self.threads:
            thread.join(timeout=3)
        if any(thread.is_alive() for thread in self.threads):
            self.manager.fail("SDK worker 无法停止")
        self.capture.close()


def build_driver_class(base, cfg, manager, observer):
    """Keep original SDK/driver guards; accept only collection teleop inputs."""
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, String
    from std_srvs.srv import Trigger
    from rclpy.callback_groups import ReentrantCallbackGroup

    model = ET.parse(local_path(cfg["urdf"])).getroot()
    limits = {
        joint.get("name"): (
            float(joint.find("limit").get("lower")),
            float(joint.find("limit").get("upper")),
        )
        for joint in model.findall("joint")
        if joint.get("name") in JOINT_NAMES
    }
    if set(limits) != set(JOINT_NAMES):
        raise ValueError("URDF 缺少六轴限位")

    class CollectionDriver(base):
        def _load_parameters(self):
            super()._load_parameters()
            if self.arm_type != cfg["arm_type"] or self.effector_type != "agx_gripper":
                raise ValueError("驱动型号必须与已核对的 Piper-X 模型一致，末端必须为 AGX 夹爪")
            if self.fast_mode:
                raise ValueError("调试采集禁止 fast_mode=true，请使用带速度规划的 move_j")
            if self.control_enabled or not self.external_command_watchdog_enabled:
                raise ValueError("采集入口需要 control_enabled=false 和 watchdog=true")
            if not np.allclose(self.tcp_offset, cfg["tcp_offset"], atol=1e-9):
                raise ValueError("ROS 驱动 TCP 偏移与配置不符")

        def _init_effector(self):
            super()._init_effector()
            if self.gripper is None or self.arm_joint_count != 6:
                raise RuntimeError("六轴机械臂或 AGX 夹爪初始化失败")
            self.agx_arm = LockedProxy(self.agx_arm, observer)
            self.gripper._effector = LockedProxy(
                self.gripper._effector, observer, gripper=True, sdk_gripper=True
            )
            self.gripper = LockedProxy(self.gripper, observer, gripper=True)

        def _setup_subscribers(self):
            # No /control/joint_states publisher/subscriber and no policy input.
            self.create_subscription(JointState, "/teleop/joint_states", self.on_target, 20)
            self.create_subscription(Bool, "/teleop/enabled", self.on_enabled, 10)

        def __init__(self):
            self.targets = TeleopTarget(limits, cfg["command_timeout_s"], cfg["gripper_max_m"])
            self.pending_episode = False
            self.armed_ns = None
            self.aligned = False
            self.last_error = None
            self.motion = MotionLimiter(limits, cfg["motion"])
            self.gripper_input_reference = None
            self.gripper_actual_reference = None
            self.gripper_command = None
            self.homing = False
            self.preparing_home = False
            self.home_verified = False
            self.home_cancel = threading.Event()
            self.home_lock = threading.Lock()
            self.last_stop_ns = 0
            self.last_control_tick_ns = None
            super().__init__()
            self.runtime = SensorRuntime(cfg, manager, observer, self.agx_arm, self.gripper)
            self.runtime.start()
            self.status_pub = self.create_publisher(String, "/data_collect/status", 1)
            self.create_timer(0.1, self.publish_status)
            self.create_timer(1.0 / cfg["control_hz"], self.control_tick)
            self.create_timer(5.0, self.log_diagnostics)
            # Homing runs separately so B can still be handled during the move.
            self.home_callback_group = ReentrantCallbackGroup()
            self.create_service(Trigger, "/data_collect/prepare_home", self.prepare_home, callback_group=self.home_callback_group)

        def close_teleop(self, reason=None, *, fatal=False):
            was_controlling = self.control_enabled or manager.current is not None
            self.targets.set_enabled(False)
            self.pending_episode = False
            self.control_enabled = False
            self.aligned = False
            self.last_error = reason
            self.home_verified = False
            if self.external_command_watchdog is not None:
                self.external_command_watchdog.close()
            if was_controlling and not self.homing:
                self.hold_position()
            if reason:
                if fatal:
                    manager.fail(reason)
                else:
                    manager.reject(reason)
                self.get_logger().error(reason)
                self.log_diagnostics()
            else:
                path = manager.stop()
                if path:
                    if self.context.ok():
                        self.get_logger().info(f"Finalizing {path}")
                    else:
                        print(f"Finalizing {path}", flush=True)
            if hasattr(self, "status_pub"):
                self.publish_status()

        def checked_feedback(self, stationary=False):
            with manager.lock:
                feedback = manager.latest.get("robot_feedback")
                history = list(manager.feedback_preroll)
            now = time.monotonic_ns()
            require_feedback_fresh(feedback, now, cfg["audit"]["robot_red_ms"])
            check_kinematics(feedback, cfg["motion"])
            check_feedback_motion(history, cfg["motion"])
            if stationary:
                duration = cfg["motion"]["stationary_duration_s"]
                valid = [r for r in history if now - int(r["available_host_ts_ns"]) <= (duration + 0.02) * 1e9]
                if len(valid) < 2 or (int(valid[-1]["available_host_ts_ns"]) - int(valid[0]["available_host_ts_ns"])) / 1e9 < duration - 2 / cfg["feedback_hz"]:
                    raise ValueError("尚无足够静止反馈，请保持机械臂静止后重新按 A")
                if np.max(np.ptp([r["joint_pos"] for r in valid], axis=0)) > cfg["motion"]["stationary_tolerance_rad"]:
                    raise ValueError("机械臂尚未静止，拒绝 A 键对齐")
            return feedback

        def on_enabled(self, message):
            if not message.data:
                self.last_stop_ns = time.monotonic_ns()
                self.stop_home()
                self.close_teleop()
                return
            if self.targets.enabled:
                return
            if manager.failure:
                self.close_teleop(f"采集故障，请重启入口: {manager.failure}")
                return
            try:
                if self.homing or not self.home_verified:
                    raise ValueError("尚未完成本次 A 键回零确认，拒绝遥操作")
                feedback = self.checked_feedback(stationary=True)
                if not self.control_ready or not self.enable_flag:
                    raise ValueError("机械臂未使能或未就绪")
                self.motion.reset(feedback["joint_pos"], time.monotonic_ns())
            except Exception as exc:
                self.close_teleop(str(exc))
                return
            self.targets.set_enabled(True)
            # Homing/previous-session targets must not enter this episode's
            # command pre-roll or be paired with a new gripper command.
            with observer.lock:
                observer.target_joint = observer.target_gripper = None
                with manager.lock:
                    manager.latest.pop("robot_command", None)
            self.home_verified = False
            self.pending_episode = True
            self.aligned = False
            self.last_error = None
            self.gripper_input_reference = None
            self.gripper_actual_reference = float(feedback["gripper_pos"][0])
            self.gripper_command = self.gripper_actual_reference
            self.armed_ns = time.monotonic_ns()
            self.armed_ros_ns = self.get_clock().now().nanoseconds
            self.get_logger().info("A 键参考已锁定；等待首条近邻目标对齐，手柄不动时不下发运动")
            self.publish_status()

        def hold_position(self):
            try:
                with observer.lock:
                    packet = self.agx_arm.get_joint_angles()
                    if packet is not None and packet.hz > 0:
                        self.agx_arm.move_j(list(packet.msg))
            except Exception as exc:
                manager.fail(f"发送保持目标失败：{exc}")
                self.get_logger().error(f"发送保持目标失败：{exc}")

        def stop_home(self):
            # Cancellation never waits for SDK access during readiness waiting.
            self.home_cancel.set()
            if not self.homing:
                return
            with observer.lock:
                if self.homing:
                    self.hold_position()
                    self.home_verified = False
                    message = "回零已中止并发送当前位置保持目标"
                    if self.context.ok():
                        self.get_logger().warn(message)
                    else:
                        print(message, flush=True)

        def prepare_home(self, request, response):
            del request
            if not self.home_lock.acquire(blocking=False):
                response.success, response.message = False, "已有回零任务"
                return response
            self.home_verified = False
            speed_changed = False
            try:
                with observer.lock:
                    if time.monotonic_ns() - self.last_stop_ns < 250_000_000:
                        raise ValueError("刚收到 B 停止，请松开按键后重新按 A")
                    self.home_cancel.clear()
                if manager.failure:
                    raise ValueError(f"采集故障锁定，请重启入口：{manager.failure}")
                if self.targets.enabled or manager.current:
                    raise ValueError("请先按 B 停止当前遥操作，再按 A 回零")
                self.preparing_home = True
                self.last_error = None
                self.publish_status()
                self.get_logger().info("A 键准备：等待设备就绪，最多 1 秒；等待期间不下发运动指令，B 可取消")
                deadline = time.monotonic() + 1.0
                last_not_ready = "等待真实反馈"
                while True:
                    if self.home_cancel.is_set():
                        raise ValueError("等待设备就绪已由 B 取消")
                    if manager.failure:
                        raise ValueError(f"采集故障：{manager.failure}")
                    if time.monotonic() >= deadline:
                        raise ValueError(f"等待设备就绪超时（1 秒）：{last_not_ready}")
                    try:
                        manager.require_ready()
                        self.checked_feedback(stationary=True)
                        if time.monotonic() >= deadline:
                            raise ValueError("设备就绪检查完成时已超过 1 秒等待期限")
                        break
                    except (ValueError, RuntimeError) as exc:
                        last_not_ready = str(exc)
                    if time.monotonic() >= deadline:
                        raise ValueError(f"等待设备就绪超时（1 秒）：{last_not_ready}")
                    self.home_cancel.wait(0.005)
                if not self.control_ready or not self.enable_flag:
                    raise ValueError("机械臂未使能或未就绪，不能回零")
                self.close_teleop()
                with observer.lock:
                    if self.home_cancel.is_set():
                        raise ValueError("等待设备就绪已由 B 取消")
                    self.preparing_home = False
                    self.homing = True
                self.publish_status()
                try:
                    self.agx_arm.set_speed_percent(self.home_speed_percent)
                except Exception as exc:
                    raise RuntimeError(f"SDK 回零速度设置失败: {exc}") from exc
                speed_changed = True
                self.get_logger().info(f"A 键准备：以 {self.home_speed_percent}% 规划速度回零，回零完成前不录制")
                deadline = time.monotonic() + cfg["motion"]["home_timeout_s"]
                last_send, stable_since = float("-inf"), None
                while time.monotonic() < deadline:
                    if self.home_cancel.is_set():
                        raise ValueError("回零已由 B 取消")
                    feedback = self.checked_feedback()
                    if manager.failure:
                        raise ValueError(f"采集故障：{manager.failure}")
                    now = time.monotonic()
                    if now - last_send >= 0.2:
                        with observer.lock:
                            if self.home_cancel.is_set():
                                raise ValueError("回零已取消")
                            try:
                                self.agx_arm.move_j([0.0] * 6)
                            except Exception as exc:
                                raise RuntimeError(f"SDK 回零指令失败: {exc}") from exc
                        last_send = now
                    if np.max(np.abs(feedback["joint_pos"])) <= cfg["motion"]["home_tolerance_rad"]:
                        stable_since = now if stable_since is None else stable_since
                        if now - stable_since >= 0.3:
                            with observer.lock:
                                if self.home_cancel.is_set():
                                    raise ValueError("回零已取消")
                                self.home_verified = True
                            response.success = True
                            response.message = json.dumps({"tcp_pose": feedback["tcp_pose"].tolist(), "joint_pos": feedback["joint_pos"].tolist()})
                            self.get_logger().info("回零位置已稳定确认，允许锁定手柄与零位 TCP 参考")
                            return response
                    else:
                        stable_since = None
                    self.home_cancel.wait(0.02)
                raise ValueError("回零超时，拒绝进入遥操作")
            except Exception as exc:
                # Stop issuing zero targets before sending the hold command.
                cancelled = self.home_cancel.is_set()
                self.stop_home()
                if not cancelled and not isinstance(exc, ValueError):
                    manager.fail(f"回零设备/采集故障：{exc}")
                response.success, response.message = False, str(exc)
                self.last_error = None if cancelled else str(exc)
                if cancelled:
                    self.get_logger().warn(str(exc))
                else:
                    self.get_logger().error(str(exc))
                self.log_diagnostics()
                return response
            finally:
                self.homing = False
                self.preparing_home = False
                try:
                    if speed_changed:
                        self.agx_arm.set_speed_percent(self.speed_percent)
                except Exception as exc:
                    manager.fail(f"恢复规划速度失败：{exc}")
                    response.success, response.message = False, str(exc)
                finally:
                    self.home_lock.release()
                self.publish_status()

        def on_target(self, message):
            if not self.targets.enabled:
                return
            stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
            age = self.get_clock().now().nanoseconds - stamp
            if stamp < self.armed_ros_ns or not 0 <= age <= self.targets.timeout_ns:
                return
            if self.targets.enabled and not self.targets.update(
                message.name, message.position, time.monotonic_ns()
            ):
                self.close_teleop("遥操作目标字段、数值或限位非法")

        def control_tick(self):
            begin = time.monotonic_ns()
            if self.last_control_tick_ns is not None:
                observer.timings.add("control_interval", begin - self.last_control_tick_ns)
            self.last_control_tick_ns = begin
            try:
                self._control_tick()
            finally:
                observer.timings.add("control_tick", time.monotonic_ns() - begin)

        def _control_tick(self):
            if not self.targets.enabled:
                return
            if manager.failure:
                self.close_teleop(f"采集故障: {manager.failure}")
                return
            now = time.monotonic_ns()
            target = self.targets.snapshot(now)
            if target is None:
                if self.control_enabled or now - self.armed_ns > self.targets.timeout_ns:
                    ages = {
                        name: round((now - self.targets.timestamps[name]) / 1e6, 1) if name in self.targets.timestamps else None
                        for name in (*JOINT_NAMES, "gripper")
                    }
                    self.close_teleop(f"六轴或夹爪目标缺失/过期，目标年龄(ms)={ages}；本条轨迹淘汰，请 B 停止后重新按 A")
                return
            try:
                feedback = self.checked_feedback()
                now = time.monotonic_ns()
                if not self.aligned:
                    self.motion.check_first(target[:6], feedback["joint_pos"])
                    self.motion.reset(feedback["joint_pos"], now)
                    self.gripper_input_reference = target[6]
                    self.gripper_actual_reference = float(feedback["gripper_pos"][0])
                    self.gripper_command = self.gripper_actual_reference
                    self.aligned = True
                    self.get_logger().info("首条目标已通过近邻检查；对齐完成，未发送运动指令")
                    self.publish_status()
                    return
                previous_ns = self.motion.time_ns
                joints = self.motion.step(target[:6], feedback["joint_pos"], now)
                # A also clutches the trigger to the current physical opening.
                # Holding the trigger still at A cannot jump the gripper.
                grip_goal = float(np.clip(self.gripper_actual_reference + target[6] - self.gripper_input_reference, 0, cfg["gripper_max_m"]))
                dt = (now - previous_ns) / 1e9
                self.gripper_command += float(np.clip(grip_goal - self.gripper_command, -0.02 * dt, 0.02 * dt))
                if self.pending_episode and np.max(np.abs(np.asarray(joints) - feedback["joint_pos"])) < 1e-5 and abs(self.gripper_command - float(feedback["gripper_pos"][0])) < 1e-5:
                    return
                if self.pending_episode:
                    if not self.control_ready or not self.enable_flag:
                        self.close_teleop("机器人未使能或未就绪")
                        return
                    # Start immediately before the first real SDK command, avoiding
                    # an artificial command gap while initial IK is still computing.
                    path = manager.start()
                    self.pending_episode = False
                    self.control_enabled = True
                    self.external_command_watchdog.open(time.monotonic())
                    self.get_logger().info(f"Recording {path}")
                if not self.control_enabled:
                    self.close_teleop("驱动控制门已关闭，必须重新按 A")
                    return
                message = JointState()
                message.name = [*JOINT_NAMES, "gripper"]
                message.position = [*joints, self.gripper_command]
                # Same original driver's checks, followed by the only SDK send path.
                try:
                    super()._joint_states_callback(message)
                except Exception as exc:
                    raise RuntimeError(f"SDK 指令执行失败: {exc}") from exc
            except ValueError as exc:
                self.close_teleop(f"运动或反馈检查未通过: {exc}；请 B 停止后重新按 A")
            except Exception as exc:
                self.close_teleop(f"采集或 SDK 调用失败: {exc}；需排查后重启入口", fatal=True)

        def _control_gate_callback(self, request, response):
            if not request.data:
                self.stop_home()
                self.close_teleop()
                return super()._control_gate_callback(request, response)
            response.success = False
            response.message = "采集模式只允许 Quest A 在新目标就绪后打开遥操作门"
            return response

        def _move_home_callback(self, request, response):
            if manager.failure or self.homing or self.preparing_home:
                self.get_logger().error("采集故障锁定期间拒绝回零；请先退出并排查故障")
                return response
            if self.targets.enabled:
                self.close_teleop("回零请求中止当前遥操作录制")
            return super()._move_home_callback(request, response)

        def log_diagnostics(self):
            stats = observer.timings.snapshot()
            with manager.lock:
                ages = feedback_ages(manager.latest.get("robot_feedback", {}), time.monotonic_ns())
            capture = self.runtime.capture
            executor = self.executor
            executor_stats = executor.diagnostics() if hasattr(executor, "diagnostics") else None
            self.get_logger().info("时序诊断(ms): " + json.dumps({"feedback_age": {key: round(value, 2) for key, value in ages.items()}, "capture_pid": capture.process.pid, "capture_status_age": round((time.monotonic() - capture.last_status) * 1000, 2) if capture.last_status else None, "executor": executor_stats, "timings": stats}, ensure_ascii=False))

        def publish_status(self):
            if not self.context.ok():
                return
            with manager.lock:
                now = time.monotonic_ns()
                feedback = manager.latest.get("robot_feedback", {})
                streams = {}
                for name, stamps in manager.stream_times.items():
                    age = (now - stamps[-1]) / 1e6 if stamps else None
                    fps = (len(stamps) - 1) * 1e9 / (stamps[-1] - stamps[0]) if len(stamps) > 1 and stamps[-1] > stamps[0] else 0.0
                    streams[name] = {"age_ms": age, "fps": fps if age is not None and age < 1000 else 0.0}
                robot = {
                    key: feedback[key].tolist()
                    for key in ("joint_pos", "gripper_pos", "tcp_pose") if key in feedback
                }
                robot["joint_host_ts_ns"] = int(feedback.get("joint_available_host_ts_ns", 0))
                robot["feedback_age_ms"] = feedback_ages(feedback, now)
                robot["age_ms"] = (now - int(feedback["joint_available_host_ts_ns"])) / 1e6 if feedback else None
                command = manager.latest.get("robot_command", {})
                robot["target_joint_pos"] = command["target_joint_pos"].tolist() if "target_joint_pos" in command else None
                robot["target_gripper_pos"] = command["target_gripper_pos"].tolist() if "target_gripper_pos" in command else None
                writer = manager.current
                finalizing = [item for item in manager.finished if item.thread.is_alive()]
                active_writers = ([writer] if writer else []) + finalizing
                queue_bytes, worst_fill = 0, 0.0
                for item in active_writers:
                    with item.lock:
                        queue_bytes += item.pending_bytes
                        worst_fill = max(worst_fill, item.pending_bytes / item.max_bytes)
                message = String()
                message.data = json.dumps(
                    {
                        "recording": str(manager.current.path) if manager.current else None,
                        "armed": self.pending_episode,
                        "aligned": self.aligned,
                        "homing": self.homing,
                        "preparing_home": self.preparing_home,
                        "teleop_enabled": self.targets.enabled,
                        "arm_enabled": self.enable_flag,
                        "control_ready": self.control_ready,
                        "pid": os.getpid(),
                        "robot": robot,
                        "streams": streams,
                        "writer": {"pending_mb": queue_bytes / 1024**2, "capacity_mb": cfg["queue_max_mb"] * max(1, len(active_writers)), "finalizing": len(finalizing), "worst_fill": worst_fill},
                        "recording_duration_s": (now - writer.metadata["start_host_ts_ns"]) / 1e9 if writer else 0.0,
                        "status_ros_ns": self.get_clock().now().nanoseconds,
                        "error": manager.failure or self.last_error or [w.error for w in manager.finished if w.error],
                        "fk_position_error_m": float(manager.latest.get("robot_feedback", {}).get("fk_position_error_m", 0.0)),
                        "fk_rotation_error_rad": float(manager.latest.get("robot_feedback", {}).get("fk_rotation_error_rad", 0.0)),
                        "ready_streams": sorted(manager.latest),
                    }
                )
            self.status_pub.publish(message)

    return CollectionDriver


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="config/collection.yaml")
    args, ros_args = parser.parse_known_args()
    cfg = load_config(args.config)
    os.chdir(ROOT)
    validate_calibration(cfg)
    if sys.version_info >= (3, 12):
        raise RuntimeError(
            "本地 Tac-X SDK 不支持 Python 3.12；请使用与 ROS ABI 一致的 3.10/3.11 环境"
        )
    sys.path.insert(0, str(ROOT / "vendor/dm_sdk"))
    sys.path.insert(0, str(ROOT / "vendor/pyagxarm"))
    import agx_arm_ctrl.agx_arm_ctrl_single_node as driver
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from .executor import TeleopExecutor

    if not Path(driver.__file__).resolve().is_relative_to(ROOT / "ros_ws"):
        raise RuntimeError("请先 source Data_Collect/ros_ws/install/setup.bash，不能使用外部驱动")
    cfg["urdf_sha256"] = fingerprint(local_path(cfg["urdf"]))
    fk = ForwardKinematics(
        local_path(cfg["urdf"]), cfg["base_link"], cfg["tip_link"], cfg["tcp_offset"]
    )
    manager = EpisodeManager(cfg, fk)
    observer = RobotObserver(manager, fk)
    node_class = build_driver_class(driver.AgxArmRosNode, cfg, manager, observer)
    rclpy.init(args=ros_args)
    node = None
    executor = None
    cleanup_failed = False
    try:
        node = node_class()
        executor = TeleopExecutor()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if node:
                node.publisher_stop.set()
                node.stop_home()
                node.close_teleop()
                try:
                    if executor is not None:
                        if not executor.shutdown(timeout_sec=3):
                            node.get_logger().warn("等待回调线程退出；回零已请求取消")
                            executor.shutdown()
                finally:
                    try:
                        node.runtime.close()
                    except RuntimeError as exc:
                        cleanup_failed = True
                        node.get_logger().error(f"采集资源已清理，存在设备/写盘故障：{exc}")
                    if manager.failure and not cleanup_failed:
                        cleanup_failed = True
                        node.get_logger().error(f"会话存在设备/采集故障：{manager.failure}")
        finally:
            rclpy.try_shutdown()
            if node:
                node.publisher_thread.join(timeout=2)
                node.agx_arm.disconnect()
                node.destroy_node()
    if cleanup_failed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
