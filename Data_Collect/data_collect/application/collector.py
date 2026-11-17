"""Piper teleoperation driver with optional sensors and episode recording."""

import argparse
import json
import os
import sys
import threading
import time
from pathlib import Path

import numpy as np

from data_collect.control.robot import JOINT_NAMES, LockedProxy, RobotObserver, TeleopTarget
from data_collect.common.config import ROOT, fingerprint, load_config, local_path
from data_collect.control.kinematics import ForwardKinematics
from data_collect.recording.storage import EpisodeManager
from data_collect.common.diagnostics import feedback_ages


def validate_calibration(cfg):
    from data_collect.calibration.tracking import load_tracking_latency

    load_tracking_latency(cfg)
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
        required = 2 if category == "tactile" else len(options)
        if not required or len(ids) != required or len(set(ids)) != required:
            raise ValueError(f"{category} 需要 {required} 个不同设备")
        if set(calibration[field]) != set(options):
            raise ValueError(f"{category} 相机位置/设备清单与标定不符")
        for name, value in options.items():
            if str(calibration[field][name]) != str(value[id_key]):
                raise ValueError(f"{name} 与标定中的设备不符")
    from data_collect.sensors.camera_selection import load_gopro_latency

    for camera in cfg["cameras"].values():
        if camera.get("backend") == "gopro":
            load_gopro_latency(camera)
    if not np.allclose(calibration["tcp_offset"], cfg["tcp_offset"], atol=1e-9):
        raise ValueError("TCP 偏移与标定不符")
    if calibration.get("arm_type") != cfg["arm_type"]:
        raise ValueError("标定中的 arm_type 与 Piper-X 配置不符")
    cfg["calibration"] = calibration


class SensorRuntime:
    def __init__(self, cfg, manager, observer, arm, gripper, capture_enabled=True):
        self.cfg, self.manager, self.observer = cfg, manager, observer
        self.arm, self.gripper = arm, gripper
        self.stop_event = threading.Event()
        self.threads = []
        self.capture = None
        if capture_enabled and not cfg.get("camera_selection_error"):
            from data_collect.sensors.capture import CaptureRuntime

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
        if self.capture is not None:
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
        if self.capture is not None:
            self.capture.close()


def build_driver_class(base, cfg, manager, observer, *, teleop_only=False, robot_latency_only=False):
    """Use the reference driver's motion path; keep recording independent."""
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool, String

    class CollectionDriver(base):
        def _load_parameters(self):
            super()._load_parameters()
            if cfg.get("robot_tracking_latency"):
                from data_collect.calibration.tracking import validate_tracking_settings

                actual = {**cfg, "arm_type": self.arm_type, "tcp_offset": list(self.tcp_offset),
                          "teleop": {**cfg["teleop"], "fast_mode": self.fast_mode,
                                     "speed_percent": self.speed_percent}}
                validate_tracking_settings(cfg["robot_tracking_latency"], actual)

        def _init_effector(self):
            super()._init_effector()
            self.agx_arm = LockedProxy(self.agx_arm, observer)
            if self.gripper is not None:
                self.gripper._effector = LockedProxy(
                    self.gripper._effector, observer, gripper=True, sdk_gripper=True
                )
                self.gripper = LockedProxy(self.gripper, observer, gripper=True)

        def _setup_subscribers(self):
            # Only this driver owns the SDK; no second command outlet or policy input.
            self.create_subscription(JointState, "/teleop/joint_states", self.on_target, 20)
            self.create_subscription(Bool, "/teleop/enabled", self.on_enabled, 10)
            self.create_subscription(Bool, "/teleop/following", self.on_following, 10)

        def __init__(self):
            self.targets = TeleopTarget()
            self.pending_episode = False
            self.recording_resume_pending = False
            self.aligned = False
            self.last_error = None
            self.recording_error = None
            self.homing = False
            self.following = False
            self.follow_started_ns = 0
            self.last_sent_joint = None
            self.reference_key = ""
            self.reference_pending = False
            self.arm_command_started = False
            self.reported_files = set()
            super().__init__()
            if not teleop_only and manager.failure:
                self.get_logger().error(f"录制未启动：{manager.failure}；遥操可继续，但当前不会生成数据文件")
            self.runtime = SensorRuntime(
                cfg, manager, observer, self.agx_arm, self.gripper,
                capture_enabled=not teleop_only and not robot_latency_only,
            )
            try:
                self.runtime.start()
            except Exception as exc:
                manager.fail(f"采集启动失败：{exc}")
                self.get_logger().error(f"录制不可用，遥操仍可使用：{exc}")
            self.status_pub = self.create_publisher(String, "/data_collect/status", 1)
            self.ik_reference_pub = self.create_publisher(JointState, "/teleop/ik_reference", 10)
            self.create_timer(0.1, self.publish_status)
            self.create_timer(1.0 / cfg["control_hz"], self.control_tick)
            self.create_timer(5.0, self.log_diagnostics)

        def close_teleop(self):
            writer = manager.current
            self.following = False
            with observer.lock:
                observer.following = False
            self.targets.set_enabled(False)
            self.last_sent_joint = None
            self.reference_pending = False
            self.arm_command_started = False
            self.pending_episode = False
            self.recording_resume_pending = False
            self.aligned = False
            path = manager.stop()
            if path and self.context.ok():
                duration = writer.timeline.duration_s(time.monotonic_ns()) if writer.timeline else 0
                self.get_logger().info(f"[采集] 结束 {Path(path).name} | 有效 {duration:.2f} 秒 | 正在封盘")
            if hasattr(self, "status_pub"):
                self.publish_status()

        def on_enabled(self, message):
            if not message.data:
                self.close_teleop()
                return
            if self.targets.enabled:
                return
            self.targets.set_enabled(True)
            self.pending_episode = not teleop_only
            self.aligned = True
            self.arm_command_started = False
            self.last_error = None
            self.recording_error = None
            with observer.lock:
                # In particular, retain the home command (six zeros), rather
                # than turning its small measured tracking error into a new goal.
                previous_command = getattr(observer, "target_joint", None)
                if previous_command is not None:
                    self.last_sent_joint = np.asarray(previous_command, dtype=float).copy()
                observer.target_joint = observer.target_gripper = None
                with manager.lock:
                    manager.latest.pop("robot_command", None)
            self.hold_arm_target()
            if self.following:
                self.publish_ik_reference()
            self.get_logger().info("[采集] A 已对齐，等待 RG；尚未录制")
            self.publish_status()

        def hold_arm_target(self):
            """Freeze the last sent setpoint; never ratchet it towards feedback."""
            with observer.lock:
                joint = getattr(observer, "target_joint", None)
            if joint is None:
                joint = self.last_sent_joint
            if joint is None:
                with manager.lock:
                    joint = manager.latest.get("robot_feedback", {}).get("joint_pos")
            for name in JOINT_NAMES:
                self.targets.values.pop(name, None)
            if joint is not None:
                self.targets.update(JOINT_NAMES, joint)

        def publish_ik_reference(self):
            self.reference_pending = not all(name in self.targets.values for name in JOINT_NAMES)
            if self.reference_pending:
                return
            reference = JointState()
            self.follow_started_ns = self.get_clock().now().nanoseconds
            self.reference_key = str(self.follow_started_ns)
            reference.header.frame_id = self.reference_key
            reference.header.stamp.sec, reference.header.stamp.nanosec = divmod(
                self.follow_started_ns, 1_000_000_000
            )
            reference.name = list(JOINT_NAMES)
            reference.position = [self.targets.values[name] for name in JOINT_NAMES]
            self.ik_reference_pub.publish(reference)

        def on_following(self, message):
            following = bool(message.data)
            if following == self.following:
                return
            self.following = following
            with observer.lock:
                observer.following = following
            if following:
                # IK preserves its input Pose timestamp. Discard a solution still
                # in flight from before this resume, without an age threshold.
                self.hold_arm_target()
                self.publish_ik_reference()
                # Missing startup frames/baselines can recover while teleop
                # continues. A new RG press is an explicit retry; a latched
                # device/writer/calibration fault still blocks recording.
                with manager.lock:
                    retry_allowed = manager.failure is None
                self.recording_resume_pending = not teleop_only and retry_allowed
                if self.recording_resume_pending:
                    self.recording_error = None
            elif self.targets.enabled:
                self.reference_pending = False
                self.hold_arm_target()
                self.recording_resume_pending = False
                if not teleop_only:
                    manager.pause()
                    writer = manager.current
                    if writer:
                        duration = writer.timeline.duration_s(time.monotonic_ns()) if writer.timeline else 0
                        self.get_logger().info(f"[采集] 暂停 {writer.path.name} | 有效 {duration:.2f} 秒")
            with manager.lock:
                actual = manager.latest.get("robot_feedback", {}).get("joint_pos")
            self.get_logger().info("RG 切换参考: " + json.dumps({
                "following": following,
                "command_joint_rad": [self.targets.values.get(name) for name in JOINT_NAMES],
                "feedback_joint_rad": np.asarray(actual).tolist() if actual is not None else None,
            }))
            self.publish_status()

        def on_target(self, message):
            if len(message.name) != len(message.position):
                return
            stamp = message.header.stamp
            stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
            accept_arm = (
                self.following and not self.reference_pending
                and stamp_ns >= self.follow_started_ns
                and message.header.frame_id == self.reference_key
            )
            pairs = [(name, position) for name, position in zip(message.name, message.position)
                     if name == "gripper" or accept_arm]
            self.targets.update([name for name, _ in pairs], [position for _, position in pairs])
            if accept_arm:
                received = {name for name, position in pairs
                            if name in JOINT_NAMES and np.isfinite(position)}
                if received == set(JOINT_NAMES):
                    self.arm_command_started = True

        def control_tick(self):
            begin = time.monotonic_ns()
            try:
                if self.targets.enabled and self.following and self.reference_pending:
                    self.hold_arm_target()
                    self.publish_ik_reference()
                if self.targets.enabled and not self.following and any(
                    name not in self.targets.values for name in JOINT_NAMES
                ):
                    self.hold_arm_target()
                target = self.targets.snapshot()
                # A only captures a reference. Sending feedback as a move_js
                # target here would enter fast control before RG was pressed.
                # On first RG, wait for a complete solution from the new IK
                # reference. Later pauses keep the already active hold target.
                if target is None or not self.arm_command_started:
                    return
                if self.following and (self.pending_episode or self.recording_resume_pending):
                    self.pending_episode = False
                    self.recording_resume_pending = False
                    try:
                        resuming = manager.current is not None
                        path = manager.start()
                        self.get_logger().info(f"[采集] {'恢复' if resuming else '开始'} Recording {path}")
                    except Exception as exc:
                        self.recording_error = str(exc)
                        with manager.lock:
                            retry_allowed = manager.failure is None
                        hint = ("传感器就绪后松开并重新按住 RG 重试"
                                if retry_allowed else "采集故障已锁定，处理后重启采集程序")
                        self.get_logger().error(f"无法开始录制，遥操继续：{exc}；{hint}")
                message = JointState()
                message.name = [*JOINT_NAMES, "gripper"]
                message.position = target
                # No additional trajectory, feedback-age, FK or motion checks.
                super()._joint_states_callback(message)
                with observer.lock:
                    sent = getattr(observer, "target_joint", None)
                    if sent is not None:
                        self.last_sent_joint = np.asarray(sent, dtype=float).copy()
            finally:
                observer.timings.add("control_tick", time.monotonic_ns() - begin)

        def _control_gate_callback(self, request, response):
            if not request.data:
                self.close_teleop()
            return super()._control_gate_callback(request, response)

        def _move_home_callback(self, request, response):
            self.close_teleop()
            self.homing = True
            self.publish_status()
            try:
                return super()._move_home_callback(request, response)
            finally:
                self.homing = False
                self.publish_status()

        def log_diagnostics(self):
            stats = observer.timings.snapshot()
            with manager.lock:
                ages = feedback_ages(manager.latest.get("robot_feedback", {}), time.monotonic_ns())
            capture = self.runtime.capture
            executor = self.executor
            executor_stats = executor.diagnostics() if hasattr(executor, "diagnostics") else None
            self.get_logger().info("时序诊断(ms): " + json.dumps({"feedback_age": {key: round(value, 2) for key, value in ages.items()}, "capture_pid": capture.process.pid if capture is not None else None, "capture_status_age": round((time.monotonic() - capture.last_status) * 1000, 2) if capture is not None and capture.last_status else None, "executor": executor_stats, "timings": stats}, ensure_ascii=False))
            with manager.lock:
                now = time.monotonic_ns()
                writer = manager.current
                if writer:
                    duration = writer.timeline.duration_s(now) if writer.timeline else (now - writer.metadata["start_host_ts_ns"]) / 1e9
                    phase = "暂停" if writer.timeline and not writer.timeline.active else "录制"
                    state = f"{phase} {writer.path.name} | 有效 {duration:.1f} 秒"
                else:
                    state = "未录制"
                try:
                    if teleop_only:
                        from data_collect.common.diagnostics import require_feedback_fresh

                        require_feedback_fresh(manager.latest.get("robot_feedback"), now, cfg["audit"]["robot_red_ms"])
                    else:
                        manager.require_ready()
                    readiness = "传感器就绪" if not teleop_only else "机器人反馈就绪"
                except (RuntimeError, ValueError) as exc:
                    readiness = "未就绪: " + str(exc)
                labels = {"cameras/wrist_rgb": "手腕", "cameras/global_rgb": "全局",
                          "tactile/left_raw": "左触觉", "tactile/right_raw": "右触觉"}
                rates = []
                for stream, stamps in manager.stream_times.items():
                    limit = cfg["audit"]["rgb_red_ms" if stream.startswith("cameras/") else "tactile_red_ms"]
                    fresh = bool(stamps) and 0 <= now - stamps[-1] <= limit * 1e6
                    fps = (len(stamps) - 1) * 1e9 / (stamps[-1] - stamps[0]) if len(stamps) > 1 and stamps[-1] > stamps[0] else 0
                    rates.append(f"{labels.get(stream, stream)} {fps:.0f}fps" if fresh else f"{labels.get(stream, stream)} 未就绪")
                age = f"反馈 {max(ages.values()):.0f}ms" if ages else "无机器人反馈"
                self.get_logger().info("[状态] " + " | ".join([state, readiness, *rates, age]))
                for finished in manager.finished:
                    if finished.path in self.reported_files or finished.thread.is_alive():
                        continue
                    self.reported_files.add(finished.path)
                    if finished.error or not finished.path.exists():
                        self.get_logger().error(f"封盘失败 {finished.path.name}: {finished.error or '文件未生成'}")
                    else:
                        self.get_logger().info(f"[采集] 已保存 {finished.path}")

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
                        "recording_paused": bool(writer and writer.timeline and not writer.timeline.active),
                        "armed": self.pending_episode,
                        "aligned": self.aligned,
                        "homing": self.homing,
                        "teleop_enabled": self.targets.enabled,
                        "following": self.following and self.targets.enabled,
                        "teleop_only": teleop_only,
                        "robot_latency_only": robot_latency_only,
                        "robot_tracking_latency": cfg.get("robot_tracking_latency"),
                        "arm_enabled": self.enable_flag,
                        "control_ready": self.control_ready,
                        "pid": os.getpid(),
                        "robot": robot,
                        "streams": streams,
                        "cameras": {name: {"backend": opts.get("backend", "realsense"), "serial": opts["serial"]}
                                    for name, opts in cfg["cameras"].items()},
                        "writer": {"pending_mb": queue_bytes / 1024**2, "capacity_mb": cfg["queue_max_mb"] * max(1, len(active_writers)), "finalizing": len(finalizing), "worst_fill": worst_fill},
                        "recording_duration_s": (writer.timeline.duration_s(now) if writer.timeline else
                                                 (now - writer.metadata["start_host_ts_ns"]) / 1e9) if writer else 0.0,
                        "status_ros_ns": self.get_clock().now().nanoseconds,
                        "error": self.last_error,
                        "recording_error": manager.failure or self.recording_error or [w.error for w in manager.finished if w.error],
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
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--teleop-only", action="store_true", help="只运行遥操和机器人反馈，不启动相机、触觉或录制")
    mode.add_argument("--robot-latency-only", action="store_true", help="仅录制机器人指令/反馈，不启动相机或触觉，不生成训练数据")
    args, ros_args = parser.parse_known_args()
    cfg = load_config(args.config)
    os.chdir(ROOT)
    calibration_error = None
    if args.robot_latency_only:
        cfg["cameras"] = {}
        cfg["tactile"] = {}
        cfg["camera_mode"] = "disabled"
        cfg["recording_purpose"] = "robot_tracking_latency"
    if not args.teleop_only and not args.robot_latency_only:
        try:
            from data_collect.sensors.camera_selection import resolve_cameras

            try:
                resolve_cameras(cfg)
            except (OSError, ValueError, RuntimeError) as exc:
                cfg["camera_selection_error"] = str(exc)
                raise
            validate_calibration(cfg)
        except (OSError, ValueError, KeyError, RuntimeError) as exc:
            calibration_error = f"录制标定尚未就绪：{exc}"
    if sys.version_info >= (3, 12):
        raise RuntimeError(
            "本地 Tac-X SDK 不支持 Python 3.12；请使用与 ROS ABI 一致的 3.10/3.11 环境"
        )
    sys.path.insert(0, str(ROOT / "vendor/dm_sdk"))
    sys.path.insert(0, str(ROOT / "vendor/pyagxarm"))
    import agx_arm_ctrl.agx_arm_ctrl_single_node as driver
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from data_collect.common.executor import TeleopExecutor

    if not Path(driver.__file__).resolve().is_relative_to(ROOT / "ros_ws"):
        raise RuntimeError("请先 source Data_Collect/ros_ws/install/setup.bash，不能使用外部驱动")
    cfg["urdf_sha256"] = fingerprint(local_path(cfg["urdf"]))
    fk = ForwardKinematics(
        local_path(cfg["urdf"]), cfg["base_link"], cfg["tip_link"], cfg["tcp_offset"]
    )
    if args.robot_latency_only:
        from data_collect.calibration.robot_latency import RobotLatencyManager

        manager = RobotLatencyManager(cfg, fk)
    else:
        manager = EpisodeManager(cfg, fk)
    if calibration_error:
        manager.fail(calibration_error)
    observer = RobotObserver(manager, fk, tracking_latency=cfg.get("robot_tracking_latency"))
    node_class = build_driver_class(
        driver.AgxArmRosNode, cfg, manager, observer, teleop_only=args.teleop_only,
        robot_latency_only=args.robot_latency_only,
    )
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
                node.close_teleop()
                try:
                    if executor is not None:
                        if not executor.shutdown(timeout_sec=3):
                            node.get_logger().warn("等待回调线程退出")
                            executor.shutdown()
                finally:
                    try:
                        try:
                            node.runtime.close()
                        finally:
                            if args.robot_latency_only:
                                manager.close()
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
