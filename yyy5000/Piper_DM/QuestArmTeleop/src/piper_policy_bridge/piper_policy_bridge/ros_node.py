"""Non-blocking ROS bridge from wrist observations to safety-bounded policy targets."""

from __future__ import annotations

import argparse
from concurrent.futures import Future, ThreadPoolExecutor

import numpy as np

from .camera_sync import (
    NearestCameraSynchronizer,
    TimestampedImage,
    select_pair_state,
)
from .action_scheduler import AbsoluteActionScheduler
from .contract import ACTION_DIM, JOINT_NAMES, cameras_ready, state_by_name
from .ipc_client import PolicyClient
from .safety import Deadman, SafeTargetController, TrackingError


def main() -> None:  # pragma: no cover - live ROS graph integration
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument(
        "--shadow", action="store_true", help="Infer and log; never publish policy commands"
    )
    mode.add_argument(
        "--actuate",
        action="store_true",
        help="Allow output only while every safety gate is active",
    )
    parser.add_argument("--endpoint", default="ipc:///tmp/piper_lerobot.sock")
    parser.add_argument("--timeout-ms", type=int, default=200)
    parser.add_argument("--max-delta-scale", type=float, default=1.0)
    parser.add_argument(
        "--observation-profile",
        choices=("legacy-v2", "quest-tac-v1"),
        default="legacy-v2",
        help="checkpoint input contract; quest-tac-v1 is 7-D state and 640x480 RGB",
    )
    parser.add_argument(
        "--startup-hold",
        action="store_true",
        help="require /policy/start_control before inference and command publication",
    )
    arguments, ros_arguments = parser.parse_known_args()
    if arguments.shadow and arguments.startup_hold:
        parser.error("--startup-hold is only valid with --actuate")

    import cv2
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image, JointState
    from std_msgs.msg import Bool
    from std_srvs.srv import SetBool

    class PiperPolicyBridge(Node):
        def __init__(self) -> None:
            super().__init__("piper_policy_bridge")
            self.declare_parameter("wrist_color_topic", "/camera/wrist_camera/color/image_raw")
            self.declare_parameter("global_color_topic", "/camera/global_camera/color/image_raw")
            self.declare_parameter("max_camera_skew_s", 0.025)
            self.declare_parameter("max_state_image_skew_s", 0.05)
            self.declare_parameter("camera_timeout_s", 0.15)
            self.declare_parameter("feedback_topic", "/feedback/joint_states")
            self.declare_parameter("enabled_topic", "/policy/enabled")
            self.declare_parameter("command_topic", "/policy/joint_states")
            self.bridge = CvBridge()
            if arguments.observation_profile == "quest-tac-v1":
                policy_state_dim, policy_image_shape = 7, (480, 640, 3)
            else:
                policy_state_dim, policy_image_shape = 14, (240, 320, 3)
            self.client = PolicyClient(
                arguments.endpoint,
                arguments.timeout_ms,
                state_dim=policy_state_dim,
                image_shape=policy_image_shape,
            )
            self.controller = SafeTargetController(arguments.max_delta_scale)
            self.action_scheduler = AbsoluteActionScheduler(0.1)
            self.deadman = Deadman(0.25)
            # Do not use ``executor``: rclpy.Node owns that property for its
            # ROS executor and attempts to call add_node() on assigned values.
            self.ipc_executor = ThreadPoolExecutor(
                max_workers=1, thread_name_prefix="piper_policy_ipc"
            )
            self.future: Future | None = None
            self.request_id = 0
            self.latest_rgb: dict[str, np.ndarray | None] = {"wrist": None, "global": None}
            self.latest_image_stamp: dict[str, float | None] = {"wrist": None, "global": None}
            self.latest_image_arrival: dict[str, float | None] = {"wrist": None, "global": None}
            self.latest_state: np.ndarray | None = None
            self.latest_state_stamp: float | None = None
            self.latest_observation_state: np.ndarray | None = None
            self.latest_observation_state_stamp: float | None = None
            self.latest_state_image_skew: float | None = None
            self.last_feedback_time: float | None = None
            self.last_state_sync_warning = float("-inf")
            self.last_request_time: float | None = None
            self.last_applied_action_slot: int | None = None
            self.feedback_timeout_s = 0.15
            self.response_timeout_s = 0.25
            self.max_camera_skew_s = float(self.get_parameter("max_camera_skew_s").value)
            self.max_state_image_skew_s = float(
                self.get_parameter("max_state_image_skew_s").value
            )
            self.camera_timeout_s = float(self.get_parameter("camera_timeout_s").value)
            self.camera_synchronizer = NearestCameraSynchronizer(
                self.max_camera_skew_s
            )
            self.shadow = bool(arguments.shadow)
            self.execution_armed = not arguments.startup_hold
            self.hardware_control_active = not arguments.startup_hold
            self.tracking_fault = False
            self.control_gate_client = self.create_client(SetBool, "/control_enable")
            self.create_subscription(
                Image,
                self.get_parameter("wrist_color_topic").value,
                lambda message: self.on_image("wrist", message),
                qos_profile_sensor_data,
            )
            self.create_subscription(
                Image,
                self.get_parameter("global_color_topic").value,
                lambda message: self.on_image("global", message),
                qos_profile_sensor_data,
            )
            self.create_subscription(
                JointState, self.get_parameter("feedback_topic").value, self.on_feedback, 20
            )
            self.create_subscription(
                Bool, self.get_parameter("enabled_topic").value, self.on_enabled, 10
            )
            self.publisher = self.create_publisher(
                JointState, self.get_parameter("command_topic").value, 20
            )
            self.create_service(SetBool, "/policy/start_control", self.on_start_control)
            self.create_service(
                SetBool,
                "/policy/hardware_control_active",
                self.on_hardware_control_active,
            )
            self.create_timer(0.1, self.policy_tick)
            self.create_timer(0.02, self.control_tick)
            self.get_logger().info(
                f"Piper policy bridge ready: shadow={self.shadow}, endpoint={arguments.endpoint}, "
                f"max_delta_scale={arguments.max_delta_scale:g}, "
                f"observation_profile={arguments.observation_profile}, "
                f"startup_hold={arguments.startup_hold}, "
                f"max_camera_skew={self.max_camera_skew_s:g}s, "
                f"max_state_image_skew={self.max_state_image_skew_s:g}s"
            )

        def now_seconds(self) -> float:
            return self.get_clock().now().nanoseconds * 1e-9

        def feedback_fresh(self, now: float) -> bool:
            return (
                self.last_feedback_time is not None
                and now - self.last_feedback_time <= self.feedback_timeout_s
            )

        def all_gates_active(self, now: float) -> bool:
            return (
                self.execution_armed
                and not self.tracking_fault
                and self.deadman.active(now)
                and self.feedback_fresh(now)
            )

        def on_start_control(self, request, response):
            if self.shadow:
                response.success = False
                response.message = "shadow bridge cannot arm policy control"
                return response
            self.execution_armed = bool(request.data)
            if not self.execution_armed:
                self.hardware_control_active = False
                self.tracking_fault = False
            self.controller.clear()
            self.action_scheduler.clear()
            self.last_applied_action_slot = None
            response.success = True
            response.message = (
                "policy execution armed"
                if self.execution_armed
                else "policy execution held"
            )
            self.get_logger().info(response.message)
            return response

        def on_hardware_control_active(self, request, response):
            if self.shadow:
                response.success = False
                response.message = "shadow bridge has no hardware control"
                return response
            if request.data and not self.execution_armed:
                response.success = False
                response.message = "policy execution is still held"
                return response
            if request.data and self.tracking_fault:
                response.success = False
                response.message = "tracking fault is latched; restart deployment"
                return response
            self.hardware_control_active = bool(request.data)
            response.success = True
            response.message = (
                "absolute command cursor and tracking guard active"
                if self.hardware_control_active
                else "hardware command cursor held"
            )
            self.get_logger().info(response.message)
            return response

        def stop_for_tracking_fault(self, error: TrackingError) -> None:
            self.tracking_fault = True
            self.hardware_control_active = False
            self.controller.clear()
            self.action_scheduler.clear()
            self.last_applied_action_slot = None
            self.get_logger().error(f"closing hardware gate: {error}")
            request = SetBool.Request()
            request.data = False
            if self.control_gate_client.service_is_ready():
                self.control_gate_client.call_async(request)
            else:
                self.get_logger().error(
                    "/control_enable is unavailable; no further policy commands will publish"
                )

        def cameras_fresh_and_aligned(self, now: float) -> bool:
            return cameras_ready(
                now,
                self.latest_image_arrival,
                self.latest_image_stamp,
                self.camera_timeout_s,
                self.max_camera_skew_s,
            )

        def on_image(self, role: str, message) -> None:
            try:
                rgb = self.bridge.imgmsg_to_cv2(message, desired_encoding="rgb8")
                expected_height, expected_width = self.client.image_shape[:2]
                if rgb.shape[:2] != (expected_height, expected_width):
                    rgb = cv2.resize(
                        rgb,
                        (expected_width, expected_height),
                        interpolation=(
                            cv2.INTER_AREA
                            if rgb.shape[0] > expected_height or rgb.shape[1] > expected_width
                            else cv2.INTER_LINEAR
                        ),
                    )
                timestamp = (
                    float(message.header.stamp.sec)
                    + float(message.header.stamp.nanosec) * 1e-9
                )
                arrival = self.now_seconds()
                state = None if self.latest_state is None else self.latest_state.copy()
                for wrist, global_ in self.camera_synchronizer.push(
                    TimestampedImage(
                        role,
                        rgb,
                        timestamp,
                        arrival,
                        state,
                        self.latest_state_stamp,
                    )
                ):
                    pair_state = select_pair_state(
                        wrist,
                        global_,
                        self.max_state_image_skew_s,
                    )
                    if pair_state is None:
                        if arrival - self.last_state_sync_warning >= 2.0:
                            self.get_logger().warning(
                                "discarded camera pair: feedback state is unavailable "
                                "or exceeds the state/image skew limit"
                            )
                            self.last_state_sync_warning = arrival
                        continue
                    self.latest_rgb = {
                        "wrist": wrist.image,
                        "global": global_.image,
                    }
                    self.latest_image_stamp = {
                        "wrist": wrist.timestamp,
                        "global": global_.timestamp,
                    }
                    self.latest_image_arrival = {
                        "wrist": wrist.arrival,
                        "global": global_.arrival,
                    }
                    self.latest_observation_state = np.asarray(
                        pair_state.state[: self.client.state_dim], dtype=np.float32
                    ).copy()
                    self.latest_observation_state_stamp = pair_state.timestamp
                    self.latest_state_image_skew = pair_state.image_skew
            except Exception as error:
                self.get_logger().warning(f"discarded {role} frame: {error}")

        def on_feedback(self, message) -> None:
            state = state_by_name(message.name, message.position, message.velocity)
            if state is not None:
                self.latest_state = state
                self.latest_state_stamp = (
                    float(message.header.stamp.sec)
                    + float(message.header.stamp.nanosec) * 1e-9
                )
                self.last_feedback_time = self.now_seconds()

        def on_enabled(self, message) -> None:
            now = self.now_seconds()
            enabled = bool(message.data)
            was_active = self.deadman.active(now)
            self.deadman.update(enabled, now)
            if not enabled:
                self.controller.clear()
                self.action_scheduler.clear()
                self.last_applied_action_slot = None
                if was_active and self.future is None:
                    self.request_id += 1
                    self.future = self.ipc_executor.submit(self.client.reset, self.request_id)

        def accept_result(self, now: float) -> None:
            if self.future is None or not self.future.done():
                return
            future = self.future
            self.future = None
            try:
                result = future.result()
                if result is None:
                    return
                action_chunk, metadata = result
                if (
                    self.last_request_time is None
                    or now - self.last_request_time > self.response_timeout_s
                ):
                    raise TimeoutError("discarded stale policy response")
                if not self.all_gates_active(now):
                    return
                scheduled = self.action_scheduler.add_chunk(
                    action_chunk,
                    observation_timestamp=float(metadata["observation_timestamp"]),
                    received_timestamp=now,
                    policy_type=str(metadata["policy_type"]),
                    execution_steps=int(metadata["execution_steps"]),
                    temporal_ensemble_coeff=metadata.get("temporal_ensemble_coeff"),
                    period_s=float(metadata["period_s"]),
                )
                self.get_logger().debug(
                    f"chunk_steps={len(action_chunk)} accepted={scheduled.accepted_actions} "
                    f"expired={scheduled.expired_actions} "
                    f"committed_overlap={scheduled.committed_overlap_actions} slots="
                    f"{scheduled.first_slot}:{scheduled.last_slot} "
                    f"inference_ms={metadata.get('inference_ms')}"
                )
            except Exception as error:
                self.controller.clear()
                self.get_logger().error(f"policy request failed safely: {error}")

        def policy_tick(self) -> None:
            now = self.now_seconds()
            self.accept_result(now)
            if not self.all_gates_active(now) or not self.cameras_fresh_and_aligned(now):
                self.controller.clear()
                return
            if (
                self.future is not None
                or self.latest_rgb["wrist"] is None
                or self.latest_rgb["global"] is None
                or self.latest_observation_state is None
            ):
                return
            observation_timestamp = (
                float(self.latest_image_stamp["wrist"])
                + float(self.latest_image_stamp["global"])
            ) * 0.5
            if not self.action_scheduler.should_request(observation_timestamp, now):
                return
            if self.latest_observation_state_stamp is None:
                return
            self.request_id += 1
            self.last_request_time = now
            self.action_scheduler.note_request(observation_timestamp)
            self.future = self.ipc_executor.submit(
                self.client.infer,
                self.request_id,
                self.latest_observation_state.copy(),
                self.latest_rgb["wrist"].copy(),
                self.latest_rgb["global"].copy(),
                self.latest_image_stamp["wrist"],
                self.latest_image_stamp["global"],
                self.latest_observation_state_stamp,
            )

        def control_tick(self) -> None:
            now = self.now_seconds()
            if (
                self.shadow
                or not self.all_gates_active(now)
                or not self.cameras_fresh_and_aligned(now)
                or self.latest_state is None
            ):
                self.controller.clear()
                return
            if (
                self.future is not None
                and self.last_request_time is not None
                and now - self.last_request_time > self.response_timeout_s
            ):
                self.controller.clear()
                return
            current_slot = self.action_scheduler.slot(now)
            raw_action = self.action_scheduler.action_for_time(now)
            if raw_action is None:
                self.controller.clear()
                return
            if current_slot != self.last_applied_action_slot:
                safe_action, clipped = self.controller.set_target(
                    raw_action, self.latest_state[:ACTION_DIM]
                )
                self.last_applied_action_slot = current_slot
                self.get_logger().debug(
                    f"slot={current_slot} raw={raw_action.tolist()} "
                    f"safe={safe_action.tolist()} clipped={clipped}"
                )
            try:
                command = self.controller.tick(
                    self.latest_state[:ACTION_DIM],
                    advance=self.hardware_control_active,
                    enforce_tracking=self.hardware_control_active,
                )
            except TrackingError as error:
                self.stop_for_tracking_fault(error)
                return
            if command is None:
                return
            message = JointState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.name = list(JOINT_NAMES)
            message.position = command.tolist()
            self.publisher.publish(message)

        def close(self) -> None:
            self.controller.clear()
            if self.future is not None:
                self.future.cancel()
            self.ipc_executor.shutdown(wait=False, cancel_futures=True)

    rclpy.init(args=ros_arguments)
    node = PiperPolicyBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
