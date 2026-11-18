#!/usr/bin/env python3
import math
import json
import time
from typing import Any

import numpy as np
import rclpy
from button_edge import RisingEdge
from geometry_msgs.msg import PoseStamped
from gripper_mapping import gripper_position_from_trigger
from oculus_reader import OculusReader
from rclpy.executors import ExternalShutdownException
from data_collect.common.executor import TeleopExecutor
from data_collect.control.quest_world import WorldInputState
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Header, String
from std_srvs.srv import Empty
from transformations import euler_from_quaternion


def xyzrpy_to_mat(
    x: float,
    y: float,
    z: float,
    roll: float,
    pitch: float,
    yaw: float,
) -> np.ndarray:
    mat = np.eye(4)
    mat[:3, :3] = Rotation.from_euler("xyz", [roll, pitch, yaw]).as_matrix()
    mat[:3, 3] = np.array([x, y, z])
    return mat


def mat2xyzquat(matrix: np.ndarray):
    pos = matrix[:3, 3]
    rotation_matrix = matrix[:3, :3]
    quat = Rotation.from_matrix(rotation_matrix).as_quat()
    return pos, quat


def calc_pose_incre(
    start_pose_matrix: np.ndarray,
    current_pose_matrix,
    zero_matrix: np.ndarray,
    translation_scale: np.ndarray,
    delta_frame: str = "tcp",
):
    end_matrix = xyzrpy_to_mat(
        current_pose_matrix[0],
        current_pose_matrix[1],
        current_pose_matrix[2],
        current_pose_matrix[3],
        current_pose_matrix[4],
        current_pose_matrix[5],
    )
    if delta_frame == "base":
        # Both input poses already use the reference Quest -> ROS world axes.
        # Rebase position and orientation separately: a full SE(3) conjugation
        # would rotate translation by the arbitrary hand/TCP orientations at A.
        result_matrix = zero_matrix.copy()
        result_matrix[:3, 3] += translation_scale * (end_matrix[:3, 3] - start_pose_matrix[:3, 3])
        result_matrix[:3, :3] = end_matrix[:3, :3] @ start_pose_matrix[:3, :3].T @ zero_matrix[:3, :3]
        return mat2xyzquat(result_matrix)
    if delta_frame != "tcp":
        raise ValueError("delta_frame must be base or tcp")
    relative_matrix = np.dot(np.linalg.inv(start_pose_matrix), end_matrix)
    # Scale the controller displacement in its start-frame axes before mapping
    # it into the current TCP frame.  Rotation mapping is deliberately unchanged.
    relative_matrix[:3, 3] *= translation_scale
    result_matrix = np.dot(zero_matrix, relative_matrix)
    return mat2xyzquat(result_matrix)


class RosOperator(Node):
    def __init__(self):
        super().__init__("pub_delta_pose_node")

        # Topic parameters
        self.declare_parameter("handle_pose_topic", "/right_handle_pose")
        self.declare_parameter("feedback_tcp_pose_topic", "/feedback/tcp_pose")
        self.declare_parameter("delta_pose_topic", "/delta_pose")
        self.declare_parameter("control_joint_topic", "/control/joint_states")
        self.declare_parameter("teleop_enabled_topic", "/teleop/enabled")
        self.declare_parameter("following_topic", "/teleop/following")
        self.declare_parameter("move_home_service", "/move_home")

        # Button mapping parameters
        dynamic_string_param = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter("start_button", "A", dynamic_string_param)
        self.declare_parameter("stop_button", "B", dynamic_string_param)
        self.declare_parameter("reset_button", "RJ", dynamic_string_param)
        self.declare_parameter("follow_button", "RG", dynamic_string_param)
        self.declare_parameter("trigger_axis", "rightTrig")

        # Gripper/control parameters
        self.declare_parameter("gripper_joint_name", "gripper")
        self.declare_parameter("gripper_max_range", 0.07)
        self.declare_parameter("control_rate_hz", 30.0)
        self.declare_parameter("hand_name", "right")
        self.declare_parameter("translation_scale", [1.0, 1.0, 1.0])
        self.declare_parameter("delta_frame", "tcp")
        self.declare_parameter("coordinate_mode", "relative")
        self.declare_parameter("auto_home_on_stop", True)
        self.declare_parameter("auto_home_delay_s", 0.5)
        # Empty means that OculusReader selects an ADB USB device.  Keep the
        # existing Wi-Fi endpoint as the default for the original launch files.
        self.declare_parameter("oculus_ip", "192.168.5.32")

        handle_pose_topic = str(self.get_parameter("handle_pose_topic").value)
        feedback_tcp_pose_topic = str(
            self.get_parameter("feedback_tcp_pose_topic").value
        )
        delta_pose_topic = str(self.get_parameter("delta_pose_topic").value)
        control_joint_topic = str(self.get_parameter("control_joint_topic").value)
        self.teleop_enabled_topic = str(
            self.get_parameter("teleop_enabled_topic").value
        )
        move_home_service = str(self.get_parameter("move_home_service").value)

        self.start_button = self._normalize_button_name(
            self.get_parameter("start_button").value, "start_button"
        )
        self.stop_button = self._normalize_button_name(
            self.get_parameter("stop_button").value, "stop_button"
        )
        self.reset_button = self._normalize_button_name(
            self.get_parameter("reset_button").value, "reset_button"
        )
        self.follow_button = self._normalize_button_name(
            self.get_parameter("follow_button").value, "follow_button"
        )
        if len({self.start_button, self.stop_button, self.reset_button, self.follow_button}) != 4:
            raise ValueError(
                "start_button, stop_button, reset_button, and follow_button must be distinct."
            )
        self.start_edge = RisingEdge()
        self.stop_edge = RisingEdge()
        self.reset_edge = RisingEdge()
        self.trigger_axis = str(self.get_parameter("trigger_axis").value)

        self.gripper_joint_name = str(self.get_parameter("gripper_joint_name").value)
        self.gripper_max_range = float(self.get_parameter("gripper_max_range").value)
        if (
            not math.isfinite(self.gripper_max_range)
            or self.gripper_max_range < 0.0
        ):
            raise ValueError(
                "gripper_max_range must be a finite value greater than or equal to zero."
            )
        control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.hand_name = str(self.get_parameter("hand_name").value)
        self.delta_frame = str(self.get_parameter("delta_frame").value)
        self.coordinate_mode = str(self.get_parameter("coordinate_mode").value)
        if self.coordinate_mode not in ("relative", "world"):
            raise ValueError("coordinate_mode must be relative or world")
        if self.coordinate_mode == "world" and self.delta_frame != "base":
            raise ValueError("世界坐标模式必须使用 delta_frame=base")
        if self.delta_frame not in ("base", "tcp"):
            raise ValueError("delta_frame must be base or tcp")
        self.translation_scale = np.asarray(
            self.get_parameter("translation_scale").value, dtype=float
        )
        if self.translation_scale.shape != (3,) or not np.all(np.isfinite(self.translation_scale)):
            raise ValueError("translation_scale must contain exactly three finite values.")
        if np.any(self.translation_scale <= 0.0):
            raise ValueError("translation_scale values must be greater than zero.")
        self.auto_home_on_stop = bool(
            self.get_parameter("auto_home_on_stop").value
        )
        self.auto_home_delay_s = float(
            self.get_parameter("auto_home_delay_s").value
        )
        if not math.isfinite(self.auto_home_delay_s) or self.auto_home_delay_s <= 0.0:
            raise ValueError("auto_home_delay_s must be finite and positive.")
        oculus_ip = str(self.get_parameter("oculus_ip").value).strip()

        self.pub_delta_pose = self.create_publisher(PoseStamped, delta_pose_topic, 10)
        self.pub_move_j = self.create_publisher(JointState, control_joint_topic, 10)
        self.pub_teleop_enabled = self.create_publisher(
            Bool, self.teleop_enabled_topic, 10
        )
        self.pub_following = self.create_publisher(
            Bool, str(self.get_parameter("following_topic").value), 10
        )
        self.move_home_client = self.create_client(Empty, move_home_service)
        self.create_subscription(Bool, self.teleop_enabled_topic, self.external_enabled_callback, 10)
        self.home_timer = None
        self.home_future = None

        # Callback inputs
        self.x = None
        self.y = None
        self.z = None
        self.roll = None
        self.pitch = None
        self.yaw = None

        self.tcp_x = None
        self.tcp_y = None
        self.tcp_z = None
        self.tcp_roll = None
        self.tcp_pitch = None
        self.tcp_yaw = None

        self.flag = False
        self.following = False
        self.follow_requires_release = False

        self.world_input = WorldInputState() if self.coordinate_mode == "world" else None
        self.world_input_error = None
        self.world_input_ready = False
        self.pub_coordinate_status = self.create_publisher(String, "/teleop/coordinate_status", 10)
        if self.world_input is None:
            self.create_subscription(PoseStamped, handle_pose_topic, self.handle_pose_callback, 1)
        else:
            self.create_subscription(String, "/quest/world_input", self.world_input_callback, 1)
        self.create_subscription(PoseStamped, feedback_tcp_pose_topic, self.tcp_pose_callback, 1)

        self.oculus_reader = OculusReader(ip_address=oculus_ip or None) if self.world_input is None else None

        # Ensure OculusReader has been initialized.
        time.sleep(0.5)

        self.zero_matrix = xyzrpy_to_mat(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.start_pose_matrix = self.zero_matrix

        self.control_timer = self.create_timer(1.0 / max(control_rate_hz, 1.0), self.control_loop)
        if self.world_input is not None:
            self.create_timer(5.0, self.log_world_input_status)

        self.get_logger().info(
            f"pub_delta_pose ready ({self.hand_name}). "
            f"transport={'Quest LOCAL / USB ADB' if self.world_input is not None else 'Wi-Fi ADB (' + oculus_ip + ':5555)' if oculus_ip else 'USB ADB'}, "
            f"translation_scale={self.translation_scale.tolist()}, "
            f"delta_frame={self.delta_frame}, "
            f"coordinate_mode={self.coordinate_mode}, "
            f"handle_topic={handle_pose_topic}, feedback_topic={feedback_tcp_pose_topic}, "
            f"delta_topic={delta_pose_topic}, control_topic={control_joint_topic}, "
            f"teleop_status={self.teleop_enabled_topic}, "
            f"buttons=(start={self.start_button}, stop={self.stop_button}, "
            f"reset={self.reset_button}, follow={self.follow_button}), gripper_trigger={self.trigger_axis}, "
            f"auto_home_on_stop={self.auto_home_on_stop}, "
            f"home_service={move_home_service}"
        )

    def world_input_callback(self, msg):
        try:
            changed = self.world_input.update(msg.data)
        except (ValueError, TypeError, KeyError) as error:
            self.world_input.packet = None
            self.world_input.received_ns = None
            self._hold_world_input(str(error))
            return
        if changed:
            self._hold_world_input("Quest 追踪空间已重置；请将手柄水平指向基座 +X 后重新按 A")
        self._update_world_pose()

    def _update_world_pose(self):
        pose = self.world_input.mapped_pose()
        if pose is None:
            self.x = None
            return
        self.x, self.y, self.z = pose[:3, 3].tolist()
        self.roll, self.pitch, self.yaw = Rotation.from_matrix(pose[:3, :3]).as_euler("xyz").tolist()

    def _hold_world_input(self, reason):
        if self.flag or self.following:
            self._set_following(False)
            self._set_teleop_enabled(False)
            self.follow_requires_release = True
        if reason != self.world_input_error:
            self.get_logger().warning(reason)
            self.world_input_error = reason

    def log_world_input_status(self):
        # Camera/tactile readiness is independent of Quest input readiness.
        # Keep an inactive controller visible after the one-time warning has
        # scrolled past during image-device startup.
        reason = self.world_input.unavailable_reason()
        if reason is not None:
            self.get_logger().info(f"[状态] Quest 未就绪 | {reason} | 恢复后重新按 A 接入")

    def handle_pose_callback(self, msg: PoseStamped):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(
            [
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ]
        )

    def tcp_pose_callback(self, msg: PoseStamped):
        self.tcp_x = msg.pose.position.x
        self.tcp_y = msg.pose.position.y
        self.tcp_z = msg.pose.position.z
        (self.tcp_roll, self.tcp_pitch, self.tcp_yaw) = euler_from_quaternion(
            [
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            ]
        )

        # Use first valid TCP feedback as zero pose
        if self.start_pose_matrix is self.zero_matrix:
            self.zero_matrix = xyzrpy_to_mat(
                self.tcp_x,
                self.tcp_y,
                self.tcp_z,
                self.tcp_roll,
                self.tcp_pitch,
                self.tcp_yaw,
            )
            self.start_pose_matrix = self.zero_matrix

    def _extract_trigger_value(self, buttons: dict) -> float:
        trigger_raw: Any = buttons.get(self.trigger_axis, [0.0])
        if isinstance(trigger_raw, (list, tuple)):
            return float(trigger_raw[0]) if trigger_raw else 0.0
        if isinstance(trigger_raw, (int, float)):
            return float(trigger_raw)
        return 0.0

    def _normalize_button_name(self, raw_value: Any, param_name: str) -> str:
        if isinstance(raw_value, bool):
            normalized_bool = "Y" if raw_value else "N"
            self.get_logger().warn(
                f"Parameter '{param_name}' is BOOL({raw_value}), "
                f"normalized to '{normalized_bool}'. "
                "Please pass string button names in launch/params."
            )
            return normalized_bool

        normalized = str(raw_value).strip()
        if (
            len(normalized) >= 2
            and normalized[0] == normalized[-1]
            and normalized[0] in ("'", '"')
        ):
            normalized = normalized[1:-1].strip()

        if not normalized:
            raise ValueError(f"Parameter '{param_name}' cannot be empty.")
        return normalized

    def _set_teleop_enabled(self, enabled: bool) -> None:
        """Publish a state change for passive recorders and data loggers."""
        if self.flag == enabled:
            return
        self.flag = enabled
        message = Bool()
        message.data = enabled
        self.pub_teleop_enabled.publish(message)

    def _set_following(self, following: bool) -> None:
        if self.following == following:
            return
        self.following = following
        message = Bool()
        message.data = following
        self.pub_following.publish(message)

    def external_enabled_callback(self, msg: Bool):
        # Our own B=False echo arrives after the home timer was scheduled.
        # Only an actual external transition out of an active session stops it.
        if not msg.data and self.flag:
            self._cancel_pending_home()
            self._set_following(False)
            self._set_teleop_enabled(False)

    def _log_reference_frames(self):
        # The reference algorithm maps displacement through the captured hand
        # and TCP orientations; matching a constant RPY alone is insufficient.
        hand_to_base = np.diag(self.translation_scale)
        if self.delta_frame == "tcp":
            hand_to_base = self.zero_matrix[:3, :3] @ hand_to_base @ self.start_pose_matrix[:3, :3].T
        self.get_logger().info("遥操参考坐标: " + json.dumps({
            "coordinate_mode": getattr(self, "coordinate_mode", "relative"),
            "delta_frame": self.delta_frame,
            "hand_reference": self.start_pose_matrix.round(6).tolist(),
            "tcp_reference": self.zero_matrix.round(6).tolist(),
            "hand_world_delta_to_base_delta": hand_to_base.round(6).tolist(),
        }, ensure_ascii=False))

    def _reset_teleop_reference(self) -> None:
        """Recenter hand-to-TCP mapping without commanding a return-home motion."""
        if not self.flag:
            self.get_logger().warning(
                f"[{self.hand_name}] 忽略复位：请先按开始键进入遥操作"
            )
            return
        if self.tcp_x is None:
            self.get_logger().warning(
                f"[{self.hand_name}] 忽略复位：尚未收到机械臂 TCP 反馈"
            )
            return
        # RJ also starts a new downstream IK reference; otherwise rebasing the
        # input alone would be interpreted as motion within the old segment.
        self._set_following(False)
        self._capture_reference()
        self.get_logger().info(
            f"[{self.hand_name}] 已复位遥操参考位姿，机械臂保持当前位置"
        )

    def _capture_reference(self) -> None:
        self.start_pose_matrix = xyzrpy_to_mat(
            self.x, self.y, self.z, self.roll, self.pitch, self.yaw
        )
        self.zero_matrix = xyzrpy_to_mat(
            self.tcp_x,
            self.tcp_y,
            self.tcp_z,
            self.tcp_roll,
            self.tcp_pitch,
            self.tcp_yaw,
        )
        self._log_reference_frames()
        world = getattr(self, "world_input", None)
        if world is not None:
            metadata = {**world.metadata(), "hand_reference": self.start_pose_matrix.tolist(),
                        "tcp_reference": self.zero_matrix.tolist(), "delta_frame": self.delta_frame,
                        "reference_host_ts_ns": time.monotonic_ns()}
            self.pub_coordinate_status.publish(String(data=json.dumps(metadata, allow_nan=False)))

    def _home_in_progress(self) -> bool:
        return self.home_future is not None and not self.home_future.done()

    def _cancel_pending_home(self) -> None:
        if self.home_timer is None:
            return
        self.destroy_timer(self.home_timer)
        self.home_timer = None
        self.get_logger().info(f"[{self.hand_name}] 已取消尚未开始的自动回零")

    def _schedule_home(self) -> None:
        if not self.auto_home_on_stop:
            return
        self._cancel_pending_home()
        self.home_timer = self.create_timer(
            self.auto_home_delay_s, self._request_home
        )
        self.get_logger().info(
            f"[{self.hand_name}] 将在 {self.auto_home_delay_s:.1f} 秒后请求机械臂回零"
        )

    def _request_home(self) -> None:
        if self.home_timer is not None:
            self.destroy_timer(self.home_timer)
            self.home_timer = None
        if self.flag:
            self.get_logger().warning(
                f"[{self.hand_name}] 已重新进入遥操，取消自动回零"
            )
            return
        if self._home_in_progress():
            return
        if not self.move_home_client.service_is_ready():
            self.get_logger().error(
                f"[{self.hand_name}] 自动回零失败：/move_home 服务不可用"
            )
            return
        self.get_logger().info(f"[{self.hand_name}] 正在请求机械臂回到固定 home 位姿")
        self.home_future = self.move_home_client.call_async(Empty.Request())
        self.home_future.add_done_callback(self._home_done)

    def _home_done(self, future) -> None:
        try:
            future.result()
        except Exception as error:
            self.get_logger().error(
                f"[{self.hand_name}] 自动回零服务调用失败：{error}"
            )
        else:
            self.get_logger().info(
                f"[{self.hand_name}] 自动回零请求已完成，请核对驱动回零日志"
            )

    def control_loop(self):
        world = getattr(self, "world_input", None)
        if world is None:
            _, buttons = self.oculus_reader.get_transformations_and_buttons()
        else:
            buttons = world.packet["buttons"] if world.fresh() and world.packet is not None else {}
        start_pressed = self.start_edge.update(buttons.get(self.start_button, False))
        if start_pressed:
            self.get_logger().info(f"[遥操] {self.start_button} 已收到")
        if world is not None:
            if not world.valid():
                self.world_input_ready = False
                reason = world.unavailable_reason()
                self._hold_world_input(reason + "；已禁止跟随，恢复后请松开并重新按 A")
                if start_pressed:
                    self.get_logger().warning(f"[遥操] A 未接入：{reason}")
                # B still works on fresh button data, but no cached gripper
                # command or A/RG engagement may pass a lost-pose boundary.
                if self.stop_edge.update(buttons.get(self.stop_button, False)):
                    action = "停止遥操作并回零" if self.auto_home_on_stop else "停止遥操作，不自动回零"
                    self.get_logger().info(f"[遥操] B：{action}")
                    self._schedule_home()
                return
            if not getattr(self, "world_input_ready", False) or self.world_input_error is not None:
                self.get_logger().info("[设备] Quest 世界输入就绪：完整空间追踪有效；松开后重新按 A 对齐，再按住 RG 跟随")
            self.world_input_ready = True
            self.world_input_error = None
            self._update_world_pose()

        # Gripper control
        trigger_value = self._extract_trigger_value(buttons)
        gripper_value = gripper_position_from_trigger(
            trigger_value, self.gripper_max_range
        )
        gripper_msg = JointState()
        gripper_msg.header = Header()
        gripper_msg.header.stamp = self.get_clock().now().to_msg()
        gripper_msg.name = [self.gripper_joint_name]
        gripper_msg.position = [gripper_value]
        self.pub_move_j.publish(gripper_msg)

        # B must work even if hand tracking is unavailable or A was never pressed.
        if self.stop_edge.update(buttons.get(self.stop_button, False)):
            action = "停止遥操作并回零" if self.auto_home_on_stop else "停止遥操作，不自动回零"
            self.get_logger().info(f"[遥操] B：{action}")
            self._set_following(False)
            self._set_teleop_enabled(False)
            self._schedule_home()
            return

        if world is not None and start_pressed and world.rotation is None:
            if self._home_in_progress() or self.tcp_x is None:
                self.get_logger().warning("机械臂 TCP 未就绪或正在回零，忽略世界方向标定，请就绪后重新按 A")
                return
            try:
                world.align_once()
            except ValueError as error:
                self.get_logger().warning(str(error))
                return
            self._update_world_pose()
            self.get_logger().info("[设备] 世界方向已锁定：首次 A 的水平指向 → 基座 +X；此后 A/RG/RJ 只重设起点")

        if self.x is None:
            return

        current_pose = [self.x, self.y, self.z, self.roll, self.pitch, self.yaw]

        if self.reset_edge.update(buttons.get(self.reset_button, False)):
            self._reset_teleop_reference()

        if start_pressed:
            if self._home_in_progress():
                self.get_logger().warning(
                    f"[{self.hand_name}] 回零尚未完成，忽略开始遥操作请求"
                )
            else:
                self._cancel_pending_home()
                if self.tcp_x is None:
                    self.get_logger().warning(
                        f"[{self.hand_name}] 尚未收到 TCP 反馈，忽略开始遥操作请求"
                    )
                else:
                    self._capture_reference()
                    self._set_following(False)
                    self.follow_requires_release = bool(buttons.get(self.follow_button, False))
                    self.get_logger().info(
                        f"[遥操] A：已对齐当前 TCP；松开后重新按住 {self.follow_button} 才跟随"
                    )
                    self._set_teleop_enabled(True)

        if self.flag:
            follow_requested = bool(buttons.get(self.follow_button, False))
            if self.follow_requires_release:
                if not follow_requested:
                    self.follow_requires_release = False
                self._set_following(False)
                return
            if not follow_requested:
                if self.following:
                    self.get_logger().info(f"[{self.hand_name}] 松开 {self.follow_button}：冻结最后下发的关节目标")
                self._set_following(False)
                return
            if not self.following:
                # Discard paused hand motion. Downstream IK reanchors these
                # increments at the held joint command to avoid feedback drift.
                if self.tcp_x is None:
                    return
                self._capture_reference()
                self._set_following(True)
                self.get_logger().info(f"[{self.hand_name}] 按住 {self.follow_button}：重置手柄参考，连续接入保持目标")
            xyz, quat = calc_pose_incre(
                self.start_pose_matrix,
                current_pose,
                self.zero_matrix,
                self.translation_scale,
                self.delta_frame,
            )
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "base_link"
            pose_msg.pose.position.x = float(xyz[0])
            pose_msg.pose.position.y = float(xyz[1])
            pose_msg.pose.position.z = float(xyz[2])
            pose_msg.pose.orientation.x = float(quat[0])
            pose_msg.pose.orientation.y = float(quat[1])
            pose_msg.pose.orientation.z = float(quat[2])
            pose_msg.pose.orientation.w = float(quat[3])
            self.pub_delta_pose.publish(pose_msg)

    def destroy_node(self):
        self._cancel_pending_home()
        if self.context.ok():
            self._set_following(False)
            self._set_teleop_enabled(False)
        if self.oculus_reader is not None:
            self.oculus_reader.stop()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    teleop_node = None
    executor = None
    try:
        teleop_node = RosOperator()
        executor = TeleopExecutor()
        executor.add_node(teleop_node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            if executor is not None:
                executor.shutdown()
        finally:
            if teleop_node is not None:
                teleop_node.destroy_node()
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
