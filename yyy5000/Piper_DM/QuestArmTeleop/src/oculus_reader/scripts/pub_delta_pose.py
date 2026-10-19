#!/usr/bin/env python3
import math
import time
from typing import Any

import numpy as np
import rclpy
from button_edge import RisingEdge
from geometry_msgs.msg import PoseStamped
from gripper_mapping import gripper_position_from_trigger
from oculus_reader import OculusReader
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Header
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
):
    end_matrix = xyzrpy_to_mat(
        current_pose_matrix[0],
        current_pose_matrix[1],
        current_pose_matrix[2],
        current_pose_matrix[3],
        current_pose_matrix[4],
        current_pose_matrix[5],
    )
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
        self.declare_parameter("move_home_service", "/move_home")

        # Button mapping parameters
        dynamic_string_param = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter("start_button", "A", dynamic_string_param)
        self.declare_parameter("stop_button", "B", dynamic_string_param)
        self.declare_parameter("reset_button", "RJ", dynamic_string_param)
        self.declare_parameter("trigger_axis", "rightTrig")

        # Gripper/control parameters
        self.declare_parameter("gripper_joint_name", "gripper")
        self.declare_parameter("gripper_max_range", 0.07)
        self.declare_parameter("control_rate_hz", 30.0)
        self.declare_parameter("hand_name", "right")
        self.declare_parameter("translation_scale", [1.0, 1.0, 1.0])
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
        if len({self.start_button, self.stop_button, self.reset_button}) != 3:
            raise ValueError(
                "start_button, stop_button, and reset_button must be distinct."
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
        self.move_home_client = self.create_client(Empty, move_home_service)
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

        self.create_subscription(PoseStamped, handle_pose_topic, self.handle_pose_callback, 1)
        self.create_subscription(PoseStamped, feedback_tcp_pose_topic, self.tcp_pose_callback, 1)

        self.oculus_reader = OculusReader(ip_address=oculus_ip or None)

        # Ensure OculusReader has been initialized.
        time.sleep(0.5)

        self.zero_matrix = xyzrpy_to_mat(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.start_pose_matrix = self.zero_matrix

        self.control_timer = self.create_timer(1.0 / max(control_rate_hz, 1.0), self.control_loop)

        self.get_logger().info(
            f"pub_delta_pose ready ({self.hand_name}). "
            f"transport={'Wi-Fi ADB (' + oculus_ip + ':5555)' if oculus_ip else 'USB ADB'}, "
            f"translation_scale={self.translation_scale.tolist()}, "
            f"handle_topic={handle_pose_topic}, feedback_topic={feedback_tcp_pose_topic}, "
            f"delta_topic={delta_pose_topic}, control_topic={control_joint_topic}, "
            f"teleop_status={self.teleop_enabled_topic}, "
            f"buttons=(start={self.start_button}, stop={self.stop_button}, "
            f"reset={self.reset_button}), trigger={self.trigger_axis}, "
            f"auto_home_on_stop={self.auto_home_on_stop}, "
            f"home_service={move_home_service}"
        )

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
        self.get_logger().info(
            f"[{self.hand_name}] 已复位遥操参考位姿，机械臂保持当前位置"
        )

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
        _, buttons = self.oculus_reader.get_transformations_and_buttons()

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

        if self.x is None:
            return

        current_pose = [self.x, self.y, self.z, self.roll, self.pitch, self.yaw]

        if self.reset_edge.update(buttons.get(self.reset_button, False)):
            self._reset_teleop_reference()

        if self.start_edge.update(buttons.get(self.start_button, False)):
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
                    # Rebase both sides at the same instant. This is essential after
                    # auto-home changed the arm pose while teleoperation was disabled.
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
                    self.get_logger().info(
                        f"[{self.hand_name}] 开始遥操作，已锁定当前 TCP 参考"
                    )
                    self._set_teleop_enabled(True)

        if self.stop_edge.update(buttons.get(self.stop_button, False)):
            was_enabled = self.flag
            if was_enabled:
                self.get_logger().info(f"[{self.hand_name}] 停止遥操作")
            self._set_teleop_enabled(False)
            if self.tcp_x is not None:
                self.zero_matrix = xyzrpy_to_mat(
                    self.tcp_x,
                    self.tcp_y,
                    self.tcp_z,
                    self.tcp_roll,
                    self.tcp_pitch,
                    self.tcp_yaw,
                )
                self.start_pose_matrix = xyzrpy_to_mat(
                    self.x, self.y, self.z, self.roll, self.pitch, self.yaw
                )
            if was_enabled:
                self._schedule_home()

        if self.flag:
            xyz, quat = calc_pose_incre(
                self.start_pose_matrix,
                current_pose,
                self.zero_matrix,
                self.translation_scale,
            )
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "vr_device"
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
        self._set_teleop_enabled(False)
        self.oculus_reader.stop()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    teleop_node = None
    try:
        teleop_node = RosOperator()
        executor = MultiThreadedExecutor()
        executor.add_node(teleop_node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if teleop_node is not None:
            teleop_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
