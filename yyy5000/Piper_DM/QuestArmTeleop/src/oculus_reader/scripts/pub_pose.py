#!/usr/bin/env python3
from oculus_reader import OculusReader
from transformations import quaternion_from_matrix
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import PoseStamped, TransformStamped
from math import sin, cos
import numpy as np

def xyzrpy2Mat(x, y, z, roll, pitch, yaw):
    cr, sr = cos(roll), sin(roll)
    cp, sp = cos(pitch), sin(pitch)
    cy, sy = cos(yaw), sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, sy * sr + cy * sp * cr, x],
        [sy * cp, cy * cr + sy * sp * sr, sy * sp * cr - cy * sr, y],
        [-sp, cp * sr, cp * cr, z],
        [0.0, 0.0, 0.0, 1.0],
    ])
    
class OculusPublisher(Node):
    def __init__(self):
        super().__init__("pub_pose_node")
        self.declare_parameter("publish_rate_hz", 100.0)
        self.declare_parameter("parent_frame_id", "vr_device")
        self.declare_parameter("right_handle_pose_topic", "right_handle_pose")
        self.declare_parameter("left_handle_pose_topic", "left_handle_pose")
        self.declare_parameter("ros_to_arm_rpy", [-np.pi / 2, 0.0, 0.0])
        self.declare_parameter("ros_to_arm_xyz", [0.0, 0.0, 0.0])
        # Empty means that OculusReader selects an ADB USB device.  Keep the
        # existing Wi-Fi endpoint as the default for the original launch files.
        self.declare_parameter("oculus_ip", "192.168.5.32")

        self.parent_frame_id = str(self.get_parameter("parent_frame_id").value)
        right_handle_pose_topic = str(self.get_parameter("right_handle_pose_topic").value)
        left_handle_pose_topic = str(self.get_parameter("left_handle_pose_topic").value)
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        ros_to_arm_rpy = self.get_parameter("ros_to_arm_rpy").value
        ros_to_arm_xyz = self.get_parameter("ros_to_arm_xyz").value
        oculus_ip = str(self.get_parameter("oculus_ip").value).strip()
        self.oculus_reader = OculusReader(ip_address=oculus_ip or None)
        transport = (
            f"Wi-Fi ADB ({oculus_ip}:5555)" if oculus_ip else "USB ADB"
        )
        self.get_logger().info(
            f"Quest transport: {transport}, "
            f"controller_axis_rpy={[float(value) for value in ros_to_arm_rpy]}"
        )
        if len(ros_to_arm_rpy) != 3 or len(ros_to_arm_xyz) != 3:
            raise ValueError("ros_to_arm_rpy/ros_to_arm_xyz must have exactly 3 elements")

        self.right_handle_pose_pub = self.create_publisher(PoseStamped, right_handle_pose_topic, 1)
        self.left_handle_pose_pub = self.create_publisher(PoseStamped, left_handle_pose_topic, 1)
        self.br = TransformBroadcaster(self)

        self.adj_mat = np.array([
            [0.0, 0.0, -1.0, 0.0],
            [-1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ], dtype=float)
        
        self.r_adj = xyzrpy2Mat(0, 0, 0, -np.pi, 0, -np.pi / 2)  # 将手柄坐标系调整到与ROS一致
        
        self.ros_to_arm_mat = xyzrpy2Mat(
            float(ros_to_arm_xyz[0]),
            float(ros_to_arm_xyz[1]),
            float(ros_to_arm_xyz[2]),
            float(ros_to_arm_rpy[0]),
            float(ros_to_arm_rpy[1]),
            float(ros_to_arm_rpy[2]),
        )

        # 复用消息对象，降低循环内内存分配
        self.right_tf_msg = TransformStamped()
        self.left_tf_msg = TransformStamped()
        self.right_pose_msg = PoseStamped()
        self.left_pose_msg = PoseStamped()
        self.right_tf_msg.header.frame_id = self.parent_frame_id
        self.right_tf_msg.child_frame_id = "right_controller"
        self.left_tf_msg.header.frame_id = self.parent_frame_id
        self.left_tf_msg.child_frame_id = "left_controller"
        self.right_pose_msg.header.frame_id = self.parent_frame_id
        self.left_pose_msg.header.frame_id = self.parent_frame_id

        self.timer = self.create_timer(1.0 / max(publish_rate_hz, 1.0), self._timer_callback)

    def _correct_to_ros(self, transform):
        
        if transform.shape != (4, 4):
            raise ValueError("Input transform must be a 4x4 numpy array.")

        transform = self.adj_mat @ transform
        """OpenXR → ROS 坐标系转换（X前向、Y左向、Z上向）"""
        transform = np.dot(transform, self.r_adj)
        """将ROS坐标系转换为机械臂末端坐标系"""
        transform = np.dot(transform, self.ros_to_arm_mat)
        return transform

    def _publish_transform(self, transform, pose_pub, tf_msg, pose_msg, now):
        """通用发布函数（左右手柄共用）"""
        translation = transform[:3, 3]
        quat = quaternion_from_matrix(transform)

        tf_msg.header.stamp = now
        tf_msg.transform.translation.x = float(translation[0])
        tf_msg.transform.translation.y = float(translation[1])
        tf_msg.transform.translation.z = float(translation[2])
        tf_msg.transform.rotation.x = float(quat[0])
        tf_msg.transform.rotation.y = float(quat[1])
        tf_msg.transform.rotation.z = float(quat[2])
        tf_msg.transform.rotation.w = float(quat[3])
        self.br.sendTransform(tf_msg)

        # 同时发布 PoseStamped
        pose_msg.header.stamp = now
        pose_msg.pose.position.x = float(translation[0])
        pose_msg.pose.position.y = float(translation[1])
        pose_msg.pose.position.z = float(translation[2])
        pose_msg.pose.orientation.x = float(quat[0])
        pose_msg.pose.orientation.y = float(quat[1])
        pose_msg.pose.orientation.z = float(quat[2])
        pose_msg.pose.orientation.w = float(quat[3])
        pose_pub.publish(pose_msg)

    def _timer_callback(self):
        transformations, _ = self.oculus_reader.get_transformations_and_buttons()
        r_transform = transformations.get('r')
        l_transform = transformations.get('l')
        if r_transform is None or l_transform is None:
            return

        # 将坐标系转换为ROS标准（X朝前、Y朝左、Z朝上）
        # ps：需要将头显放平且手握手柄时摇杆面朝向自己才能确保遥操的跟手
        r_transform_ros = self._correct_to_ros(r_transform)
        l_transform_ros = self._correct_to_ros(l_transform)
        now = self.get_clock().now().to_msg()

        self._publish_transform(
            r_transform_ros, self.right_handle_pose_pub, self.right_tf_msg, self.right_pose_msg, now
        )
        self._publish_transform(
            l_transform_ros, self.left_handle_pose_pub, self.left_tf_msg, self.left_pose_msg, now
        )

    def destroy_node(self):
        self.oculus_reader.stop()
        return super().destroy_node()

if __name__ == '__main__':
    rclpy.init(args=None)
    publisher = OculusPublisher()
    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        rclpy.shutdown()
