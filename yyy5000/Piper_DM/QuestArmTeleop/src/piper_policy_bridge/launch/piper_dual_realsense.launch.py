"""Publish wrist D435 and global D415 RGB streams for multi-view policy use."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _camera(name, serial, color_profile):
    return Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        namespace="camera",
        name=name,
        output="screen",
        parameters=[
            {
                "camera_name": name,
                "serial_no": ParameterValue(serial, value_type=str),
                "enable_color": True,
                "enable_depth": False,
                "enable_infra": False,
                "enable_infra1": False,
                "enable_infra2": False,
                # Publish the policy's native resolution so two raw RGB streams
                # do not cross DDS at four times the required payload size.
                "rgb_camera.color_profile": ParameterValue(color_profile, value_type=str),
                "rgb_camera.color_format": "RGB8",
                "publish_tf": False,
            }
        ],
    )


def generate_launch_description():
    wrist_serial = LaunchConfiguration("wrist_serial_no")
    global_serial = LaunchConfiguration("global_serial_no")
    preview = LaunchConfiguration("preview")
    color_profile = LaunchConfiguration("color_profile")
    return LaunchDescription(
        [
            DeclareLaunchArgument("wrist_serial_no", default_value="801212071416"),
            DeclareLaunchArgument("global_serial_no", default_value="211622069098"),
            DeclareLaunchArgument(
                "color_profile",
                default_value="320x240x30",
                description="RealSense RGB profile; use 640x480x30 for quest-tac-v1.",
            ),
            DeclareLaunchArgument(
                "preview",
                default_value="false",
                choices=["true", "false"],
                description="Open one rqt_image_view window for each RGB stream.",
            ),
            _camera("wrist_camera", wrist_serial, color_profile),
            _camera("global_camera", global_serial, color_profile),
            Node(
                package="rqt_image_view",
                executable="rqt_image_view",
                name="wrist_image_view",
                arguments=["/camera/wrist_camera/color/image_raw"],
                condition=IfCondition(preview),
                output="screen",
            ),
            Node(
                package="rqt_image_view",
                executable="rqt_image_view",
                name="global_image_view",
                arguments=["/camera/global_camera/color/image_raw"],
                condition=IfCondition(preview),
                output="screen",
            ),
        ]
    )
