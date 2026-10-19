"""Start the standard USB Quest teleoperation stack for a Piper X."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Configure the shared USB workflow for Piper X hardware."""
    package_share = get_package_share_directory("oculus_reader")
    usb_launch = os.path.join(
        package_share, "launch", "teleop_single_piper_usb.launch.py"
    )
    ik_config = os.path.join(
        package_share, "config", "arm_ik_pose_node.piper_x.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("can_port", default_value="can0"),
            DeclareLaunchArgument(
                "fast_mode", default_value="true", choices=["true", "false"]
            ),
            DeclareLaunchArgument("speed_percent", default_value="50"),
            DeclareLaunchArgument(
                "home_speed_percent",
                default_value="5",
                description="Low speed used by the speed-planned Piper X home move.",
            ),
            DeclareLaunchArgument(
                "home_position_tolerance",
                default_value="0.03",
                description="Maximum Piper X home joint error in radians.",
            ),
            DeclareLaunchArgument(
                "auto_home_on_stop",
                default_value="true",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument("auto_home_delay_s", default_value="0.5"),
            DeclareLaunchArgument("reset_button", default_value="RJ"),
            DeclareLaunchArgument(
                "quest_input",
                default_value="true",
                choices=["true", "false"],
            ),
            # Piper X base_link follows REP-103: +X forward, +Y left, +Z up.
            # These rotations align the normally-held right controller axes
            # with the Piper X TCP axes used by the delta-pose mapping.
            DeclareLaunchArgument("ros_to_arm_roll", default_value="-1.5708"),
            DeclareLaunchArgument("ros_to_arm_pitch", default_value="0.0"),
            DeclareLaunchArgument("ros_to_arm_yaw", default_value="-1.5708"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(usb_launch),
                launch_arguments={
                    "arm_type": "piper_x",
                    "custom_model": (
                        "piper_x/urdf/piper_x_with_gripper_description.urdf"
                    ),
                    "ik_config_file": ik_config,
                    "tcp_offset": "[0.0, 0.0, 0.1325, 0.0, 0.0, 0.0]",
                    "gripper_max_range": "0.03",
                    "can_port": LaunchConfiguration("can_port"),
                    "fast_mode": LaunchConfiguration("fast_mode"),
                    "speed_percent": LaunchConfiguration("speed_percent"),
                    "home_speed_percent": LaunchConfiguration(
                        "home_speed_percent"
                    ),
                    "home_position_tolerance": LaunchConfiguration(
                        "home_position_tolerance"
                    ),
                    "auto_home_on_stop": LaunchConfiguration(
                        "auto_home_on_stop"
                    ),
                    "auto_home_delay_s": LaunchConfiguration(
                        "auto_home_delay_s"
                    ),
                    "reset_button": LaunchConfiguration("reset_button"),
                    "quest_input": LaunchConfiguration("quest_input"),
                    "ros_to_arm_roll": LaunchConfiguration("ros_to_arm_roll"),
                    "ros_to_arm_pitch": LaunchConfiguration("ros_to_arm_pitch"),
                    "ros_to_arm_yaw": LaunchConfiguration("ros_to_arm_yaw"),
                }.items(),
            ),
        ]
    )
