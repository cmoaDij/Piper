"""Quest teleoperation preview for Piper in RViz only.

This launch intentionally does not include agx_arm_ctrl, so it never opens a
CAN interface or sends commands to a physical arm. IK output drives only the
robot_state_publisher used by RViz.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    oculus_reader_pkg_dir = get_package_share_directory("oculus_reader")
    agx_arm_description_pkg_dir = get_package_share_directory("agx_arm_description")

    arm_ik_param_file = os.path.join(
        oculus_reader_pkg_dir, "config", "arm_ik_pose_node.piper.yaml"
    )

    # RViz model only: do not include agx_arm_ctrl or any CAN-facing node.
    piper_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                agx_arm_description_pkg_dir,
                "launch",
                "display.launch.py",
            )
        ),
        launch_arguments={
            "arm_type": "piper",
            "custom_model": "piper/urdf/piper_with_gripper_description.urdf",
            "tcp_offset": "[0.0, 0.0, 0.113, 0.0, 0.0, 0.0]",
            "follow": "false",
            "control": "false",
            "control_topic": "control/joint_states",
        }.items(),
    )

    arm_ik_pose_node = Node(
        package="oculus_reader",
        executable="arm_ik_pose_node.py",
        name="arm_ik_pose_node",
        output="screen",
        parameters=[
            arm_ik_param_file,
            {
                "simulation_feedback": True,
                "simulation_feedback_joint_topic": "/feedback/joint_states",
                "simulation_feedback_tcp_pose_topic": "/feedback/tcp_pose",
                "simulation_feedback_rate_hz": 30.0,
            },
        ],
    )

    pub_pose_node = Node(
        package="oculus_reader",
        executable="pub_pose.py",
        name="pub_pose_node",
        output="screen",
        arguments=["--ros-args", "-p", "ros_to_arm_rpy:=[0.0, 1.5708, 0.0]"],
    )

    pub_delta_pose_node = Node(
        package="oculus_reader",
        executable="pub_delta_pose.py",
        name="pub_delta_pose_node",
        output="screen",
    )

    quest_rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="quest_rviz2",
        output="screen",
        arguments=[
            "-d",
            os.path.join(oculus_reader_pkg_dir, "config", "oculus_reader.rviz"),
        ],
    )

    return LaunchDescription(
        [
            piper_rviz_launch,
            arm_ik_pose_node,
            pub_pose_node,
            pub_delta_pose_node,
            quest_rviz_node,
        ]
    )
