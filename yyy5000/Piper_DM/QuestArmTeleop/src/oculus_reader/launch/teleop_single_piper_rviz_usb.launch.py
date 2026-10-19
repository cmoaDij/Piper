"""Preview Quest teleoperation over USB in RViz only for a legacy Piper.

This launch deliberately excludes agx_arm_ctrl.  It does not open can0 and no
node in this launch can send commands to a physical arm.
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

    # Use an isolated topic namespace.  This prevents a separately running
    # physical controller from consuming the preview's JointState messages.
    simulation_control_topic = "/rviz_sim/control/joint_states"
    simulation_feedback_joint_topic = "/rviz_sim/feedback/joint_states"
    simulation_feedback_tcp_pose_topic = "/rviz_sim/feedback/tcp_pose"
    simulation_delta_pose_topic = "/rviz_sim/delta_pose"
    simulation_right_handle_pose_topic = "/rviz_sim/right_handle_pose"

    # RViz's robot_state_publisher listens to the simulated IK joint command.
    # No CAN-facing agx_arm_ctrl launch is included here.
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
            "custom_model": "piper/urdf/piper_with_gripper_description_old.urdf",
            "tcp_offset": "[0.0, 0.0, 0.113, 0.0, 0.0, 0.0]",
            "follow": "false",
            "control": "false",
            "control_topic": simulation_control_topic,
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
                "pin_joint_status_topic": simulation_control_topic,
                "feedback_joint_topic": simulation_feedback_joint_topic,
                "pose_stamped_topic": simulation_delta_pose_topic,
                "simulation_feedback_joint_topic": simulation_feedback_joint_topic,
                "simulation_feedback_tcp_pose_topic": simulation_feedback_tcp_pose_topic,
                "simulation_feedback_rate_hz": 30.0,
            },
        ],
    )

    # Empty oculus_ip selects the Quest connected through USB ADB.
    pub_pose_node = Node(
        package="oculus_reader",
        executable="pub_pose.py",
        name="pub_pose_node",
        output="screen",
        parameters=[
            {
                "oculus_ip": "",
                "ros_to_arm_rpy": [0.0, 1.0471, 0.0],
                "right_handle_pose_topic": simulation_right_handle_pose_topic,
                "left_handle_pose_topic": "/rviz_sim/left_handle_pose",
            },
        ],
    )

    pub_delta_pose_node = Node(
        package="oculus_reader",
        executable="pub_delta_pose.py",
        name="pub_delta_pose_node",
        output="screen",
        parameters=[
            {
                "oculus_ip": "",
                "handle_pose_topic": simulation_right_handle_pose_topic,
                "feedback_tcp_pose_topic": simulation_feedback_tcp_pose_topic,
                "delta_pose_topic": simulation_delta_pose_topic,
                "control_joint_topic": simulation_control_topic,
                "translation_scale": [1.2, 1.2, 1.2],
            },
        ],
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
