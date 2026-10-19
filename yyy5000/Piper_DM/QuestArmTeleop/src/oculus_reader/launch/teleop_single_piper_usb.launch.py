"""Teleoperate a Piper-family arm through a Quest connected by USB ADB.

This launch starts the real CAN arm controller.  Verify the workspace, frame
alignment, and emergency stop before pressing the teleoperation start button.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _validate_command_topic(context):
    """Keep the hardware command topic under exclusive mux ownership."""
    command_topic = LaunchConfiguration("command_topic").perform(context)
    normalized_topic = "/" + command_topic.strip().strip("/")
    if normalized_topic != "/teleop/joint_states":
        raise RuntimeError(
            "command_topic must be /teleop/joint_states; "
            "piper_command_mux is the sole publisher to /control/joint_states"
        )
    return []


def _launch_pub_pose_node(context):
    """Create the Quest pose node with a typed controller-frame rotation."""
    parameter_names = (
        "ros_to_arm_roll",
        "ros_to_arm_pitch",
        "ros_to_arm_yaw",
    )
    try:
        ros_to_arm_rpy = [
            float(LaunchConfiguration(name).perform(context))
            for name in parameter_names
        ]
    except ValueError as exc:
        raise RuntimeError(
            "ros_to_arm_roll, ros_to_arm_pitch, and ros_to_arm_yaw "
            "must be valid floating-point values"
        ) from exc

    return [
        Node(
            package="oculus_reader",
            executable="pub_pose.py",
            name="pub_pose_node",
            output="screen",
            parameters=[
                {
                    "oculus_ip": "",
                    "ros_to_arm_rpy": ros_to_arm_rpy,
                }
            ],
        )
    ]


def generate_launch_description():
    oculus_reader_pkg_dir = get_package_share_directory("oculus_reader")
    agx_arm_ctrl_pkg_dir = get_package_share_directory("agx_arm_ctrl")
    default_arm_ik_param_file = os.path.join(
        oculus_reader_pkg_dir, "config", "arm_ik_pose_node.piper.yaml"
    )

    arm_type_arg = DeclareLaunchArgument(
        "arm_type",
        default_value="piper",
        description="AGX arm model, for example piper or piper_x.",
    )
    custom_model_arg = DeclareLaunchArgument(
        "custom_model",
        default_value="piper/urdf/piper_with_gripper_description.urdf",
        description="Piper V1.5.8 robot model relative to agx_arm_urdf.",
    )
    ik_config_file_arg = DeclareLaunchArgument(
        "ik_config_file",
        default_value=default_arm_ik_param_file,
        description="Absolute IK parameter YAML path for the selected arm.",
    )
    tcp_offset_arg = DeclareLaunchArgument(
        "tcp_offset",
        default_value="[0.0, 0.0, 0.113, 0.0, 0.0, 0.0]",
        description="Driver TCP offset [x,y,z,roll,pitch,yaw].",
    )
    gripper_max_range_arg = DeclareLaunchArgument(
        "gripper_max_range",
        default_value="0.07",
        description="Released-trigger gripper opening in metres.",
    )
    quest_input_arg = DeclareLaunchArgument(
        "quest_input",
        default_value="true",
        choices=["true", "false"],
        description="Start Quest pose/button nodes; disable for keyboard testing.",
    )
    ros_to_arm_roll_arg = DeclareLaunchArgument(
        "ros_to_arm_roll",
        default_value="0.0",
        description="Controller-frame roll correction in radians.",
    )
    ros_to_arm_pitch_arg = DeclareLaunchArgument(
        "ros_to_arm_pitch",
        default_value="1.0471",
        description="Controller-frame pitch correction in radians.",
    )
    ros_to_arm_yaw_arg = DeclareLaunchArgument(
        "ros_to_arm_yaw",
        default_value="0.0",
        description="Controller-frame yaw correction in radians.",
    )

    fast_mode_arg = DeclareLaunchArgument(
        "fast_mode",
        default_value="true",
        choices=["true", "false"],
        description="Use move_js fast joint control. false uses move_j instead.",
    )
    can_port_arg = DeclareLaunchArgument(
        "can_port",
        default_value="can0",
        description="CAN interface used by the Piper driver.",
    )
    command_topic_arg = DeclareLaunchArgument(
        "command_topic",
        default_value="/teleop/joint_states",
        description=(
            "Internal Quest teleoperation input to piper_command_mux. "
            "It must remain /teleop/joint_states."
        ),
    )
    reset_button_arg = DeclareLaunchArgument(
        "reset_button",
        default_value="RJ",
        description=(
            "Quest button used to recenter the active hand/TCP mapping. "
            "RJ is the right joystick press."
        ),
    )
    auto_home_arg = DeclareLaunchArgument(
        "auto_home_on_stop",
        default_value="true",
        choices=["true", "false"],
        description="Request /move_home after B stops teleoperation.",
    )
    auto_home_delay_arg = DeclareLaunchArgument(
        "auto_home_delay_s",
        default_value="0.5",
        description="Delay after publishing teleop disabled before requesting home.",
    )
    speed_percent_arg = DeclareLaunchArgument(
        "speed_percent",
        default_value="50",
        description="Normal driver movement speed percentage.",
    )
    home_speed_percent_arg = DeclareLaunchArgument(
        "home_speed_percent",
        default_value="10",
        description="Low speed percentage used only for B-triggered move_home.",
    )
    home_position_tolerance_arg = DeclareLaunchArgument(
        "home_position_tolerance",
        default_value="0.10",
        description=(
            "Maximum absolute joint error in radians accepted after B-triggered home."
        ),
    )

    agx_arm_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                agx_arm_ctrl_pkg_dir,
                "launch",
                "start_single_agx_arm_rviz.launch.py",
            )
        ),
        launch_arguments={
            "can_port": LaunchConfiguration("can_port"),
            "arm_type": LaunchConfiguration("arm_type"),
            "custom_model": LaunchConfiguration("custom_model"),
            "effector_type": "agx_gripper",
            "tcp_offset": LaunchConfiguration("tcp_offset"),
            "control": "false",
            "control_enabled": "true",
            "fast_mode": LaunchConfiguration("fast_mode"),
            "speed_percent": LaunchConfiguration("speed_percent"),
            "home_speed_percent": LaunchConfiguration("home_speed_percent"),
            "home_position_tolerance": LaunchConfiguration(
                "home_position_tolerance"
            ),
        }.items(),
    )

    command_mux_node = Node(
        package="piper_policy_bridge",
        executable="piper_command_mux",
        name="piper_command_mux",
        output="screen",
        parameters=[
            {
                "teleop_topic": LaunchConfiguration("command_topic"),
                "teleop_enabled_topic": "/teleop/enabled",
                "hardware_topic": "/control/joint_states",
            }
        ],
    )

    arm_ik_pose_node = Node(
        package="oculus_reader",
        executable="arm_ik_pose_node.py",
        name="arm_ik_pose_node",
        output="screen",
        parameters=[
            LaunchConfiguration("ik_config_file"),
            {"pin_joint_status_topic": LaunchConfiguration("command_topic")},
        ],
    )

    # Resolve the three launch strings before constructing the ROS parameter.
    # Passing LaunchConfiguration objects directly in a list concatenates them
    # into one string on ROS 2 Humble instead of producing a DOUBLE_ARRAY.
    pub_pose_action = OpaqueFunction(
        function=_launch_pub_pose_node,
        condition=IfCondition(LaunchConfiguration("quest_input")),
    )

    pub_delta_pose_node = Node(
        package="oculus_reader",
        executable="pub_delta_pose.py",
        name="pub_delta_pose_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("quest_input")),
        parameters=[
            {
                "oculus_ip": "",
                "translation_scale": [1.2, 1.2, 1.2],
                "gripper_max_range": ParameterValue(
                    LaunchConfiguration("gripper_max_range"), value_type=float
                ),
                "control_joint_topic": LaunchConfiguration("command_topic"),
                "reset_button": LaunchConfiguration("reset_button"),
                "auto_home_on_stop": ParameterValue(
                    LaunchConfiguration("auto_home_on_stop"), value_type=bool
                ),
                "auto_home_delay_s": ParameterValue(
                    LaunchConfiguration("auto_home_delay_s"), value_type=float
                ),
            }
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            os.path.join(oculus_reader_pkg_dir, "config", "oculus_reader.rviz"),
        ],
    )

    return LaunchDescription(
        [
            arm_type_arg,
            custom_model_arg,
            ik_config_file_arg,
            tcp_offset_arg,
            gripper_max_range_arg,
            quest_input_arg,
            ros_to_arm_roll_arg,
            ros_to_arm_pitch_arg,
            ros_to_arm_yaw_arg,
            fast_mode_arg,
            can_port_arg,
            command_topic_arg,
            OpaqueFunction(function=_validate_command_topic),
            reset_button_arg,
            auto_home_arg,
            auto_home_delay_arg,
            speed_percent_arg,
            home_speed_percent_arg,
            home_position_tolerance_arg,
            agx_arm_rviz_launch,
            command_mux_node,
            arm_ik_pose_node,
            pub_pose_action,
            pub_delta_pose_node,
            rviz_node,
        ]
    )
