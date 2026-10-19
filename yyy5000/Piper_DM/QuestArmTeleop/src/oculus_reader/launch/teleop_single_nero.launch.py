import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    oculus_reader_pkg_dir = get_package_share_directory("oculus_reader")
    agx_arm_ctrl_pkg_dir = get_package_share_directory("agx_arm_ctrl")

    arm_ik_param_file = os.path.join(
        oculus_reader_pkg_dir, "config", "arm_ik_pose_node.nero.yaml"
    )

    # 1) ros2 launch agx_arm_ctrl start_single_agx_arm_rviz.launch.py ...
    agx_arm_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                agx_arm_ctrl_pkg_dir,
                "launch",
                "start_single_agx_arm_rviz.launch.py",
            )
        ),
        launch_arguments={
            "can_port": "can0",
            "arm_type": "nero",
            "auto_enable": "true",
            "effector_type": "agx_gripper",
            "tcp_offset": "[0.1755, 0.0, -0.0235, 0.0, 0.0, 0.0]",
            "control": "false",
            "fast_mode": "true",
        }.items(),
    )

    # 2) ros2 run oculus_reader arm_ik_pose_node.py --ros-args --params-file ...
    arm_ik_pose_node = Node(
        package="oculus_reader",
        executable="arm_ik_pose_node.py",
        name="arm_ik_pose_node",
        output="screen",
        parameters=[arm_ik_param_file],
    )

    # 3) ros2 run oculus_reader pub_pose.py --ros-args -p ros_to_arm_rpy:=...
    pub_pose_node = Node(
        package="oculus_reader",
        executable="pub_pose.py",
        name="pub_pose_node",
        output="screen",
        # pika frame to arm ee frame
        arguments=["--ros-args", "-p", "ros_to_arm_rpy:=[-1.5708, 0.0, 0.0]"],
    )

    # 4) ros2 run oculus_reader pub_delta_pose.py
    pub_delta_pose_node = Node(
        package="oculus_reader",
        executable="pub_delta_pose.py",
        name="pub_delta_pose_node",
        output="screen",
    )

    # 5) ros2 run rviz2 rviz2 --ros-args -p config:=...
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(get_package_share_directory('oculus_reader'), 'config', 'oculus_reader.rviz')]
    )
    
    return LaunchDescription(
        [
            agx_arm_rviz_launch,
            arm_ik_pose_node,
            pub_pose_node,
            pub_delta_pose_node,
            rviz_node,
        ]
    )
