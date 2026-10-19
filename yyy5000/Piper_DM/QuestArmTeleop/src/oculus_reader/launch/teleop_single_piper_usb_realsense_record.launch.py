"""USB Quest teleoperation with a wrist D435 colour-video recorder."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    oculus_share = get_package_share_directory("oculus_reader")
    realsense_share = get_package_share_directory("realsense2_camera")
    fast_mode = LaunchConfiguration("fast_mode")
    video_output_dir = LaunchConfiguration("video_output_dir")
    record_on_start = LaunchConfiguration("record_on_start")
    video_fps = LaunchConfiguration("video_fps")

    teleoperation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(oculus_share, "launch", "teleop_single_piper_usb.launch.py")
        ),
        launch_arguments={"fast_mode": fast_mode}.items(),
    )
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_share, "launch", "rs_launch.py")
        ),
        launch_arguments={
            "camera_name": "wrist_camera",
            "camera_namespace": "camera",
            "enable_color": "true",
            "enable_depth": "false",
            "rgb_camera.color_profile": "640x480x30",
            "rgb_camera.color_format": "RGB8",
        }.items(),
    )
    recorder = Node(
        package="oculus_reader",
        executable="realsense_video_recorder.py",
        name="realsense_video_recorder",
        output="screen",
        parameters=[
            {
                "color_topic": "/camera/wrist_camera/color/image_raw",
                "record_topic": "/teleop/enabled",
                "output_dir": video_output_dir,
                "record_on_start": record_on_start,
                "fps": video_fps,
            }
        ],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("fast_mode", default_value="true", choices=["true", "false"]),
            DeclareLaunchArgument("video_output_dir", default_value="~/QuestArmTeleop/recordings"),
            DeclareLaunchArgument("record_on_start", default_value="false", choices=["true", "false"]),
            DeclareLaunchArgument("video_fps", default_value="30.0"),
            teleoperation,
            camera,
            recorder,
        ]
    )
