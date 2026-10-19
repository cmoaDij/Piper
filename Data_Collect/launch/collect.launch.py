"""Start the local Quest/IK stack and one collection-only Piper driver."""

import importlib.util
import os
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def _check_python_environment():
    hint = f"请先执行：source {ROOT}/env.sh，然后重新运行 ros2 launch。"
    if sys.version_info[:2] not in ((3, 10), (3, 11)):
        raise RuntimeError(
            f"当前 ros2 使用 {sys.executable} (Python {sys.version_info.major}."
            f"{sys.version_info.minor})，触觉 SDK 需要 Python 3.10/3.11。{hint}"
        )
    for name in ("rclpy", "h5py"):
        spec = importlib.util.find_spec(name)
        if spec is None:
            raise RuntimeError(f"当前 Python 环境缺少 {name}。{hint}")
        version = re.search(r"/python(\d+)\.(\d+)/", spec.origin or "")
        if version and tuple(map(int, version.groups())) != sys.version_info[:2]:
            raise RuntimeError(f"{name} 来自其他 Python 版本：{spec.origin}。{hint}")


# Validate before importing ROS launch modules or starting any hardware process.
_check_python_environment()

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

sys.path.insert(0, str(ROOT))


def _launch(context):
    from data_collect.common.config import load_config, local_path

    cfg_path = LaunchConfiguration("config").perform(context)
    cfg = load_config(cfg_path)
    shares = {}
    for name in ("oculus_reader", "agx_arm_ctrl", "agx_arm_msgs", "agx_arm_description"):
        shares[name] = Path(get_package_share_directory(name)).resolve()
        if not shares[name].is_relative_to(ROOT / "ros_ws"):
            raise RuntimeError(f"{name} 不是内部包；请 source {ROOT}/env.sh")
    params = ["--ros-args"]
    values = {
        "can_port": LaunchConfiguration("can_port").perform(context),
        "arm_type": cfg["arm_type"],
        "effector_type": "agx_gripper",
        "pub_rate": "30",
        "fast_mode": LaunchConfiguration("fast_mode").perform(context),
        "auto_enable": LaunchConfiguration("auto_enable").perform(context),
        "speed_percent": LaunchConfiguration("speed_percent").perform(context),
        "home_speed_percent": LaunchConfiguration("home_speed_percent").perform(context),
        "home_position_tolerance": str(cfg["motion"]["home_tolerance_rad"]),
        "tcp_offset": str(cfg["tcp_offset"]),
        "control_enabled": "false",
        "external_command_watchdog_enabled": "true",
        "external_command_timeout": str(cfg["command_timeout_s"]),
    }
    for name, value in values.items():
        params.extend(["-p", f"{name}:={value}"])
    return [
        SetEnvironmentVariable("PYTHONPATH", str(ROOT) + os.pathsep + os.environ.get("PYTHONPATH", "")),
        SetEnvironmentVariable("ROS_LOG_DIR", str(Path(cfg["data_root"]) / "logs")),
        ExecuteProcess(
            cmd=[sys.executable, "-m", "data_collect.application.collector", "--config", cfg_path, *params],
            cwd=str(ROOT),
            output="screen",
        ),
        Node(
            package="oculus_reader",
            executable="arm_ik_pose_node.py",
            output="screen",
            parameters=[
                str(shares["oculus_reader"] / "config/arm_ik_pose_node.piper.yaml"),
                {
                    "pin_joint_status_topic": "/teleop/joint_states",
                    "robot_description_package": "agx_arm_description",
                    "urdf_relative_path": str(local_path(cfg["urdf"]).relative_to(ROOT / "ros_ws/src/agx_arm_description")),
                    "tool_pre_rot_rpy": [0.0, 0.0, 0.0],
                    "tool_translation_xyz": cfg["tcp_offset"][:3],
                    "tcp_offset_rpy": cfg["tcp_offset"][3:],
                    "require_start_alignment": True,
                    "alignment_position_tolerance_m": cfg["motion"]["fk_position_tolerance_m"],
                    "alignment_rotation_tolerance_rad": cfg["motion"]["fk_rotation_tolerance_rad"],
                },
            ],
        ),
        Node(
            package="oculus_reader",
            executable="pub_pose.py",
            output="screen",
            condition=IfCondition(LaunchConfiguration("quest_input")),
            parameters=[{"oculus_ip": "", "ros_to_arm_rpy": [0.0, 1.0471, 0.0]}],
        ),
        Node(
            package="oculus_reader",
            executable="pub_delta_pose.py",
            output="screen",
            condition=IfCondition(LaunchConfiguration("quest_input")),
            parameters=[
                {
                    "oculus_ip": "",
                    "translation_scale": [1.2, 1.2, 1.2],
                    "gripper_max_range": cfg["gripper_max_m"],
                    "control_joint_topic": "/teleop/joint_states",
                    "reset_button": "RJ",
                    "auto_home_on_stop": ParameterValue(
                        LaunchConfiguration("auto_home_on_stop"), value_type=bool
                    ),
                    "auto_home_delay_s": 0.5,
                }
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=str(ROOT / "config/collection.yaml")),
            DeclareLaunchArgument("can_port", default_value="can0"),
            DeclareLaunchArgument("auto_enable", default_value="true", choices=["true", "false"]),
            DeclareLaunchArgument("quest_input", default_value="true", choices=["true", "false"]),
            DeclareLaunchArgument("fast_mode", default_value="false", choices=["false", "true"]),
            DeclareLaunchArgument(
                "auto_home_on_stop", default_value="false", choices=["true", "false"]
            ),
            DeclareLaunchArgument("speed_percent", default_value="5"),
            DeclareLaunchArgument("home_speed_percent", default_value="5"),
            OpaqueFunction(function=_launch),
        ]
    )
