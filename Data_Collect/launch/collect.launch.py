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
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessIO, OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.logging import launch_config

sys.path.insert(0, str(ROOT))


def _launch(context):
    from data_collect.common.config import load_config, local_path

    cfg_path = LaunchConfiguration("config").perform(context)
    cfg = load_config(cfg_path)
    verbose = LaunchConfiguration("verbose").perform(context) == "true"
    # The ROS alias "log" still echoes stderr. Use an explicit destination
    # so only the filtered IO handler writes to the terminal in concise mode.
    output = "screen" if verbose else {"both": "log"}
    console_actions = []
    if not verbose:
        from data_collect.common.console import CollectionConsole

        console = CollectionConsole(lambda line: print(line, flush=True))
        console_actions = [
            RegisterEventHandler(OnProcessIO(on_stdout=console.on_output, on_stderr=console.on_output)),
            RegisterEventHandler(OnProcessExit(on_exit=console.on_exit)),
        ]
    shares = {}
    for name in ("oculus_reader", "agx_arm_ctrl", "agx_arm_msgs", "agx_arm_description"):
        shares[name] = Path(get_package_share_directory(name)).resolve()
        if not shares[name].is_relative_to(ROOT / "ros_ws"):
            raise RuntimeError(f"{name} 不是内部包；请 source {ROOT}/env.sh")

    def teleop_option(name):
        override = LaunchConfiguration(name).perform(context)
        return override or str(cfg["teleop"][name]).lower()

    controller_rpy = [
        float(LaunchConfiguration("ros_to_arm_" + axis).perform(context) or default)
        for axis, default in zip(("roll", "pitch", "yaw"), cfg["teleop"]["ros_to_arm_rpy"])
    ]
    params = ["--ros-args"]
    values = {
        "can_port": LaunchConfiguration("can_port").perform(context),
        "arm_type": cfg["arm_type"],
        "effector_type": "agx_gripper",
        "pub_rate": "30",
        "fast_mode": teleop_option("fast_mode"),
        "auto_enable": LaunchConfiguration("auto_enable").perform(context),
        "speed_percent": teleop_option("speed_percent"),
        "home_speed_percent": teleop_option("home_speed_percent"),
        "home_position_tolerance": str(cfg["teleop"]["home_position_tolerance"]),
        "tcp_offset": str(cfg["tcp_offset"]),
        "control_enabled": "true",
        "external_command_watchdog_enabled": "false",
    }
    for name, value in values.items():
        params.extend(["-p", f"{name}:={value}"])
    collector_options = ["--config", cfg_path]
    if LaunchConfiguration("teleop_only").perform(context) == "true":
        collector_options.append("--teleop-only")
    return [
        *console_actions,
        LogInfo(msg=(f"[启动] {cfg['arm_type']} | CAN {LaunchConfiguration('can_port').perform(context)}"
                     f" | 控制 {cfg['control_hz']} Hz | 配置 {local_path(cfg_path).name}"
                     f" | 完整终端日志: {launch_config.log_dir}/launch.log")),
        SetEnvironmentVariable("PYTHONPATH", str(ROOT) + os.pathsep + os.environ.get("PYTHONPATH", "")),
        # SDK print() output should appear during startup, not flush in a
        # misleading burst after Ctrl-C alongside the SDK's logging output.
        SetEnvironmentVariable("PYTHONUNBUFFERED", "1"),
        SetEnvironmentVariable("ROS_LOG_DIR", str(Path(cfg["data_root"]) / "logs")),
        ExecuteProcess(
            cmd=[sys.executable, "-m", "data_collect.application.collector", *collector_options, *params],
            cwd=str(ROOT),
            output=output,
        ),
        Node(
            package="oculus_reader",
            executable="arm_ik_pose_node.py",
            output=output,
            parameters=[
                str(shares["oculus_reader"] / "config/arm_ik_pose_node.piper.yaml"),
                {
                    "pin_joint_status_topic": "/teleop/joint_states",
                    "reference_joint_topic": "/teleop/ik_reference",
                    "robot_description_package": "agx_arm_description",
                    "urdf_relative_path": str(local_path(cfg["urdf"]).relative_to(ROOT / "ros_ws/src/agx_arm_description")),
                    "tool_pre_rot_rpy": [0.0, 0.0, 0.0],
                    "tool_translation_xyz": cfg["tcp_offset"][:3],
                    "tcp_offset_rpy": cfg["tcp_offset"][3:],
                },
            ],
        ),
        Node(
            package="oculus_reader",
            executable="pub_pose.py",
            output=output,
            condition=IfCondition(LaunchConfiguration("quest_input")),
            parameters=[{"oculus_ip": "", "ros_to_arm_rpy": controller_rpy}],
        ),
        Node(
            package="oculus_reader",
            executable="pub_delta_pose.py",
            output=output,
            condition=IfCondition(LaunchConfiguration("quest_input")),
            parameters=[
                {
                    "oculus_ip": "",
                    "translation_scale": [1.0, 1.0, 1.0],
                    "delta_frame": teleop_option("delta_frame"),
                    "gripper_max_range": cfg["gripper_max_m"],
                    "control_joint_topic": "/teleop/joint_states",
                    "reset_button": "RJ",
                    "follow_button": cfg["teleop"]["follow_button"],
                    "auto_home_on_stop": teleop_option("auto_home_on_stop") == "true",
                    "auto_home_delay_s": cfg["teleop"]["auto_home_delay_s"],
                }
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=str(ROOT / "config/collection.yaml")),
            DeclareLaunchArgument("can_port", default_value="can0"),
            DeclareLaunchArgument("verbose", default_value="false", choices=["true", "false"],
                                  description="显示全部节点输出；默认仅显示采集状态、回零和警告/错误"),
            DeclareLaunchArgument(
                "teleop_only", default_value="false", choices=["true", "false"],
                description="只运行遥操，不启动相机、触觉或录制",
            ),
            DeclareLaunchArgument("auto_enable", default_value="true", choices=["true", "false"]),
            DeclareLaunchArgument("quest_input", default_value="true", choices=["true", "false"]),
            DeclareLaunchArgument("fast_mode", default_value="", choices=["", "false", "true"]),
            DeclareLaunchArgument(
                "auto_home_on_stop", default_value="", choices=["", "true", "false"]
            ),
            DeclareLaunchArgument("speed_percent", default_value=""),
            DeclareLaunchArgument("home_speed_percent", default_value=""),
            DeclareLaunchArgument(
                "delta_frame", default_value="", choices=["", "base", "tcp"],
                description="base 按基座方向映射；tcp 复现 QuestArmTeleop 的局部增量",
            ),
            *[
                DeclareLaunchArgument(
                    "ros_to_arm_" + axis, default_value="",
                    description="手柄坐标修正（rad）；留空使用 collection.yaml，与 QuestArmTeleop 参数同名",
                )
                for axis in ("roll", "pitch", "yaw")
            ],
            OpaqueFunction(function=_launch),
        ]
    )
