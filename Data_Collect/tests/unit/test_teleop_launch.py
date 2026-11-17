"""Inspect real launch argument resolution without executing any action."""

import importlib.util
import sys
from types import SimpleNamespace

import pytest

from data_collect.common.paths import ROOT


@pytest.mark.parametrize("override,delta_frame", [(None, ""), ([0.0, 1.0471, 0.0], "base")])
@pytest.mark.parametrize("verbose", [False, True])
def test_controller_rotation_reaches_pose_node_and_tcp_model_stays_consistent(config, monkeypatch, override, delta_frame, verbose):
    if sys.version_info[:2] not in ((3, 10), (3, 11)):
        pytest.skip("生产 launch 需要 Python 3.10/3.11")
    pytest.importorskip("launch_ros")
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    from launch.utilities import perform_substitutions

    spec = importlib.util.spec_from_file_location("collect_launch", ROOT / "launch/collect.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, "get_package_share_directory", lambda name: str(ROOT / "ros_ws/src" / name))
    monkeypatch.setattr(module, "Node", lambda **kwargs: SimpleNamespace(**kwargs))
    context = LaunchContext()
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            context.launch_configurations[action.name] = perform_substitutions(context, action.default_value)
    if override is not None:
        for name, value in zip(("roll", "pitch", "yaw"), override):
            context.launch_configurations["ros_to_arm_" + name] = str(value)
    context.launch_configurations["delta_frame"] = delta_frame
    context.launch_configurations["verbose"] = str(verbose).lower()
    actions = module._launch(context)
    from launch.actions import SetEnvironmentVariable

    environment = {perform_substitutions(context, action.name): perform_substitutions(context, action.value)
                   for action in actions if isinstance(action, SetEnvironmentVariable)}
    assert environment["PYTHONUNBUFFERED"] == "1"
    nodes = [action for action in actions if isinstance(action, SimpleNamespace)]
    assert all(node.output == ("screen" if verbose else {"both": "log"}) for node in nodes)
    pose = next(node for node in nodes if node.executable == "pub_pose.py")
    ik = next(node for node in nodes if node.executable == "arm_ik_pose_node.py")
    delta = next(node for node in nodes if node.executable == "pub_delta_pose.py")
    assert pose.parameters[0]["ros_to_arm_rpy"] == (override or config["teleop"]["ros_to_arm_rpy"])
    assert ik.parameters[1]["urdf_relative_path"] == str(
        (ROOT / config["urdf"]).relative_to(ROOT / "ros_ws/src/agx_arm_description")
    )
    assert ik.parameters[1]["tool_translation_xyz"] == config["tcp_offset"][:3]
    assert ik.parameters[1]["tcp_offset_rpy"] == config["tcp_offset"][3:]
    assert ik.parameters[1]["reference_joint_topic"] == "/teleop/ik_reference"
    assert delta.parameters[0]["translation_scale"] == [1.0, 1.0, 1.0]
    assert delta.parameters[0]["follow_button"] == config["teleop"]["follow_button"] == "RG"
    assert delta.parameters[0]["auto_home_on_stop"] is True
    assert config["teleop"]["delta_frame"] == "tcp"
    assert delta.parameters[0]["delta_frame"] == (delta_frame or "tcp")


def test_gopro_profile_restores_automatic_home_at_existing_slow_speed():
    from data_collect.common.config import load_config

    cfg = load_config("config/collection_gopro.yaml")
    assert cfg["teleop"]["auto_home_on_stop"] is True
    assert cfg["teleop"]["home_speed_percent"] == 5
    assert cfg["teleop"]["auto_home_delay_s"] == .5
