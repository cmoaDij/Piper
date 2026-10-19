"""Safety contract tests for the standard USB teleoperation launch."""

from pathlib import Path


PACKAGE_DIR = Path(__file__).resolve().parents[1]


def test_usb_launch_routes_commands_through_mux():
    launch_source = (
        PACKAGE_DIR / "launch" / "teleop_single_piper_usb.launch.py"
    ).read_text(encoding="utf-8")

    assert 'default_value="/teleop/joint_states"' in launch_source
    assert 'package="piper_policy_bridge"' in launch_source
    assert 'executable="piper_command_mux"' in launch_source
    assert '"hardware_topic": "/control/joint_states"' in launch_source
    assert '"control_enabled": "true"' in launch_source
    assert '"home_position_tolerance"' in launch_source
    assert 'default_value="0.10"' in launch_source
    assert '"quest_input"' in launch_source
    assert "IfCondition" in launch_source


def test_usb_launch_defaults_to_piper_v158_model_not_piper_x():
    launch_source = (
        PACKAGE_DIR / "launch" / "teleop_single_piper_usb.launch.py"
    ).read_text(encoding="utf-8")

    assert 'default_value="piper"' in launch_source
    assert 'default_value="piper/urdf/piper_with_gripper_description.urdf"' in launch_source
    assert 'default_value="piper_x"' not in launch_source


def test_oculus_reader_declares_mux_runtime_dependency():
    package_xml = (PACKAGE_DIR / "package.xml").read_text(encoding="utf-8")

    assert "<exec_depend>piper_policy_bridge</exec_depend>" in package_xml


def test_piper_x_launch_uses_usb_mux_and_safe_defaults():
    launch_source = (
        PACKAGE_DIR / "launch" / "teleop_single_piper_x.launch.py"
    ).read_text(encoding="utf-8")

    assert '"teleop_single_piper_usb.launch.py"' in launch_source
    assert '"arm_type": "piper_x"' in launch_source
    assert '"gripper_max_range": "0.03"' in launch_source
    assert 'default_value="5"' in launch_source
    assert 'default_value="0.03"' in launch_source
    assert '"ros_to_arm_roll", default_value="-1.5708"' in launch_source
    assert '"ros_to_arm_pitch", default_value="0.0"' in launch_source
    assert '"ros_to_arm_yaw", default_value="-1.5708"' in launch_source


def test_usb_launch_builds_controller_rotation_as_float_array():
    launch_source = (
        PACKAGE_DIR / "launch" / "teleop_single_piper_usb.launch.py"
    ).read_text(encoding="utf-8")

    assert "def _launch_pub_pose_node(context):" in launch_source
    assert "float(LaunchConfiguration(name).perform(context))" in launch_source
    assert '"ros_to_arm_rpy": ros_to_arm_rpy' in launch_source
    assert '"ros_to_arm_rpy": [' not in launch_source
