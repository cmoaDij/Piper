import ast
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

from data_collect.application.collector import validate_calibration
from data_collect.common.config import inside_data_collect, load_config, local_path
from data_collect.common.paths import ROOT


def test_project_paths_do_not_depend_on_working_directory(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    cfg = load_config()
    assert ROOT == Path(__file__).resolve().parents[2]
    assert local_path(cfg["urdf"]).is_file()
    assert Path(cfg["data_root"]) == ROOT / "data"
    assert local_path("launch/collect.launch.py").is_file()
    assert (ROOT / "ros_ws/src/agx_arm_description/agx_arm_urdf/piper/urdf/piper_with_gripper_description_old.urdf").is_file()


def test_output_cannot_escape_checkout(tmp_path):
    with pytest.raises(ValueError, match="数据输出必须"):
        inside_data_collect(tmp_path / "outside")
    with pytest.raises(ValueError):
        inside_data_collect("../outside")


def test_old_model_is_retained_but_not_silently_selected(config, tmp_path):
    cfg = {**config, "arm_type": "piper"}
    path = tmp_path / "invalid.yaml"
    path.write_text(yaml.safe_dump(cfg))
    with pytest.raises(ValueError, match="Piper-X"):
        load_config(path)


def test_calibration_template_requires_explicit_verification(config, tmp_path):
    template = json.loads((ROOT / "config/calibration.example.json").read_text())
    path = tmp_path / "calibration.json"
    path.write_text(json.dumps(template))
    config["calibration_file"] = str(path)
    with pytest.raises(ValueError, match="核验"):
        validate_calibration(config)
    template["verified"] = True
    path.write_text(json.dumps(template))
    validate_calibration(config)
    template["tactile_ids"]["left_raw"] = "wrong-device"
    path.write_text(json.dumps(template))
    with pytest.raises(ValueError, match="标定中的设备不符"):
        validate_calibration(config)


def test_tcp_is_195mm_in_driver_ik_and_calibration_template(config):
    expected = [0.0, 0.0, 0.195, 0.0, 0.0, 0.0]
    template = json.loads((ROOT / "config/calibration.example.json").read_text())
    ik = yaml.safe_load((ROOT / "ros_ws/src/oculus_reader/config/arm_ik_pose_node.piper.yaml").read_text())
    assert config["tcp_offset"] == template["tcp_offset"] == expected
    assert ik["/**"]["ros__parameters"]["tool_translation_xyz"] == expected[:3]


def test_piper_x_joint6_matches_official_urdf_and_sdk_range(config):
    from pyAgxArm.api.constants import ROBOT_JOINT_LIMIT_PRESET_RAD

    model = ET.parse(local_path(config["urdf"])).getroot()
    limit = model.find("./joint[@name='joint6']/limit")
    bounds = [float(limit.get(name)) for name in ("lower", "upper")]
    assert bounds == [-2.9670597, 2.9670597]
    # SDK constants round radians to six decimals; the URDF keeps seven.
    assert bounds == pytest.approx(ROBOT_JOINT_LIMIT_PRESET_RAD["piper_x"]["joint6"], abs=1e-6)


def test_controller_frame_matches_quest_piper_x_launch(config):
    reference = ROOT.parent / "Piper_DM (2)/QuestArmTeleop/src/oculus_reader/launch/teleop_single_piper_x.launch.py"
    if not reference.exists():
        pytest.skip("独立部署目录未包含 QuestArmTeleop 参考工程")
    defaults = {}
    for call in ast.walk(ast.parse(reference.read_text())):
        if not isinstance(call, ast.Call) or not isinstance(call.func, ast.Name):
            continue
        if call.func.id == "DeclareLaunchArgument" and call.args:
            name = ast.literal_eval(call.args[0])
            if name in ("ros_to_arm_roll", "ros_to_arm_pitch", "ros_to_arm_yaw"):
                default = next(item.value for item in call.keywords if item.arg == "default_value")
                defaults[name] = float(ast.literal_eval(default))
    assert config["teleop"]["ros_to_arm_rpy"] == [
        defaults[name] for name in ("ros_to_arm_roll", "ros_to_arm_pitch", "ros_to_arm_yaw")
    ]
