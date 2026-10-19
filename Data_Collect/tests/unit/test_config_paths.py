import json
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
