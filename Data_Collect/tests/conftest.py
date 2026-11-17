import pytest

from data_collect.common.config import load_config
from data_collect.common.config import atomic_json, fingerprint, local_path
from data_collect.calibration.tracking import CHANNELS, MEASUREMENT_KIND, load_tracking_latency


@pytest.fixture
def config():
    return load_config()


@pytest.fixture
def tracking_config(tmp_path, config):
    profile = {
        "schema_version": "piper_tracking_latency_v1", "measurement_kind": MEASUREMENT_KIND,
        "measurement_completed": True, "unit": "ms", "arm_type": config["arm_type"],
        "control_hz": config["control_hz"], "sdk_joint_method": "move_js",
        "speed_percent": config["teleop"]["speed_percent"], "tcp_offset": config["tcp_offset"],
        "urdf_sha256": fingerprint(local_path(config["urdf"])),
        "channels": {name: {"delay_ms": value, "complete": True, "dropped_samples": 0,
                            "at_search_boundary": False}
                     for name, value in zip(CHANNELS, (65, 80, 85, 90, 85, 80, 50))},
    }
    path = tmp_path / "tracking_calibration.json"
    atomic_json(path, profile)
    config["robot_tracking_latency_file"] = str(path)
    load_tracking_latency(config)
    return config
