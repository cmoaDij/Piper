import json
from types import SimpleNamespace as NS

import h5py
import numpy as np
import pytest

from data_collect.calibration.tracking import load_tracking_latency, response_reference_indices
from data_collect.common.config import atomic_json
from data_collect.control.robot import RobotObserver


def test_profile_loads_all_seven_channels_without_changing_camera_or_feedback(tracking_config):
    timing = load_tracking_latency(tracking_config)
    assert timing["latency_ns"] == [65_000_000, 80_000_000, 85_000_000, 90_000_000,
                                     85_000_000, 80_000_000, 50_000_000]
    assert "receive_latency_ns" not in tracking_config["cameras"]["wrist_rgb"]


@pytest.mark.parametrize("field,value", [("delay_ms", float("nan")), ("delay_ms", -1),
                                         ("delay_ms", True), ("at_search_boundary", True),
                                         ("complete", False), ("dropped_samples", 1)])
def test_incomplete_or_invalid_channel_is_rejected(tracking_config, field, value):
    path = tracking_config["robot_tracking_latency_file"]
    profile = json.loads(open(path).read())
    profile["channels"]["joint2"][field] = value
    # NaN is intentionally malformed input; production atomic_json disallows it.
    with open(path, "w") as stream:
        json.dump(profile, stream)
    with pytest.raises(ValueError, match="joint2"):
        load_tracking_latency(tracking_config)


def test_profile_requires_gripper_channel(tracking_config):
    path = tracking_config["robot_tracking_latency_file"]
    profile = json.loads(open(path).read())
    del profile["channels"]["gripper"]
    atomic_json(path, profile)
    with pytest.raises(ValueError, match="六关节和夹爪"):
        load_tracking_latency(tracking_config)


@pytest.mark.parametrize("setting", ["mode", "speed", "tcp", "urdf"])
def test_changed_measurement_conditions_are_rejected(tracking_config, setting, tmp_path):
    if setting == "mode":
        tracking_config["teleop"]["fast_mode"] = False
    elif setting == "speed":
        tracking_config["teleop"]["speed_percent"] = 25
    elif setting == "tcp":
        tracking_config["tcp_offset"][2] = .2
    else:
        urdf = tmp_path / "changed.urdf"
        urdf.write_text("changed robot model")
        tracking_config["urdf"] = str(urdf)
    with pytest.raises(ValueError, match="当前设置不符"):
        load_tracking_latency(tracking_config)


def test_observer_retains_each_component_call_time(tracking_config, tmp_path):
    records = []
    observer = RobotObserver(NS(emit=lambda _, value: records.append(value)),
                             tracking_latency=tracking_config["robot_tracking_latency"])
    observer.command("joint", np.zeros(6), 1_000_000_000, 1_000_000_001)
    observer.command("gripper", .06, 1_001_000_000, 1_001_000_001)
    observer.command("joint", np.ones(6), 1_020_000_000, 1_020_000_001)
    row = records[-1]
    assert row["cmd_call_begin_ns"] == row["joint_cmd_call_begin_ns"] == 1_020_000_000
    assert row["gripper_cmd_call_begin_ns"] == 1_001_000_000
    np.testing.assert_array_equal(row["expected_feedback_host_ts_ns"],
                                  [1_085_000_000, 1_100_000_000, 1_105_000_000,
                                   1_110_000_000, 1_105_000_000, 1_100_000_000, 1_051_000_000])
    observer.command("gripper", .05, 1_200_000_000, 1_200_000_001)
    with h5py.File(tmp_path / "held_component.h5", "w") as file:
        file.attrs["config"] = json.dumps({"robot_tracking_latency": tracking_config["robot_tracking_latency"]})
        command = file.create_group("robot_command")
        for key in records[0]:
            command[key] = np.asarray([record[key] for record in records])
        indices = response_reference_indices(file, [1_100_000_000], [0],
                                             [[1_000_000_000, 1_300_000_000]])
        assert indices[0, 0] == 1  # Do not reference the future gripper call's held joint target.


def test_short_pause_does_not_reuse_prior_segment_component(tracking_config, tmp_path):
    from data_collect.calibration.tracking import response_fields

    with h5py.File(tmp_path / "short_pause.h5", "w") as file:
        timing = tracking_config["robot_tracking_latency"]
        file.attrs["config"] = json.dumps({"robot_tracking_latency": timing})
        file.attrs["recording_mode"] = "grip_paused"
        command = file.create_group("robot_command")
        command["cmd_call_begin_ns"] = [1_000_000_000, 1_030_000_000]
        command["recording_segment"] = [0, 1]
        rows = [response_fields(1_000_000_000, 1_000_000_000, timing),
                response_fields(1_030_000_000, 1_000_000_000, timing)]
        for key in rows[0]:
            command[key] = np.asarray([row[key] for row in rows])
        indices = response_reference_indices(file, [1_050_000_000, 1_100_000_000], [1, 1],
                                             [[1_000_000_000, 1_020_000_000],
                                              [1_030_000_000, 1_150_000_000]])
        assert (indices[:, 6] == -1).all()
        assert indices[0, 0] == -1 and indices[1, 0] == 1
