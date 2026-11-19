import json

import h5py
import numpy as np
import pytest

from data_collect.common import config as config_module
from data_collect.common.config import atomic_json, fingerprint
from data_collect.dataset.audit import audit_batch, audit_episode, build_manifest
from data_collect.dataset.export import aligned_frames, convert, preflight
from data_collect.recording.images import validate_images
from tests.fixtures.episodes import write_episode


def rules(config):
    return {**config["audit"], "min_duration_s": .1}


def approve(tmp_path, config, *, paused=False):
    episode = write_episode(tmp_path / "raw", config, paused=paused)
    reports, manifest = audit_batch(episode.parent, tmp_path / "audit", rules(config))
    assert reports[0]["status"] == "PASS", reports
    assert manifest["episodes"] == []  # Automatic PASS alone never authorizes export.
    atomic_json(tmp_path / "audit/review_decisions.json", {
        episode.name: {"decision": "approve", "source_sha256": fingerprint(episode)}})
    build_manifest(tmp_path / "audit", rules(config))
    return episode, tmp_path / "audit/approved_episodes.json"


def test_recording_audit_review_and_preflight(tmp_path, config):
    episode, manifest = approve(tmp_path, config)
    with pytest.raises(ValueError, match="合成数据"):
        preflight(manifest)
    _, entries, shapes = preflight(manifest, allow_simulated=True)
    assert len(entries) == 1
    assert shapes["cameras/global_rgb"] == (16, 16, 3)
    with h5py.File(episode) as file:
        validate_images(file, decode=True)
        assert file.attrs["complete"]
        assert file["robot_command/following"][:].tolist() == [True] * 5 + [False] * 5 + [True] * 10
        frames = list(aligned_frames(file, entries[0]["sync"]))
        assert len(frames) == 20
        assert frames[0]["action"].shape == (7,)
        assert frames[0]["observation.images.tactile_left"].shape == (8, 8, 3)


def test_gripper_above_20mm_does_not_require_a_cycle_even_with_legacy_rules(tmp_path, config):
    episode = write_episode(tmp_path / "raw", config)
    # Existing local YAML files may still carry the removed rule. Ignore it.
    legacy = {**rules(config), "require_gripper_cycle": True,
              "open_threshold_m": .05, "closed_threshold_m": .02}
    report, sync = audit_episode(episode, legacy)
    with h5py.File(episode) as file:
        assert np.all(file["robot_feedback/gripper_pos"][:] > .02)
    assert report["status"] == "PASS", report
    assert "gripper_cycle_complete" not in report
    assert sync is not None


def test_grip_pause_skips_every_stream_and_resumes_same_episode(tmp_path, config):
    episode, manifest = approve(tmp_path, config, paused=True)
    _, entries, _ = preflight(manifest, allow_simulated=True)
    assert len(entries) == 1
    sync = entries[0]["sync"]
    assert sync["recording_segment"] == [0] * 5 + [1] * 10
    assert sync["recording_intervals_host_ts_ns"] == [[1_000_000_000, 1_250_000_000],
                                                       [3_500_000_000, 4_000_000_000]]
    with h5py.File(episode) as file:
        for group in (*config_module.IMAGE_STREAMS, "robot_feedback", "robot_command"):
            name = "cmd_call_begin_ns" if group == "robot_command" else "available_host_ts_ns"
            times = file[group + "/" + name][:]
            assert len(times) == 15
            assert not ((times >= 1_250_000_000) & (times < 3_500_000_000)).any()
            assert file[group + "/recording_segment"][:].tolist() == [0] * 5 + [1] * 10
        validate_images(file, decode=True)
        assert file["robot_command/following"][:].all()
        frames = list(aligned_frames(file, sync))
        assert len(frames) == 15
        assert frames[5]["source_host_ts_ns"].item() == 3_500_000_000
        assert frames[5]["recording_segment"].item() == 1
    report, _ = audit_episode(episode, rules(config))
    assert report["duration_s"] == .75
    assert report["wall_duration_s"] == 3.


def test_paused_episode_exports_one_episode_with_boundaries(tmp_path, config, monkeypatch):
    _, manifest = approve(tmp_path, config, paused=True)
    monkeypatch.setattr(config_module, "ROOT", tmp_path)
    output = convert(manifest, tmp_path / "export/paused", "local/test", True, FakeDataset)
    info = json.loads((output / "meta/info.json").read_text())
    assert info["total_episodes"] == 1 and info["total_frames"] == 15
    source = json.loads((output / "meta/source_episodes.json").read_text())[0]
    assert source["recording_segment"] == [0] * 5 + [1] * 10


def test_rejected_episode_cannot_enter_manifest(tmp_path, config):
    episode = write_episode(tmp_path / "raw", config, rejected=True)
    with h5py.File(episode) as file:
        assert not file.attrs["complete"]
        assert file.attrs["failure_kind"] == "episode_rejected"
    report, sync = audit_episode(episode, rules(config))
    assert report["status"] == "REJECT" and sync is None


def test_changed_video_invalidates_approved_recording(tmp_path, config):
    episode, manifest = approve(tmp_path, config)
    video = episode.with_name(episode.stem + ".global_rgb.mp4")
    with video.open("ab") as stream:
        stream.write(b"tamper")
    with pytest.raises(ValueError, match="SHA-256"):
        preflight(manifest, allow_simulated=True)


class FakeDataset:
    """Exercise export lifecycle without claiming to validate the external LeRobot package."""
    @classmethod
    def create(cls, repo_id, fps, features, root, robot_type, use_videos, image_writer_threads, vcodec):
        result = cls()
        result.root, result.frames, result.episodes = root, 0, 0
        result.features = features
        (root / "meta").mkdir(parents=True)
        return result

    def add_frame(self, frame):
        assert frame["action"].shape == (7,)
        if "tracking.reference" in self.features:
            assert frame["tracking.reference"].shape == frame["tracking.valid"].shape == (7,)
            assert frame["tracking.reference"].dtype == np.float32
            assert frame["tracking.valid"].dtype == np.int64
        self.frames += 1

    def save_episode(self, parallel_encoding):
        self.episodes += 1

    def finalize(self):
        atomic_json(self.root / "meta/info.json", {"codebase_version": "v3.0",
            "total_episodes": self.episodes, "total_frames": self.frames})
        atomic_json(self.root / "meta/stats.json", {})


def test_export_publishes_completed_staging_directory(tmp_path, config, monkeypatch):
    _, manifest = approve(tmp_path, config)
    monkeypatch.setattr(config_module, "ROOT", tmp_path)
    output = tmp_path / "export/dataset"
    assert convert(manifest, output, "local/test", True, FakeDataset) == output
    assert not output.with_name(output.name + ".partial").exists()
    info = json.loads((output / "meta/info.json").read_text())
    assert info["splits"] == {"train": "0:1"}
    with pytest.raises(FileExistsError):
        convert(manifest, output, "local/test", True, FakeDataset)


@pytest.mark.parametrize("paused", [False, True])
def test_delayed_gopro_images_match_scene_actions_through_audit_and_export(tmp_path, tracking_config, monkeypatch, paused):
    # A picture received 150 ms later must match the robot at its scene time,
    # not the robot at arrival. Exercise real MP4 encode/decode and HDF5 maps.
    config = tracking_config
    config["cameras"].pop("global_rgb")
    config["cameras"]["wrist_rgb"].update(backend="gopro", receive_latency_ns=150_000_000,
                                           receive_latency_sha256="synthetic_measured_latency")
    episode = write_episode(tmp_path / "raw", config, paused=paused, moving=True)
    reports, _ = audit_batch(episode.parent, tmp_path / "audit", rules(config))
    assert reports[0]["status"] == "PASS", reports
    atomic_json(tmp_path / "audit/review_decisions.json", {
        episode.name: {"decision": "approve", "source_sha256": fingerprint(episode)}})
    build_manifest(tmp_path / "audit", rules(config))
    manifest = tmp_path / "audit/approved_episodes.json"
    _, entries, shapes = preflight(manifest, allow_simulated=True)
    sync = entries[0]["sync"]
    assert "cameras/global_rgb" not in shapes
    assert sync["timestamp_policy"] == "gopro_calibrated_scene_time_offline_v1"
    with h5py.File(episode) as file:
        frames = list(aligned_frames(file, sync))
        camera = file["cameras/wrist_rgb"]
        for k, frame in enumerate(frames):
            i = sync["indices"]["cameras/wrist_rgb"][k]
            j = sync["indices"]["robot_command"][k]
            assert camera["capture_host_ts_ns"][i] == sync["grid_host_ts_ns"][k]
            assert camera["available_host_ts_ns"][i] == sync["grid_host_ts_ns"][k] + 150_000_000
            assert camera["frame_id"][i] == file["robot_command/cmd_seq"][j] - 1 + 3
            assert np.allclose(frame["action"][:6], (camera["frame_id"][i] - 3) * .001)
            # The encoded picture is the frame referenced by that timestamp.
            assert abs(frame["observation.images.camera_wrist"].mean() - camera["frame_id"][i]) <= 2
            assert "observation.images.camera_global" not in frame
            ref = np.asarray(sync["tracking_reference_indices"][k])
            valid = frame["tracking.valid"].astype(bool)
            np.testing.assert_array_equal(valid, ref >= 0)
            for channel in np.flatnonzero(valid):
                source = ref[channel]
                assert file["robot_command/expected_feedback_host_ts_ns"][source, channel] <= sync["grid_host_ts_ns"][k]
                assert frame["tracking.reference"][channel] == pytest.approx(
                    file["robot_command/target_joint_pos"][source, channel] if channel < 6
                    else file["robot_command/target_gripper_pos"][source, 0])
                if paused:
                    assert file["robot_command/recording_segment"][source] == sync["recording_segment"][k]
            if k == 0 or sync["recording_segment"][k] != sync["recording_segment"][k - 1]:
                assert not valid.any()  # No references before a segment's first response.
            if valid[0]:
                assert frame["tracking.reference"][0] < frame["action"][0]
        assert len(frames) == (9 if paused else 17)
        if paused:
            assert sync["recording_segment"] == [0] * 2 + [1] * 7
    monkeypatch.setattr(config_module, "ROOT", tmp_path)
    output = convert(manifest, tmp_path / "export/gopro", "local/gopro", True, FakeDataset)
    assert json.loads((output / "meta/info.json").read_text())["total_frames"] == len(frames)
    timing = json.loads((output / "meta/robot_tracking_latency.json").read_text())
    assert timing["calibration"] == config["robot_tracking_latency"]
    assert timing["state_timestamps_shifted"] is False


def test_tracking_uses_recorded_snapshot_and_rejects_modified_response_cache(tmp_path, tracking_config):
    episode, manifest_path = approve(tmp_path, tracking_config)
    # Old episodes remain interpretable if the current external profile changes.
    atomic_json(tracking_config["robot_tracking_latency_file"], {"new_calibration": True})
    _, entries, _ = preflight(manifest_path, allow_simulated=True)
    sync_path = entries[0]["sync_path"]
    sync = entries[0]["sync"]
    sync["tracking_reference_indices"][0][0] = 0  # A future response at the first step.
    atomic_json(sync_path, sync)
    manifest = json.loads(manifest_path.read_text())
    manifest["episodes"][0]["sync_sha256"] = fingerprint(sync_path)
    atomic_json(manifest_path, manifest)
    with pytest.raises(ValueError, match="响应参考索引"):
        preflight(manifest_path, allow_simulated=True)
    with h5py.File(episode, "r+") as file:
        file["robot_command/expected_feedback_host_ts_ns"][0, 0] += 1
    report, _ = audit_episode(episode, rules(tracking_config))
    assert report["status"] == "REJECT" and "预计响应时间" in str(report["reasons"])


@pytest.mark.parametrize("feedback_lag_ms", [0, 30])
def test_explicit_short_segment_diagnostic_checks_remaining_gates_without_export(tmp_path, config, feedback_lag_ms):
    config["cameras"].pop("global_rgb")
    config["cameras"]["wrist_rgb"].update(
        backend="gopro", receive_latency_ns=300_000_000,
        receive_latency_sha256="synthetic_latency")
    episode = write_episode(tmp_path / "raw", config, paused=True)
    if feedback_lag_ms:
        with h5py.File(episode, "r+") as file:
            for part in ("joint", "tcp", "gripper"):
                field = file[f"robot_feedback/{part}_available_host_ts_ns"]
                field[:] = field[:] - feedback_lag_ms * 1_000_000
    source_hash = fingerprint(episode)
    strict, _ = audit_episode(episode, rules(config))
    assert "无对应 GoPro 图像" in str(strict["reasons"])
    report, sync = audit_episode(episode, rules(config), diagnostic_excluded_segments=(0,))
    assert report["diagnostic_only"] is True
    assert report["excluded_recording_segments"] == [0]
    assert report["evaluated_recording_segments"] == [1]
    assert report["excluded_duration_s"] == .25
    assert report["duration_s"] == .75
    assert report["evaluated_duration_s"] == .5
    assert report["robot_max_age_ms"] == feedback_lag_ms
    assert report["status"] == ("REJECT" if feedback_lag_ms else "PASS"), report
    if feedback_lag_ms:
        assert "robot_age_red" in report["reasons"]
    assert sync is None  # A selected-subset diagnosis cannot authorize export.
    assert fingerprint(episode) == source_hash
    invalid, _ = audit_episode(episode, rules(config), diagnostic_excluded_segments=(2,))
    assert "片段编号非法" in str(invalid["reasons"])


def test_gopro_without_timing_calibration_is_rejected(tmp_path, config):
    config["cameras"].pop("global_rgb")
    config["cameras"]["wrist_rgb"].update(backend="gopro", receive_latency_ns=150_000_000)
    episode = write_episode(tmp_path / "raw", config)
    report, sync = audit_episode(episode, rules(config))
    assert report["status"] == "REJECT" and sync is None
    assert "接收延迟标定" in str(report["reasons"])


def test_relative_latency_tool_supports_single_gopro_video_and_tactile(tmp_path, config):
    from data_collect.calibration.latency import _features, measure

    config["cameras"].pop("global_rgb")
    config["cameras"]["wrist_rgb"].update(backend="gopro", receive_latency_ns=150_000_000,
                                           receive_latency_sha256="synthetic_measured_latency")
    episode = write_episode(tmp_path / "raw", config)
    with h5py.File(episode) as file:
        _, features = _features(file["cameras/wrist_rgb"])
        assert len(features) == 20  # Real MP4 reader, not an HDF5 image array.
    rows = ["event_id,stream,frame_id"]
    for event, frame_id in enumerate((0, 3, 6, 9, 12)):
        rows.extend((f"{event},wrist_rgb,{frame_id + 3}", f"{event},left_raw,{frame_id}",
                     f"{event},right_raw,{frame_id}"))
    events = tmp_path / "events.csv"
    events.write_text("\n".join(rows) + "\n")
    result = measure(episode, events=events)
    assert result["relative_offsets_ns"] == {"wrist_rgb": 0, "left_raw": 150_000_000,
                                               "right_raw": 150_000_000}
    assert result["absolute_latency_measured"] is False
