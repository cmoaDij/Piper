import json

import h5py
import pytest

from data_collect.common import config as config_module
from data_collect.common.config import atomic_json, fingerprint
from data_collect.dataset.audit import audit_batch, audit_episode, build_manifest
from data_collect.dataset.export import aligned_frames, convert, preflight
from data_collect.recording.images import validate_images
from tests.fixtures.episodes import write_episode


def rules(config):
    # Short synthetic tests have no physical gripper cycle; production YAML stays unchanged.
    return {**config["audit"], "min_duration_s": .1, "require_gripper_cycle": False}


def approve(tmp_path, config):
    episode = write_episode(tmp_path / "raw", config)
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
        frames = list(aligned_frames(file, entries[0]["sync"]))
        assert len(frames) == 20
        assert frames[0]["action"].shape == (7,)
        assert frames[0]["observation.images.tactile_left"].shape == (8, 8, 3)


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
        (root / "meta").mkdir(parents=True)
        return result

    def add_frame(self, frame):
        assert frame["action"].shape == (7,)
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
