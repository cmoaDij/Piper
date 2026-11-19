import copy
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np

from data_collect.recording import storage
from data_collect.recording.timeline import RecordingTimeline
from data_collect.sensors.capture import CaptureWriter


def test_manager_resume_keeps_file_and_only_seeds_current_observations(tmp_path, config, monkeypatch):
    clock = NS(now=100)
    monkeypatch.setattr(storage.time, "monotonic_ns", lambda: clock.now)
    cfg = copy.deepcopy(config)
    cfg["data_root"] = str(tmp_path)
    manager = storage.EpisodeManager(cfg)
    manager.require_ready = lambda: None  # Synthetic observations at the device boundary.
    opened = []

    class Writer:
        def __init__(self, path, metadata, *args):
            self.path, self.metadata, self.error = Path(path), metadata, None
            self.timeline = RecordingTimeline(metadata["start_host_ts_ns"])
            self.thread = NS(is_alive=lambda: False)
            self.records = []
            opened.append(self)

        def submit(self, group, record, *, seed=False):
            prepared = self.timeline.prepare(group, record, seed=seed)
            if prepared is not None:
                self.records.append((group, prepared))
            return True

        def set_recording(self, active, stamp):
            self.timeline.set_recording(active, stamp)
            return True

        def finish(self, stamp, *args, **kwargs):
            self.timeline.set_recording(False, stamp)

    manager.writer_factory = Writer
    manager.emit("robot_feedback", {"available_host_ts_ns": np.int64(95), "is_new_packet": True})
    manager.emit("robot_command", {"cmd_call_begin_ns": np.int64(96), "following": False})
    path = manager.start()
    writer = manager.current
    assert not any(group == "robot_command" for group, _ in writer.records)
    manager.emit("robot_command", {"cmd_call_begin_ns": np.int64(105), "following": True})
    clock.now = 120
    manager.pause()
    count = len(writer.records)
    manager.emit("robot_feedback", {"available_host_ts_ns": np.int64(180), "is_new_packet": True})
    manager.emit("robot_command", {"cmd_call_begin_ns": np.int64(185), "following": False})
    assert len(writer.records) == count
    clock.now = 200
    assert manager.start() == path and manager.current is writer and len(opened) == 1
    assert writer.records[-1][0] == "robot_feedback"
    assert writer.records[-1][1]["available_host_ts_ns"] == 180  # Original timestamp retained.
    assert writer.records[-1][1]["recording_segment"] == 1
    manager.emit("robot_command", {"cmd_call_begin_ns": np.int64(205), "following": True})
    clock.now = 220
    manager.pause()
    clock.now = 300
    assert manager.stop() == path
    assert writer.timeline.intervals == [[100, 120], [200, 220]]
    assert writer.timeline.duration_s(clock.now) == 40e-9


def test_capture_ipc_stops_records_and_preserves_pause_resume_order(tmp_path):
    events = []
    runtime = NS(writers={}, send=lambda event: (events.append(event) or True))
    writer = CaptureWriter(runtime, tmp_path / "episode.h5",
                           {"start_host_ts_ns": 100, "recording_mode": "grip_paused"}, 100, 1024)
    assert writer.submit("robot_feedback", {"available_host_ts_ns": 105})
    assert writer.set_recording(False, 120)
    count = len(events)
    assert writer.submit("robot_feedback", {"available_host_ts_ns": 150})
    assert len(events) == count
    assert writer.set_recording(True, 200)
    count = len(events)
    assert writer.submit("robot_feedback", {"available_host_ts_ns": 180})
    assert len(events) == count  # No late packet from the pause enters the resumed interval.
    assert writer.submit("robot_feedback", {"available_host_ts_ns": 195}, seed=True)
    assert writer.submit("robot_feedback", {"available_host_ts_ns": 205})
    writer.finish(220)
    assert [event[0] for event in events] == ["open", "record", "recording", "recording",
                                           "record", "record", "finish"]
    assert events[2][2:] == (False, 120)
    assert events[3][2:] == (True, 200)
    assert writer.timeline.intervals == [[100, 120], [200, 220]]
