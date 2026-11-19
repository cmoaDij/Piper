"""Recorded host-time intervals, with the physical clock left unchanged."""

import json

import numpy as np


class RecordingTimeline:
    """Access under the owning writer's lock."""

    def __init__(self, start_ns):
        self.intervals = [[int(start_ns), None]]

    @property
    def active(self):
        return self.intervals[-1][1] is None

    def set_recording(self, active, stamp_ns):
        if bool(active) == self.active:
            return
        stamp_ns = int(stamp_ns)
        if stamp_ns < self.intervals[-1][0 if self.active else 1]:
            raise ValueError("录制切换时间不能倒退")
        if active:
            self.intervals.append([stamp_ns, None])
        else:
            self.intervals[-1][1] = stamp_ns

    def duration_s(self, now_ns):
        return sum((end if end is not None else now_ns) - start
                   for start, end in self.intervals) / 1e9

    def prepare(self, group, record, *, seed=False):
        if not self.active:
            return None
        key = "cmd_call_begin_ns" if group == "robot_command" else "available_host_ts_ns"
        if group.startswith("cameras/") and "capture_host_ts_ns" in record:
            # Receive time can be in the resumed segment while the scene was
            # captured during the pause. Even a seed must obey scene time.
            key = "capture_host_ts_ns"
            if int(record[key]) < self.intervals[-1][0]:
                return None
        if not seed and int(record[key]) < self.intervals[-1][0]:
            return None
        return {**record, "recording_segment": np.int64(len(self.intervals) - 1)}


def episode_grid(attrs, fps):
    """Return every active grid and its segment, including legacy files."""
    start, end = int(attrs["start_host_ts_ns"]), int(attrs["end_host_ts_ns"])
    if end <= start:
        raise ValueError("Episode 时间范围非法")
    intervals = (json.loads(attrs["recording_intervals_host_ts_ns"])
                 if attrs.get("recording_mode") == "grip_paused" else [[start, end]])
    grids, segments = [], []
    previous_end = start
    for segment, (lower, upper) in enumerate(intervals):
        if not (isinstance(lower, int) and isinstance(upper, int)
                and previous_end <= lower <= upper <= end):
            raise ValueError("录制有效区间非法")
        grid = np.arange(lower, upper, 1_000_000_000 // fps, dtype=np.int64)
        grids.append(grid)
        segments.append(np.full(len(grid), segment, dtype=np.int64))
        previous_end = upper
    if not grids or not any(len(grid) for grid in grids):
        raise ValueError("Episode 没有有效录制区间")
    return np.concatenate(grids), np.concatenate(segments), intervals
