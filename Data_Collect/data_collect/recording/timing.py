"""Offline scene-time synchronization for delayed GoPro observations.

Receive time remains intact. Only the GoPro's independently calibrated scene
time is used for offline robot/image matching; it is not an availability claim.
"""

import numpy as np

from data_collect.common.config import recorded_image_streams
from data_collect.recording.timeline import episode_grid


def observation_timestamps(group):
    available = group["available_host_ts_ns"][:]
    if group.attrs.get("backend") != "gopro":
        return available
    latency = group.attrs.get("receive_latency_ns")
    if (not isinstance(latency, (int, np.integer)) or latency < 0
            or not group.attrs.get("receive_latency_sha256")):
        raise ValueError(f"{group.name} 未记录 GoPro 接收延迟标定")
    scene = group["capture_host_ts_ns"][:]
    if (scene.shape != available.shape or scene.dtype.kind not in "iu"
            or (scene <= 0).any() or (np.diff(scene) < 0).any()
            or not np.array_equal(available - scene, np.full(len(scene), latency))):
        raise ValueError(f"{group.name} GoPro 图像时间与接收延迟不符")
    return scene


def alignment_grid(file, fps, *, excluded_segments=()):
    grid, segments, intervals = episode_grid(file.attrs, fps)
    excluded = set(excluded_segments)
    if any(type(s) is not int or not 0 <= s < len(intervals) for s in excluded):
        raise ValueError("排除的录制片段编号非法")
    keep = ~np.isin(segments, list(excluded))
    for stream in recorded_image_streams(file):
        group = file[stream]
        if group.attrs.get("backend") != "gopro":
            continue
        scene = observation_timestamps(group)
        source_segments = (group["recording_segment"][:] if file.attrs.get("recording_mode") == "grip_paused"
                           else np.zeros(len(scene), dtype=np.int64))
        for segment, (lower, upper) in enumerate(intervals):
            if segment in excluded:
                continue
            valid = scene[(source_segments == segment) & (scene >= lower) & (scene < upper)]
            if not len(valid):
                raise ValueError(f"{stream} 录制片段 {segment} 无对应 GoPro 图像（片段可能短于接收延迟）")
            in_segment = segments == segment
            keep &= ~in_segment | ((grid >= valid[0]) & (grid <= valid[-1]))
    if not keep.any() or any(not np.any(keep & (segments == s)) for s in np.unique(segments) if s not in excluded):
        raise ValueError("GoPro 有效图像不足以覆盖每个录制片段的同步采样点")
    return grid[keep], segments[keep], intervals
