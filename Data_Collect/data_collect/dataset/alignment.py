"""Strict causal observations and nearest-window / ZOH command matching."""

import numpy as np


def causal_indices(timestamps, grid, valid=None):
    timestamps = np.asarray(timestamps, dtype=np.int64)
    source = np.arange(len(timestamps))
    if valid is not None:
        source = source[np.asarray(valid, dtype=bool)]
    if len(source) == 0:
        raise ValueError("没有有效真实样本")
    position = np.searchsorted(timestamps[source], grid, side="right") - 1
    if (position < 0).any():
        raise ValueError("Episode 起点缺少因果观测；不会裁掉开头")
    indices = source[position]
    return indices, grid - timestamps[indices]


def action_indices(timestamps, grid, half_window_ns=25_000_000):
    timestamps = np.asarray(timestamps, dtype=np.int64)
    result, held = [], []
    for time_ns in grid:
        right = np.searchsorted(timestamps, time_ns, side="left")
        candidates = [
            i
            for i in (right - 1, right)
            if 0 <= i < len(timestamps) and abs(int(timestamps[i]) - int(time_ns)) <= half_window_ns
        ]
        if candidates:
            # Ties prefer the past. Future targets are permitted only by the action window.
            index = min(
                candidates, key=lambda i: (abs(int(timestamps[i]) - int(time_ns)), timestamps[i])
            )
            held.append(False)
        else:
            index = np.searchsorted(timestamps, time_ns, side="right") - 1
            if index < 0:
                raise ValueError("Episode 起点无可匹配指令")
            held.append(True)
        result.append(index)
    return np.asarray(result, dtype=np.int64), np.asarray(held, dtype=bool)
