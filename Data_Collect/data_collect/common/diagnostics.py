"""Bounded timing samples for the collection/control path (milliseconds)."""

import threading
from collections import defaultdict, deque

import numpy as np


class Timings:
    def __init__(self):
        self.lock = threading.Lock()
        self.samples = defaultdict(lambda: deque(maxlen=2048))

    def add(self, name, elapsed_ns):
        with self.lock:
            self.samples[name].append(elapsed_ns / 1e6)

    def snapshot(self):
        with self.lock:
            samples = {key: list(values) for key, values in self.samples.items()}
            self.samples.clear()
        return {
            key: dict(zip(("p50", "p95", "p99", "max"), np.percentile(values, [50, 95, 99, 100]).round(3).tolist()), n=len(values))
            for key, values in samples.items() if values
        }


def feedback_ages(feedback, now_ns):
    return {
        key: (now_ns - int(feedback[key + "_available_host_ts_ns"])) / 1e6
        for key in ("joint", "tcp", "gripper")
        if key + "_available_host_ts_ns" in feedback
    }


def require_feedback_fresh(feedback, now_ns, limit_ms):
    ages = feedback_ages(feedback or {}, now_ns)
    invalid = [f"{key}={ages[key]:.1f} ms" if key in ages else f"{key}=无数据"
               for key in ("joint", "tcp", "gripper")
               if key not in ages or not 0 <= ages[key] <= limit_ms]
    if invalid:
        raise ValueError(f"robot_feedback 数据缺失/过期: {', '.join(invalid)}；要求 0–{limit_ms:g} ms")
