"""Robot-only timing records through the collection driver's SDK observer."""

import json
import time

import h5py

from data_collect.common.diagnostics import require_feedback_fresh
from data_collect.recording.storage import EpisodeManager, EpisodeWriter


class RobotLatencyWriter(EpisodeWriter):
    """Use the existing bounded writer without image devices or codec probes."""

    def _run(self):
        try:
            with h5py.File(self.partial, "x") as file:
                for key, value in self.metadata.items():
                    file.attrs[key] = (
                        json.dumps(value, ensure_ascii=False)
                        if isinstance(value, (dict, list)) else value
                    )
                file.attrs.update(
                    schema_version="piper_robot_latency_v1", complete=False,
                    image_streams="[]", not_for_training=True,
                    measurement_kind="SDK command to host-received feedback effective tracking lag",
                    absolute_robot_observation_latency_measured=False,
                )
                for name in ("robot_command", "robot_feedback"):
                    file.require_group(name)
                while not self.done.is_set() or not self.queue.empty():
                    batch = self._batch()
                    if not batch:
                        continue
                    try:
                        grouped = {}
                        for group, record, _ in batch:
                            if group not in ("robot_command", "robot_feedback"):
                                raise ValueError(f"Unexpected robot-latency stream: {group}")
                            grouped.setdefault(group, []).append(record)
                        for group, records in grouped.items():
                            self._append(file[group], records)
                    finally:
                        with self.lock:
                            self.pending_bytes -= sum(item[2] for item in batch)
                file.attrs["end_host_ts_ns"] = self.end_ns or time.monotonic_ns()
                if self.timeline is not None:
                    file.attrs["recording_intervals_host_ts_ns"] = json.dumps(self.timeline.intervals)
                file.attrs["dropped_samples"] = self.dropped
                file.attrs["collection_error"] = self.error or self.rejection or ""
                file.attrs["failure_kind"] = (
                    "capture_fault" if self.error else "episode_rejected" if self.rejection else ""
                )
                file.attrs["complete"] = not bool(self.error or self.rejection)
                file.flush()
            self.partial.replace(self.path)
        except Exception as exc:
            self.error = f"writer_failure: {exc}"
            self.done.set()


class RobotLatencyManager(EpisodeManager):
    """Require fresh robot feedback; camera/tactile state is irrelevant here."""

    def __init__(self, config, fk=None):
        super().__init__(config, fk)
        self.stream_times = {}
        self.writer_factory = RobotLatencyWriter

    def emit(self, group, record):
        if group not in ("robot_command", "robot_feedback"):
            raise ValueError(f"Unexpected robot-latency stream: {group}")
        return super().emit(group, record)

    def require_ready(self):
        with self.lock:
            if self.failure:
                raise RuntimeError(self.failure)
            feedback = self.latest.get("robot_feedback")
            if feedback is None:
                raise RuntimeError("机器人反馈尚未就绪")
            require_feedback_fresh(feedback, time.monotonic_ns(), self.cfg["audit"]["robot_red_ms"])
