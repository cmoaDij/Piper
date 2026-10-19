"""Bounded producer queues, batched HDF5 and RGB video per episode."""

import json
import queue
import threading
import time
from collections import deque
from datetime import datetime, timezone
from pathlib import Path

import h5py
import numpy as np

from .config import IMAGE_STREAMS
from .diagnostics import require_feedback_fresh
from .images import RGBVideoWriter, select_encoder


class EpisodeWriter:
    def __init__(
        self, path, metadata, max_items=2048, max_bytes=256 * 1024**2, baselines=None, fk=None
    ):
        self.path = Path(path)
        self.partial = self.path.with_suffix(".h5.partial")
        self.metadata = metadata
        self.settings = metadata.get("config", {}).get("storage", {})
        self.baselines = baselines or {}
        self.fk = fk
        self.queue = queue.Queue(maxsize=max_items)
        self.max_bytes = max_bytes
        self.pending_bytes = 0
        self.lock = threading.Lock()
        self.done = threading.Event()
        self.error = None
        self.rejection = None
        self.dropped = 0
        self.end_ns = None
        self.thread = threading.Thread(target=self._run, name=self.path.stem + "-writer")
        self.thread.start()

    def submit(self, group, record):
        size = sum(np.asarray(v).nbytes for v in record.values())
        with self.lock:
            if self.done.is_set() or self.error:
                return False
            if self.pending_bytes + size > self.max_bytes or self.queue.full():
                self.dropped += 1
                self.error = (
                    f"writer_queue_overflow: pending_bytes={self.pending_bytes}, "
                    f"incoming_bytes={size}, limit_bytes={self.max_bytes}, "
                    f"queued_items={self.queue.qsize()}/{self.queue.maxsize}"
                )
                return False
            # Producers pass owned arrays; no camera memory is shared with the writer.
            self.pending_bytes += size
            self.queue.put_nowait((group, record, size))
        return True

    def finish(self, end_ns, error=None, *, rejected=False):
        with self.lock:
            self.end_ns = int(end_ns)
            if error:
                if rejected:
                    self.rejection = error
                else:
                    self.error = error
            self.done.set()

    def join(self):
        self.thread.join()
        return self.path

    @staticmethod
    def _append(group, records, compression=None):
        keys = records[0].keys()
        if any(record.keys() != keys for record in records):
            raise ValueError(f"{group.name} 批次字段不一致")
        for key in keys:
            value = np.stack([record[key] for record in records])
            if value.dtype.kind == "f" and not np.isfinite(value).all():
                raise ValueError(f"nonfinite {group.name}/{key}")
            if key not in group:
                group.create_dataset(
                    key,
                    shape=(0,) + value.shape[1:],
                    maxshape=(None,) + value.shape[1:],
                    chunks=(1,) + value.shape[1:] if key == "data" else True,
                    dtype=value.dtype,
                    compression=compression if key == "data" else None,
                )
            ds = group[key]
            n = len(ds)
            ds.resize(n + len(value), axis=0)
            ds[n:] = value

    def _batch(self):
        try:
            first = self.queue.get(timeout=0.05)
        except queue.Empty:
            return []
        batch = [first]
        deadline = time.monotonic() + self.settings.get("batch_ms", 50) / 1000
        while len(batch) < self.settings.get("batch_max_items", 128):
            try:
                batch.append(self.queue.get_nowait())
            except queue.Empty:
                remaining = deadline - time.monotonic()
                if self.done.is_set() or remaining <= 0:
                    break
                try:
                    batch.append(self.queue.get(timeout=min(remaining, 0.005)))
                except queue.Empty:
                    continue
        return batch

    def _run(self):
        videos = {}
        try:
            config = self.metadata.get("config", {})
            encoder = select_encoder(config)
            with h5py.File(self.partial, "x") as f:
                for key, value in self.metadata.items():
                    f.attrs[key] = (
                        json.dumps(value, ensure_ascii=False)
                        if isinstance(value, (dict, list))
                        else value
                    )
                f.attrs["schema_version"] = "piper_raw_v2"
                f.attrs["complete"] = False
                for stream in IMAGE_STREAMS:
                    f.require_group(stream).attrs["color_order"] = "RGB"
                for name in ("robot_feedback", "robot_command"):
                    f.require_group(name)
                for stream, baseline in self.baselines.items():
                    f[stream].create_dataset("baseline", data=baseline, compression="lzf")
                    f[stream].attrs["color_order"] = "GRAY" if baseline.ndim == 2 else "RGB"
                    f[stream].attrs["baseline_ref"] = f"/{stream}/baseline"
                while not self.done.is_set() or not self.queue.empty():
                    batch = self._batch()
                    if not batch:
                        continue
                    try:
                        grouped = {}
                        for group, original, _ in batch:
                            record = dict(original)
                            if group == "robot_command" and "target_tcp_pose" not in record:
                                record["target_tcp_pose"] = self.fk(record["target_joint_pos"])
                            if group.startswith("cameras/"):
                                image = record.pop("data")
                                if group not in videos:
                                    name = group.split("/")[-1]
                                    fps = config["cameras"][name]["fps"]
                                    video = RGBVideoWriter(self.path.with_name(f"{self.path.stem}.{name}.mp4"), image.shape, fps, encoder, self.settings)
                                    videos[group] = video
                                    f[group].attrs.update(
                                        storage="video", video_path=video.path.name,
                                        image_shape=image.shape, video_encoder=encoder,
                                        video_quality=self.settings.get("video_quality", 18),
                                        video_pixel_format="yuv420p", video_fps=fps,
                                        video_time_base_num=1, video_time_base_den=fps,
                                        timestamp_source="available_host_ts_ns; video PTS is frame_index/fps",
                                    )
                                index = videos[group].append(image)
                                record["video_frame_index"] = np.int64(index)
                                record["video_pts"] = np.int64(index)
                            elif group.startswith("tactile/"):
                                f[group].attrs["color_order"] = "GRAY" if record["data"].ndim == 2 else "RGB"
                            grouped.setdefault(group, []).append(record)
                        for group, records in grouped.items():
                            self._append(f[group], records, "lzf" if group.startswith("tactile/") else None)
                    finally:
                        with self.lock:
                            self.pending_bytes -= sum(item[2] for item in batch)
                # Commit videos before HDF5; missing/truncated video can never be complete.
                for group, video in videos.items():
                    f[group].attrs["video_sha256"] = video.finish()
                f.attrs["end_host_ts_ns"] = self.end_ns or time.monotonic_ns()
                f.attrs["dropped_samples"] = self.dropped
                f.attrs["collection_error"] = self.error or self.rejection or ""
                f.attrs["failure_kind"] = "capture_fault" if self.error else "episode_rejected" if self.rejection else ""
                f.attrs["complete"] = not bool(self.error or self.rejection)
                f.flush()
            self.partial.replace(self.path)
        except Exception as exc:
            self.error = f"writer_failure: {exc}"
            # Keep .partial as evidence; it is never accepted by the audit.
            self.done.set()
        finally:
            for video in videos.values():
                video.close()


class EpisodeManager:
    def __init__(self, config, fk=None):
        self.cfg = config
        self.fk = fk
        self.raw = Path(config["data_root"]) / "raw"
        self.raw.mkdir(parents=True, exist_ok=True)
        self.lock = threading.RLock()
        self.current = None
        self.finished = []
        self.latest = {}
        self.stream_times = {name: deque(maxlen=256) for name in IMAGE_STREAMS}
        self.feedback_preroll = deque()
        self.baselines = {}
        self.failure = None
        self.writer_factory = EpisodeWriter

    def emit(self, group, record):
        with self.lock:
            if group in self.stream_times:
                stamps = self.stream_times[group]
                stamp = int(record["available_host_ts_ns"])
                stamps.append(stamp)
                while stamps and stamps[0] < stamp - 2_000_000_000:
                    stamps.popleft()
            if group == "robot_feedback":
                self.feedback_preroll.append(record)
                cutoff = int(record["available_host_ts_ns"]) - 250_000_000
                while (
                    self.feedback_preroll
                    and int(self.feedback_preroll[0]["available_host_ts_ns"]) < cutoff
                ):
                    self.feedback_preroll.popleft()
            if group != "robot_feedback" or bool(record["is_new_packet"]):
                self.latest[group] = record
            if self.current:
                if not self.current.submit(group, record):
                    self.fail(self.current.error or "writer_not_accepting_samples")

    def fail(self, message):
        with self.lock:
            self.failure = self.failure or str(message)
            if self.current:
                self.current.finish(time.monotonic_ns(), self.failure)
                self.finished.append(self.current)
                self.current = None

    def reject(self, message):
        """Reject only this trajectory; device/writer failures remain latched."""
        with self.lock:
            if self.current:
                self.current.finish(time.monotonic_ns(), str(message), rejected=True)
                self.finished.append(self.current)
                self.current = None

    def require_ready(self):
        """Reject motion/recording until every stream and tactile baseline is ready."""
        with self.lock:
            if self.failure:
                raise RuntimeError(self.failure)
            required = set(IMAGE_STREAMS) | {"robot_feedback"}
            if not required.issubset(self.latest):
                raise RuntimeError(f"传感器尚未就绪: {sorted(required - self.latest.keys())}")
            start = time.monotonic_ns()
            for stream in required:
                record = self.latest[stream]
                if stream == "robot_feedback":
                    require_feedback_fresh(record, start, self.cfg["audit"]["robot_red_ms"])
                    continue
                else:
                    oldest = int(record["available_host_ts_ns"])
                    limit = self.cfg["audit"][
                        "rgb_red_ms" if stream.startswith("cameras/") else "tactile_red_ms"
                    ]
                if not 0 <= start - oldest <= limit * 1e6:
                    raise RuntimeError(f"{stream} 数据已过期: age={(start - oldest) / 1e6:.1f} ms，要求 0–{limit:g} ms")
            missing = {s for s in IMAGE_STREAMS if s.startswith("tactile/")} - self.baselines.keys()
            if missing:
                raise RuntimeError(f"触觉无接触基线尚未就绪: {sorted(missing)}")

    def start(self):
        with self.lock:
            if self.current:
                return self.current.path
            self.require_ready()
            if sum(w.thread.is_alive() for w in self.finished) >= 2:
                raise RuntimeError("上一条仍在写盘，请稍后开始")
            start = time.monotonic_ns()
            meta = {
                key: self.cfg[key] for key in ("task_instruction", "operator", "calibration_id")
            }
            meta.update(
                record_date=datetime.now(timezone.utc).isoformat(),
                start_host_ts_ns=start,
                config=self.cfg,
                calibration=self.cfg.get("calibration", {}),
                simulated=False,
            )
            # Exclusive reservation prevents collisions across processes/restarts.
            for number in range(1, 1_000_000):
                path = self.raw / f"episode_{number:04d}.h5"
                if path.exists() or path.with_suffix(".h5.partial").exists() or any(self.raw.glob(f"{path.stem}.*.mp4*")):
                    continue
                reservation = path.with_suffix(".reserve")
                try:
                    with reservation.open("x"):
                        pass
                except FileExistsError:
                    continue
                meta["episode_id"] = path.stem
                self.current = self.writer_factory(
                    path,
                    meta,
                    self.cfg["queue_max_items"],
                    int(self.cfg["queue_max_mb"] * 1024**2),
                    self.baselines,
                    self.fk,
                )
                break
            else:
                raise RuntimeError("Episode 编号耗尽")
            writer = self.current
            if writer.error:
                self.fail(writer.error)
                raise RuntimeError(writer.error)
            for group, record in self.latest.items():
                if group != "robot_feedback":
                    if not writer.submit(group, record):
                        self.fail(writer.error or "writer_not_accepting_samples")
                        raise RuntimeError(self.failure)
            for record in self.feedback_preroll:
                if not writer.submit("robot_feedback", record):
                    self.fail(writer.error or "writer_not_accepting_samples")
                    raise RuntimeError(self.failure)
            return writer.path

    def stop(self):
        with self.lock:
            if self.current is None:
                return None
            writer = self.current
            self.current = None
            writer.finish(time.monotonic_ns())
            self.finished.append(writer)
            return writer.path

    def close(self):
        self.stop()
        for writer in self.finished:
            writer.join()
            if writer.path.exists():
                writer.path.with_suffix(".reserve").unlink(missing_ok=True)
        errors = [w.error for w in self.finished if w.error]
        if errors:
            raise RuntimeError("; ".join(errors))
