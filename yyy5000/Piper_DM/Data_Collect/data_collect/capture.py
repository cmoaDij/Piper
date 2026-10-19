"""Image acquisition and HDF5 writing in a spawned process, without CAN/ROS."""

import multiprocessing as mp
import os
import queue
import signal
import threading
import time
from collections import deque
from pathlib import Path

from .adapters.cameras import realsense_worker
from .adapters.tactile import tactile_worker
from .config import IMAGE_STREAMS, local_path
from .kinematics import ForwardKinematics
from .images import select_encoder
from .storage import EpisodeManager, EpisodeWriter


class ImageBuffer(EpisodeManager):
    def __init__(self, cfg, fk):
        super().__init__(cfg, fk)
        self.history = {name: deque(maxlen=16) for name in IMAGE_STREAMS}

    def emit(self, group, record):
        with self.lock:
            self.history[group].append(record)
            super().emit(group, record)

    def open_writer(self, path, metadata, max_items, max_bytes):
        with self.lock:
            if self.current:
                raise RuntimeError("采集进程已有正在录制的 Episode")
            writer = EpisodeWriter(path, metadata, max_items, max_bytes, dict(self.baselines), self.fk)
            self.current = writer
            start = metadata["start_host_ts_ns"]
            for stream, records in self.history.items():
                past = [r for r in records if int(r["available_host_ts_ns"]) <= start]
                if not past:
                    self.fail(f"{stream} 缺少录制开始前图像，IPC 延迟超过缓存范围")
                    break
                seed = [past[-1]] + [r for r in records if int(r["available_host_ts_ns"]) > start]
                for record in seed:
                    if not writer.submit(stream, record):
                        self.fail(writer.error or "writer_not_accepting_samples")
                        break
            return writer


def _capture_main(cfg, incoming, outgoing, stop):
    # The CAN owner coordinates finishing files after launch/terminal SIGINT.
    # SIGTERM remains available for escalation if that owner cannot shut down.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    # Spawn keeps these native SDKs and their worker/GIL activity out of control.
    import pyrealsense2  # noqa: F401
    import onnxruntime  # noqa: F401
    from dmrobotics import Sensor  # noqa: F401

    # Probe the actual two-camera workload before devices become ready for A.
    select_encoder(cfg)
    fk = ForwardKinematics(local_path(cfg["urdf"]), cfg["base_link"], cfg["tip_link"], cfg["tcp_offset"])
    manager = ImageBuffer(cfg, fk)
    threads, writers = [], {}

    def start_worker(fn, *args):
        def guarded():
            try:
                fn(*args)
            except Exception as exc:
                manager.fail(str(exc))
        thread = threading.Thread(target=guarded, daemon=True)
        threads.append(thread)
        thread.start()

    def status():
        with manager.lock:
            result = {
                "pid": os.getpid(), "stamp": time.monotonic_ns(), "error": manager.failure,
                "streams": {name: list(stamps) for name, stamps in manager.stream_times.items()},
                "baselines": list(manager.baselines),
                "domains": {name: opts.get("device_timestamp_domain") for name, opts in cfg["cameras"].items()},
                "writers": {},
            }
            for name, writer in writers.items():
                with writer.lock:
                    result["writers"][name] = {"pending_bytes": writer.pending_bytes, "error": writer.error, "rejection": writer.rejection, "done": not writer.thread.is_alive()}
        # Keep the newest state without ever blocking sensor/control work.
        try:
            outgoing.put_nowait(result)
        except queue.Full:
            pass

    try:
        status()
        for name, opts in cfg["cameras"].items():
            start_worker(realsense_worker, name, opts, manager, stop, cfg["camera_timeout_ms"])
        for name, opts in cfg["tactile"].items():
            start_worker(tactile_worker, name, opts, manager, stop)
        next_status = time.monotonic()
        while not stop.is_set():
            try:
                event = incoming.get(timeout=0.005)
            except queue.Empty:
                event = None
            if event:
                kind, *args = event
                if kind == "open":
                    path, metadata, max_items, max_bytes = args
                    writers[path] = manager.open_writer(path, metadata, max_items, max_bytes)
                elif kind == "record":
                    path, group, record = args
                    writer = writers[path]
                    if not writer.submit(group, record):
                        manager.fail(writer.error or "writer_not_accepting_samples")
                elif kind == "finish":
                    path, end_ns, error, rejected = args
                    with manager.lock:
                        writer = writers.get(path)
                        # A failure acknowledgement may cross a late finish
                        # request. Completing/reaping an episode is idempotent.
                        if writer is not None:
                            if manager.current is writer:
                                manager.current = None
                            writer.finish(end_ns, error, rejected=rejected)
                elif kind == "shutdown":
                    break
                elif kind == "reap":
                    writer = writers.get(args[0])
                    if writer and not writer.thread.is_alive():
                        del writers[args[0]]
            if time.monotonic() >= next_status:
                status()
                next_status = time.monotonic() + 0.02
            for writer in list(writers.values()):
                if writer.error and not manager.failure:
                    manager.fail(writer.error)
    except BaseException as exc:
        manager.fail(f"采集进程异常: {type(exc).__name__}: {exc}")
        status()
        raise
    finally:
        stop.set()
        for thread in threads:
            thread.join(timeout=3)
        if any(thread.is_alive() for thread in threads):
            manager.fail("图像 SDK worker 无法停止")
        try:
            manager.close()
        finally:
            for writer in writers.values():
                writer.finish(writer.end_ns or time.monotonic_ns(), manager.failure or writer.error)
                writer.join()
            status()


class CaptureWriter:
    """EpisodeWriter interface; the actual file and image arrays stay in child."""
    def __init__(self, runtime, path, metadata, max_items, max_bytes, baselines=None, fk=None):
        self.runtime, self.path, self.metadata = runtime, Path(path), metadata
        self.max_bytes, self.pending_bytes, self.error = max_bytes, 0, None
        self.rejection = None
        self.lock, self.done = threading.Lock(), threading.Event()
        self.end_ns = None
        self.thread = self
        runtime.writers[str(self.path)] = self
        if not runtime.send(("open", str(self.path), metadata, max_items, max_bytes)):
            self.error = "采集 IPC 队列已满/进程不可用"
            self.done.set()

    def submit(self, group, record):
        if self.error or self.done.is_set():
            return False
        if group in IMAGE_STREAMS:
            return True
        return self.runtime.send(("record", str(self.path), group, record))

    def finish(self, end_ns, error=None, *, rejected=False):
        if self.end_ns is None:
            self.end_ns = int(end_ns)
        if error:
            if rejected:
                self.rejection = error
            else:
                self.error = error
        is_rejection = bool(self.rejection) and not self.error
        if not self.runtime.send(("finish", str(self.path), self.end_ns, self.error or self.rejection, is_rejection)):
            self.error = self.error or "采集 IPC 结束请求失败，文件可能为 partial"
            self.done.set()

    def is_alive(self):
        return not self.done.is_set()

    def join(self):
        if not self.done.wait(30):
            self.error = "封盘超过 30 秒，保留 partial 文件"
            self.finish(self.end_ns or time.monotonic_ns(), self.error)
            self.runtime.manager.fail(self.error)
        return self.path


class CaptureRuntime:
    def __init__(self, cfg, manager):
        self.cfg, self.manager = cfg, manager
        context = mp.get_context("spawn")
        self.incoming = context.Queue(maxsize=cfg["queue_max_items"])
        self.outgoing = context.Queue(maxsize=4)
        self.stop_event = context.Event()
        self.process = context.Process(target=_capture_main, args=(cfg, self.incoming, self.outgoing, self.stop_event), name="piper-image-capture")
        self.writers = {}
        self.closing = False
        self.last_status = None
        self.ready = False
        self.started = time.monotonic()
        self.monitor = threading.Thread(target=self._monitor, name="capture-status", daemon=True)
        manager.writer_factory = lambda *args: CaptureWriter(self, *args)

    def start(self):
        self.process.start()
        self.monitor.start()

    def send(self, event):
        if not self.process.is_alive() or self.stop_event.is_set():
            return False
        try:
            self.incoming.put_nowait(event)
            return True
        except queue.Full:
            return False

    def _monitor(self):
        while True:
            try:
                data = self.outgoing.get(timeout=0.02)
            except queue.Empty:
                data = None
            if data:
                self.last_status = data["stamp"] / 1e9
                with self.manager.lock:
                    for name, stamps in data["streams"].items():
                        self.manager.stream_times[name] = deque(stamps, maxlen=256)
                        if stamps:
                            self.manager.latest[name] = {"available_host_ts_ns": stamps[-1]}
                    self.manager.baselines = dict.fromkeys(data["baselines"])
                    self.ready = all(data["streams"].get(name) for name in IMAGE_STREAMS) and len(data["baselines"]) == 2
                    for name, domain in data["domains"].items():
                        if domain:
                            self.cfg["cameras"][name]["device_timestamp_domain"] = domain
                    for name, stats in data["writers"].items():
                        writer = self.writers.get(name)
                        if writer:
                            with writer.lock:
                                writer.pending_bytes = stats["pending_bytes"]
                                writer.error = stats["error"] or writer.error
                                writer.rejection = stats.get("rejection") or writer.rejection
                                if stats["done"]:
                                    writer.done.set()
                                    self.send(("reap", name))
                    if data["error"] and not self.manager.failure:
                        self.manager.fail(data["error"])
            if not self.process.is_alive():
                if not self.closing:
                    self.manager.fail(f"采集进程退出: exitcode={self.process.exitcode}")
                for writer in list(self.writers.values()):
                    if not writer.done.is_set():
                        writer.error = writer.error or "采集进程退出，文件可能为 partial"
                        writer.done.set()
                return
            limit = 0.25 if self.ready else 15.0
            origin = (self.last_status or self.started) if self.ready else self.started
            if not self.closing and time.monotonic() - origin > limit and not self.manager.failure:
                self.manager.fail(f"采集进程状态超过 {limit:g} 秒未更新，停止录制和运动")

    def close(self):
        try:
            self.manager.close()
        finally:
            self.closing = True
            self.send(("shutdown",))
            self.process.join(timeout=12)
            if self.process.is_alive():
                self.manager.fail("采集进程退出超时，保留 partial 文件")
                self.stop_event.set()
                self.process.terminate()
                self.process.join(timeout=3)
            self.monitor.join(timeout=2)
            self.incoming.cancel_join_thread()
            self.outgoing.cancel_join_thread()
            self.incoming.close()
            self.outgoing.close()
