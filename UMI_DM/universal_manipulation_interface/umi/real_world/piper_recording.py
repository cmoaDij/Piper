"""Optional asynchronous recording for Piper deployment diagnostics."""
import json
import queue
import threading
from pathlib import Path

import cv2
import numpy as np


class AsyncFrameRecorder:
    """Write camera frames and host timestamps away from the policy thread."""

    def __init__(self, output_dir, *, width=1280, height=960, fps=60.0,
                 queue_size=240):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.video_path = self.output_dir / 'gopro_raw.avi'
        self.timestamp_path = self.output_dir / 'gopro_timestamps.jsonl'
        self.width = int(width)
        self.height = int(height)
        self.fps = float(fps)
        self.queue = queue.Queue(maxsize=queue_size)
        self.dropped = 0
        self.error = None
        self.closed = False
        self.thread = threading.Thread(target=self._run, name='piper-frame-record', daemon=True)
        self.thread.start()

    def write(self, timestamp, frame):
        if self.closed:
            return False
        try:
            self.queue.put_nowait((float(timestamp), np.asarray(frame).copy()))
            return True
        except queue.Full:
            self.dropped += 1
            return False

    def _run(self):
        writer = None
        timestamps = None
        frame_index = 0
        try:
            writer = cv2.VideoWriter(
                str(self.video_path),
                cv2.VideoWriter_fourcc(*'MJPG'),
                self.fps,
                (self.width, self.height),
            )
            if not writer.isOpened():
                raise RuntimeError(f'Cannot open raw camera recording: {self.video_path}')
            timestamps = self.timestamp_path.open('w', buffering=1, encoding='utf-8')
            while True:
                item = self.queue.get()
                if item is None:
                    break
                timestamp, frame = item
                if frame.shape[:2] != (self.height, self.width):
                    raise ValueError(f'Unexpected camera frame shape: {frame.shape}')
                writer.write(frame)
                timestamps.write(json.dumps({
                    'frame_index': frame_index,
                    'host_recv_monotonic_s': timestamp,
                }, allow_nan=False) + '\n')
                frame_index += 1
        except Exception as exc:
            self.error = str(exc)
        finally:
            if writer is not None:
                writer.release()
            if timestamps is not None:
                timestamps.close()
            metadata = {
                'video': str(self.video_path.name),
                'timestamps': str(self.timestamp_path.name),
                'width': self.width,
                'height': self.height,
                'fps': self.fps,
                'frame_count': frame_index,
                'dropped_before_encode': self.dropped,
                'error': self.error,
            }
            (self.output_dir / 'gopro_recording.json').write_text(
                json.dumps(metadata, indent=2) + '\n')

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.queue.put(None)
        self.thread.join(timeout=15)


class AsyncModelInputRecorder:
    """Persist exact policy tensors and replay metadata asynchronously."""

    def __init__(self, output_dir, *, queue_size=128):
        self.output_dir = Path(output_dir)
        self.input_dir = self.output_dir / 'model_inputs'
        self.input_dir.mkdir(parents=True, exist_ok=True)
        self.manifest_path = self.output_dir / 'model_input_manifest.jsonl'
        self.queue = queue.Queue(maxsize=queue_size)
        self.dropped = 0
        self.error = None
        self.closed = False
        self.thread = threading.Thread(target=self._run, name='piper-model-input-record', daemon=True)
        self.thread.start()

    def write(self, prediction_id, obs, metadata):
        if self.closed:
            return False
        arrays = {}
        for key, value in obs.items():
            if hasattr(value, 'detach'):
                value = value.detach().cpu().numpy()
            arrays[key] = np.asarray(value).copy()
        try:
            self.queue.put_nowait((int(prediction_id), arrays, dict(metadata)))
            return True
        except queue.Full:
            self.dropped += 1
            return False

    def _run(self):
        manifest = None
        count = 0
        try:
            manifest = self.manifest_path.open('w', buffering=1, encoding='utf-8')
            while True:
                item = self.queue.get()
                if item is None:
                    break
                prediction_id, arrays, metadata = item
                filename = f'input_{prediction_id:06d}.npz'
                np.savez_compressed(self.input_dir / filename, **arrays)
                manifest.write(json.dumps({
                    'prediction_id': prediction_id,
                    'file': str(Path('model_inputs') / filename),
                    **metadata,
                }, allow_nan=False) + '\n')
                count += 1
        except Exception as exc:
            self.error = str(exc)
        finally:
            if manifest is not None:
                manifest.close()
            (self.output_dir / 'model_input_recording.json').write_text(
                json.dumps({
                    'recorded_inputs': count,
                    'dropped_before_write': self.dropped,
                    'error': self.error,
                }, indent=2) + '\n')

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.queue.put(None)
        self.thread.join(timeout=30)
