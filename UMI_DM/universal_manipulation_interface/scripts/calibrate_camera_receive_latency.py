#!/usr/bin/env python3
"""Measure GoPro/HDMI/capture-card receive latency with a host QR clock.

Point the real GoPro at the QR window on the same computer and connect its HDMI
output to the capture card used by deployment.  The timestamp encoded in each
QR and the frame receive timestamp both use ``time.monotonic()``.  This script
does not reset USB devices and uses the deployment 1280x960 MJPEG settings.
"""
import argparse
from collections import OrderedDict
import json
from pathlib import Path
import threading
import time

import cv2
import numpy as np


def make_qr(text, size):
    image = cv2.QRCodeEncoder_create().encode(text)
    image = cv2.copyMakeBorder(image, 6, 6, 6, 6, cv2.BORDER_CONSTANT, value=255)
    image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    return cv2.resize(image, (size, size), interpolation=cv2.INTER_NEAREST)


class CameraReader:
    """Continuously read with the same dequeue pattern as deployment."""
    def __init__(self, device):
        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(f'Cannot open camera: {device}')
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)
        self.cap.set(cv2.CAP_PROP_FPS, 60)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.frames = []
        self.sequence = 0
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.error = None
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.thread.start()

    def _run(self):
        try:
            while not self.stop_event.is_set():
                ok, frame = self.cap.read()
                received = time.monotonic()
                if not ok:
                    raise RuntimeError('Camera failed to deliver a frame')
                with self.lock:
                    self.sequence += 1
                    self.frames.append((self.sequence, received, frame))
                    del self.frames[:-180]
        except Exception as exc:
            self.error = exc

    def recent(self):
        with self.lock:
            return list(self.frames)

    def latest(self):
        with self.lock:
            return self.frames[-1] if self.frames else None

    def close(self):
        self.stop_event.set()
        self.thread.join(timeout=2)
        self.cap.release()


class QRScanner:
    """Decode only the newest frame so QR work cannot block camera capture."""
    def __init__(self, camera, received):
        self.camera = camera
        self.received = received
        self.detector = cv2.QRCodeDetector()
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.last_sequence = 0

    def start(self):
        self.thread.start()

    def _run(self):
        while not self.stop_event.is_set():
            latest = self.camera.latest()
            if latest is None or latest[0] == self.last_sequence:
                time.sleep(.005)
                continue
            sequence, t_receive, frame = latest
            self.last_sequence = sequence
            decoded, _, _ = self.detector.detectAndDecode(frame)
            try:
                t_qr = float(decoded)
            except (TypeError, ValueError):
                continue
            if t_receive < t_qr:
                continue
            with self.lock:
                if t_qr not in self.received:
                    self.received[t_qr] = {
                        'qr_monotonic_s': t_qr,
                        'receive_monotonic_s': t_receive,
                        'latency_ms': (t_receive - t_qr) * 1000.0,
                    }

    def close(self):
        self.stop_event.set()
        self.thread.join(timeout=2)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--camera', default='/dev/video0')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--samples', type=int, default=60)
    parser.add_argument('--qr-size', type=int, default=720)
    parser.add_argument('--hold-s', type=float, default=.20,
                        help='Keep each QR visible long enough for several camera frames')
    parser.add_argument('--warmup-s', type=float, default=2.0)
    parser.add_argument('--window-title', default='UMI camera latency QR')
    args = parser.parse_args()
    if args.samples < 5 or args.qr_size < 160 or args.hold_s <= 0 or args.warmup_s < 0:
        parser.error('Invalid sample, QR size, hold, or warmup setting')
    args.output.parent.mkdir(parents=True, exist_ok=True)

    camera = CameraReader(args.camera)
    received = OrderedDict()
    scanner = QRScanner(camera, received)
    started = time.monotonic()
    next_code = started + args.warmup_s
    displayed = None
    qr = np.zeros((args.qr_size, args.qr_size, 3), dtype=np.uint8)
    reported_count = 0
    try:
        camera.start()
        scanner.start()
        while len(received) < args.samples:
            if camera.error:
                raise camera.error
            now = time.monotonic()
            if displayed is None and now < next_code:
                pass
            elif displayed is None or now >= next_code:
                displayed = time.monotonic()
                code_text = f'{displayed:.9f}'
                qr = make_qr(code_text, args.qr_size)
                next_code = displayed + args.hold_s
            cv2.imshow(args.window_title, qr)
            key = cv2.waitKey(20) & 0xff
            latest_count = len(received)
            if latest_count > reported_count:
                last = list(received.values())[-1]
                print(f"sample={latest_count}/{args.samples} latency_ms={last['latency_ms']:.2f}", flush=True)
                reported_count = latest_count
            if key in (ord('q'), 27):
                break
    finally:
        scanner.close()
        camera.close()
        cv2.destroyAllWindows()

    rows = list(received.values())
    values = np.asarray([row['latency_ms'] for row in rows], dtype=float)
    if len(values) < 5:
        raise RuntimeError(f'Only {len(values)} QR samples decoded; need at least 5')
    result = {
        'camera': args.camera, 'resolution': [1280, 960], 'capture_fps': 60,
        'sample_count': int(len(values)), 'requested_samples': args.samples,
        'detection_rate': float(len(values) / max(1, args.samples)),
        'latency_ms': {'median': float(np.percentile(values, 50)),
                       'p95': float(np.percentile(values, 95)),
                       'max': float(np.max(values)), 'min': float(np.min(values)),
                       'std': float(np.std(values))},
        'nominal_latency_ms': float(np.percentile(values, 50)),
        'jitter_p95_above_nominal_ms': float(np.percentile(values, 95)
                                             - np.percentile(values, 50)),
        'timestamp_note': 'QR generation and frame receive use the same host monotonic clock.',
        'display_note': 'Measured value includes display refresh, GoPro, HDMI, capture, and dequeue delay.',
        'samples': rows,
    }
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2), flush=True)


if __name__ == '__main__':
    main()
