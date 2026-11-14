#!/usr/bin/env python3
"""Display training first-frame overlays with live GoPro; no policy, CAN or SDK."""
import argparse
import json
from pathlib import Path
import signal
import sys
import time

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from umi.real_world.piper_camera_geometry import preprocess
from umi.real_world.piper_camera_preview import CameraPreview, CameraReader
from umi.real_world.piper_training_reference import load_training_reference
from umi.real_world.piper_recording import AsyncFrameRecorder


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--setup', type=Path, required=True)
    parser.add_argument('--mask', type=Path, required=True)
    parser.add_argument('--match-dataset', type=Path, required=True)
    parser.add_argument('--match-episode', type=int, default=0)
    parser.add_argument('--match-alpha', type=float, default=.5)
    parser.add_argument('--preview-port', type=int, default=8768)
    parser.add_argument('--camera', default='/dev/video0')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=0.)
    parser.add_argument('--record-dir', type=Path)
    args = parser.parse_args()
    if not np.isfinite(args.duration) or args.duration < 0:
        parser.error('--duration must be finite and nonnegative')
    if not 1 <= args.preview_port <= 65535:
        parser.error('--preview-port must be between 1 and 65535')
    setup = yaml.safe_load(args.setup.read_text())
    mask = json.loads(args.mask.read_text())
    geometry = setup.get('camera_image_geometry', 'native_4_3')
    reference = load_training_reference(args.match_dataset)
    reference.save(args.output, args.match_episode, args.match_alpha)

    def interrupted(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    preview = camera = recorder = None
    try:
        preview = CameraPreview(args.preview_port, training_reference=reference,
                                match_episode=args.match_episode, match_alpha=args.match_alpha)
        preview.start()
        preview.update_status({'mode': 'camera_alignment_only',
                               'message': '手动调整机械臂和物体；在页面切换示教片段、调整透明度。',
                               'policy_loaded': False, 'robot_connected': False})
        if args.record_dir is not None:
            recorder = AsyncFrameRecorder(args.record_dir / 'camera')
        camera = CameraReader(args.camera, recorder=recorder)
        camera.start()
        print(f'Camera alignment only: http://127.0.0.1:{preview.port}/', flush=True)
        print('No model, CAN or robot SDK is opened. Ctrl+C exits.', flush=True)
        deadline = time.monotonic() + args.duration if args.duration else float('inf')
        last_frame = 0.
        while time.monotonic() < deadline:
            if camera.error:
                raise camera.error
            frames = camera.recent()
            if not frames or frames[-1][0] - last_frame < .1:
                time.sleep(.01)
                continue
            last_frame, raw = frames[-1]
            model_rgb, _ = preprocess(raw, mask, camera_image_geometry=geometry)
            preview.update(raw, model_rgb)
    except KeyboardInterrupt:
        print('Camera alignment stopped.', flush=True)
    finally:
        if camera is not None:
            camera.close()
        if recorder is not None:
            recorder.close()
        if preview is not None:
            preview.close()


if __name__ == '__main__':
    main()
