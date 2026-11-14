"""Regression checks for Charuco ID mapping and timestamp validation."""
import importlib.util
import json
import os
import sys
import tempfile
import types
from pathlib import Path
from unittest.mock import patch

import numpy as np


class Observation:
    def __init__(self, target):
        self.points = {}
    def setImage(self, image): pass
    def setTime(self, stamp): pass
    def updateImagePoint(self, idx, point): self.points[idx] = point
    def set_T_t_c(self, transform): pass
    def clearImage(self): pass


class Target:
    def rows(self): return 9
    def cols(self): return 7
    def point(self, idx): return np.array([idx // 7 * .05, idx % 7 * .05, 0.])


class Detector:
    def target(self): return Target()
    def geometry(self): return self
    def estimateTransformation(self, obs): return True, None


class Stamp:
    def __init__(self, ns): self.ns = ns
    def toNSec(self): return self.ns


class Dataset:
    perform_synchronization = False
    def numImages(self): return 10
    def readDataset(self):
        for i in range(10): yield Stamp(i + 1), np.zeros((4, 5), dtype=np.uint8)


def test():
    path = Path(__file__).resolve().parents[1] / 'scripts/kalibr_umi/charuco_adapter.py'
    modules = {'aslam_cv': types.SimpleNamespace(GridCalibrationTargetObservation=Observation),
               'kalibr_common': types.SimpleNamespace()}
    with patch.dict(sys.modules, modules):
        spec = importlib.util.spec_from_file_location('charuco_adapter_test', path)
        adapter = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(adapter)
    pts = {str(i): [(i % 9 + 1)*.05, (i // 9 + 1)*.05, 0] for i in range(63)}
    frames = [{'timestamp_ns': i+1, 'image_points': {str(j): [j, j+1] for j in range(63)}} for i in range(10)]
    with tempfile.TemporaryDirectory() as tmp:
        p = Path(tmp) / 'obs.json'; p.write_text(json.dumps({'frames': frames, 'scene_pts': pts, 'height': 4, 'width': 5}))
        with patch.dict(os.environ, {'UMI_CHARUCO_OBSERVATIONS': str(p)}):
            out = adapter.extract_charuco(Dataset(), Detector())
            assert len(out) == 10 and len(out[0].points) == 63
            assert np.allclose(out[0].points[7], [1, 2])  # Charuco id 1 -> Kalibr id 7
            frames[4]['timestamp_ns'] = 100
            p.write_text(json.dumps({'frames': frames, 'scene_pts': pts, 'height': 4, 'width': 5}))
            try: adapter.extract_charuco(Dataset(), Detector())
            except RuntimeError as exc: assert 'timestamp' in str(exc)
            else: raise AssertionError('Timestamp mismatch must fail')
    print('Charuco mapping and timestamp checks passed')

if __name__ == '__main__': test()
