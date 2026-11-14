#!/usr/bin/env python3
"""Check that geometric outlier screening preserves frames and detects wrong IDs."""
import json
from pathlib import Path
import subprocess
import sys
import tempfile

import cv2
import numpy as np


def test():
    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)
        intrinsics = {'focal_length': 800., 'aspect_ratio': 1.,
                      'principal_pt_x': 1352., 'principal_pt_y': 1014.}
        intrinsics.update({'radial_distortion_{}'.format(i): 0. for i in range(1, 5)})
        camera = np.array([[800., 0, 1352.], [0, 800., 1014.], [0, 0, 1.]])
        scene = {str(i): [(i % 9 + 1) * .05, (i // 9 + 1) * .05, 0.] for i in range(63)}
        objects = np.asarray(list(scene.values()), np.float64).reshape(1, -1, 3)
        points, _ = cv2.fisheye.projectPoints(objects, np.array([.1, -.2, .05]),
                                               np.array([-.2, -.1, 1.]), camera, np.zeros(4))
        points = points.reshape(-1, 2)
        frames = []
        for i in range(2):
            pts = {str(j): p.tolist() for j, p in enumerate(points)}
            if i == 1:
                pts['0'] = (points[45] + [30., 20.]).tolist()
            frames.append({'timestamp_ns': 1000 + i, 'image_points': pts})
        source = {'width': 2704, 'height': 2028, 'scene_pts': scene, 'frames': frames}
        (root / 'observations.json').write_text(json.dumps(source))
        (root / 'intrinsics.json').write_text(json.dumps({'intrinsics': intrinsics}))
        script = Path(__file__).resolve().parents[1] / 'scripts/kalibr_umi/check_corner_outliers.py'
        subprocess.run([sys.executable, str(script), '--observations', str(root / 'observations.json'),
                        '--intrinsics', str(root / 'intrinsics.json'), '--output', str(root / 'result')],
                       check=True, capture_output=True, text=True)
        filtered = json.loads((root / 'result/filtered_observations.json').read_text())
        summary = json.loads((root / 'result/summary.json').read_text())
        assert [f['timestamp_ns'] for f in filtered['frames']] == [1000, 1001]
        assert len(filtered['frames'][0]['image_points']) == 63
        assert '0' not in filtered['frames'][1]['image_points']
        assert len(filtered['frames'][1]['image_points']) == 62
        assert summary['corners_rejected'] == 1
        assert not summary['pnp_failure_frames']
        assert json.loads((root / 'observations.json').read_text()) == source


if __name__ == '__main__':
    test()
    print('Corner outlier screening test passed')
