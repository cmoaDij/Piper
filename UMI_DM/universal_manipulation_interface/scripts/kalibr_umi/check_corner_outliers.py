#!/usr/bin/env python3
"""Screen ChArUco points with per-frame robust PnP; preserve all input frames.

This is a diagnostic, not a ground-truth classifier. It never edits the input
observations or video; the output contains the same timestamps and frames.
"""
import argparse
import csv
import json
from pathlib import Path

import cv2
import numpy as np


def project(objects, rotation, translation, camera, distortion):
    points, _ = cv2.fisheye.projectPoints(
        objects.reshape(1, -1, 3), rotation, translation, camera, distortion)
    return points.reshape(-1, 2)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--observations', required=True, type=Path)
    parser.add_argument('--intrinsics', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--threshold-px', type=float, default=3.0)
    args = parser.parse_args()
    if args.threshold_px <= 0:
        parser.error('threshold must be positive')
    args.output.mkdir(parents=True, exist_ok=True)
    data = json.loads(args.observations.read_text())
    intrinsics = json.loads(args.intrinsics.read_text())['intrinsics']
    f = intrinsics['focal_length']
    camera = np.array([[f, 0, intrinsics['principal_pt_x']],
                       [0, f * intrinsics['aspect_ratio'], intrinsics['principal_pt_y']],
                       [0, 0, 1.]], dtype=np.float64)
    distortion = np.array([intrinsics['radial_distortion_{}'.format(i)]
                           for i in range(1, 5)], dtype=np.float64)
    cv2.setRNGSeed(42)
    result = dict(data)
    result['frames'] = []
    rows = []
    failures = []
    for index, frame in enumerate(data['frames']):
        ids = sorted(frame['image_points'], key=int)
        objects = np.asarray([data['scene_pts'][k] for k in ids], np.float64)
        image = np.asarray([frame['image_points'][k] for k in ids], np.float64)
        keep = np.ones(len(ids), dtype=bool)
        residuals = np.full(len(ids), np.nan)
        if len(ids) >= 8:
            undistorted = cv2.fisheye.undistortPoints(image.reshape(-1, 1, 2), camera, distortion, P=camera)
            ok, rotation, translation, inliers = cv2.solvePnPRansac(
                objects, undistorted, camera, np.zeros(4), iterationsCount=200,
                reprojectionError=args.threshold_px, confidence=0.999,
                flags=cv2.SOLVEPNP_EPNP)
            if ok and inliers is not None and len(inliers) >= 8:
                keep[:] = False
                keep[inliers.reshape(-1)] = True
                for _ in range(5):
                    ok, rotation, translation = cv2.solvePnP(
                        objects[keep], undistorted[keep], camera, np.zeros(4),
                        rotation, translation, True, flags=cv2.SOLVEPNP_ITERATIVE)
                    if not ok:
                        break
                    residuals = np.linalg.norm(project(objects, rotation, translation, camera, distortion) - image, axis=1)
                    updated = residuals <= args.threshold_px
                    if updated.sum() < 8 or np.array_equal(updated, keep):
                        break
                    keep = updated
                if not ok or keep.sum() < 8:
                    keep[:] = True
                    failures.append(index)
            else:
                failures.append(index)
        else:
            failures.append(index)
        output_frame = dict(frame)
        output_frame['image_points'] = {k: frame['image_points'][k] for k, accepted in zip(ids, keep) if accepted}
        result['frames'].append(output_frame)
        for key, point, error, accepted in zip(ids, image, residuals, keep):
            rows.append([index, frame['timestamp_ns'], key, float(point[0]), float(point[1]),
                         float(error), int(accepted)])
        if (index + 1) % 500 == 0:
            print('Screened {}/{} frames'.format(index + 1, len(data['frames'])), flush=True)
    with (args.output / 'corner_residuals.csv').open('w', newline='') as out:
        writer = csv.writer(out)
        writer.writerow(['frame', 'timestamp_ns', 'corner_id', 'x_px', 'y_px', 'pnp_residual_px', 'kept'])
        writer.writerows(rows)
    (args.output / 'filtered_observations.json').write_text(json.dumps(result, separators=(',', ':')))
    rejected = [row for row in rows if not row[-1]]
    summary = {'threshold_px': args.threshold_px, 'frames': len(data['frames']),
               'frames_unchanged': len(result['frames']) == len(data['frames']),
               'corners_total': len(rows), 'corners_rejected': len(rejected),
               'frames_with_rejections': len({r[0] for r in rejected}),
               'pnp_failure_frames': failures,
               'max_rejected_residual_px': max((r[-2] for r in rejected), default=None)}
    (args.output / 'summary.json').write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
