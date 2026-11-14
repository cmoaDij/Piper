#!/usr/bin/env python3
"""Run Kalibr with full-resolution OpenICC ChArUco corners as observations.

Kalibr's checkerboard target supplies geometry only; no checkerboard detector
is used. Original ChArUco IDs and frame timestamps are verified explicitly.
"""
import json
import os
import runpy
import sys

import aslam_cv as acv
import kalibr_common as kc
import numpy as np


def extract_charuco(dataset, detector, multithreading=False, numProcesses=None,
                    clearImages=True, noTransformation=False):
    path = os.environ['UMI_CHARUCO_OBSERVATIONS']
    with open(path) as f:
        data = json.load(f)
    frames = data['frames']
    if dataset.perform_synchronization:
        raise RuntimeError('Timestamp rewriting is disabled for the GoPro dataset')
    if dataset.numImages() != len(frames):
        raise RuntimeError('Bag image count differs from ChArUco frame count; do not use --bag-freq or --bag-from-to')
    target = detector.target()
    if (target.rows(), target.cols()) != (9, 7):
        raise RuntimeError('Expected geometry target with 9 rows and 7 columns')
    # Kalibr grid coordinates are (row, col); ChArUco IDs are (row*9+col)
    # with source x=col+1 and y=row+1. The difference is a constant origin.
    for charuco_id in range(63):
        row, col = divmod(charuco_id, 9)
        kalibr_id = col * 7 + row
        expected = np.asarray(data['scene_pts'][str(charuco_id)], dtype=float)
        point = np.asarray(target.point(kalibr_id), dtype=float).reshape(3)
        if not np.allclose(point + [0.05, 0.05, 0.0], expected, atol=1e-6):
            raise RuntimeError('ChArUco and Kalibr geometry do not agree')
    accepted, insufficient, pose_failed = [], 0, 0
    for index, ((stamp, image), frame) in enumerate(zip(dataset.readDataset(), frames)):
        if abs(int(stamp.toNSec()) - int(frame['timestamp_ns'])) > 2:
            raise RuntimeError('Frame {} timestamp does not match observations'.format(index))
        if image.shape[:2] != (data['height'], data['width']):
            raise RuntimeError('Frame {} has unexpected image dimensions'.format(index))
        points = frame['image_points']
        if len(points) < 8:
            insufficient += 1
            continue
        obs = acv.GridCalibrationTargetObservation(target)
        obs.setImage(image)
        obs.setTime(stamp)
        for key, value in points.items():
            idx = int(key)
            if not 0 <= idx < 63:
                raise RuntimeError('Invalid ChArUco ID {}'.format(idx))
            row, col = divmod(idx, 9)
            obs.updateImagePoint(col * 7 + row, np.asarray(value, dtype=float))
        success, transform = detector.geometry().estimateTransformation(obs)
        if not success:
            pose_failed += 1
            continue
        obs.set_T_t_c(transform)
        if clearImages:
            obs.clearImage()
        accepted.append(obs)
        if (index + 1) % 100 == 0:
            print('ChArUco observations: {}/{}; accepted {}'.format(index+1, len(frames), len(accepted)), flush=True)
    print('ChArUco observations: {} accepted, {} insufficient corners, {} pose failures'.format(
        len(accepted), insufficient, pose_failed), flush=True)
    if len(accepted) < 10:
        raise RuntimeError('Too few valid ChArUco observations')
    return accepted


def main():
    if not os.environ.get('UMI_CHARUCO_OBSERVATIONS'):
        raise RuntimeError('Set UMI_CHARUCO_OBSERVATIONS to the prepared JSON')
    kc.extractCornersFromDataset = extract_charuco
    script = os.environ.get('KALIBR_IMU_CAMERA_SCRIPT',
        '/catkin_ws/src/kalibr/aslam_offline_calibration/kalibr/python/kalibr_calibrate_imu_camera')
    sys.argv[0] = script
    runpy.run_path(script, run_name='__main__')

if __name__ == '__main__':
    main()
