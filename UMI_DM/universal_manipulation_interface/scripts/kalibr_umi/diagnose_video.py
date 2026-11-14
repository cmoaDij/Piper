#!/usr/bin/env python3
"""Diagnose original and independently detected ChArUco corners and time offset.

All video frames and the original IMU samples are read. Fit/error gates are
only used for this diagnostic and never alter a calibration input file.
"""
import argparse
import csv
import json
from pathlib import Path

import av
import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.distance import pdist
from scipy.spatial.transform import Rotation


def duplicate_pairs(points):
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    return int(np.count_nonzero(pdist(points) < .1)) if len(points) > 1 else 0


def fit_pose(points, scene, K, D):
    ids = sorted(points, key=int)
    if len(ids) < 8:
        return None
    obj = np.array([scene[str(i)] for i in ids], np.float64)
    img = np.array([points[str(i)] for i in ids], np.float64).reshape(-1, 1, 2)
    undist = cv2.fisheye.undistortPoints(img, K, D, P=K)
    ok, rvec, tvec = cv2.solvePnP(obj, undist, K, np.zeros(4), flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None
    proj, _ = cv2.fisheye.projectPoints(obj.reshape(1, -1, 3), rvec, tvec, K, D)
    residual = np.linalg.norm(proj.reshape(-1, 2) - img.reshape(-1, 2), axis=1)
    return rvec.reshape(3), tvec.reshape(3), proj.reshape(-1, 2), float(np.sqrt(np.mean(residual**2))), ids


def offset_analysis(times, rotations, errors, imu_t, gyro):
    # R is the board-to-camera orientation. Norm of adjacent rotation is
    # coordinate-frame independent; compare at pair midpoint to gyroscope norm.
    valid = np.isfinite(rotations).all(axis=1) & np.isfinite(errors) & (errors < 2.0)
    idx = np.flatnonzero(valid[:-1] & valid[1:])
    delta = Rotation.from_rotvec(rotations[idx + 1]) * Rotation.from_rotvec(rotations[idx]).inv()
    visual_t = (times[idx] + times[idx + 1]) / 2
    visual = delta.magnitude() / (times[idx + 1] - times[idx])
    # Gyro bias is not removed here: angular-speed-only lag is approximate and
    # magnitude is invariant to unknown IMU/camera axis rotation.
    gyro_mag = np.linalg.norm(gyro, axis=1)
    lags = np.arange(-100, 101, 1) / 1000.0
    result = []
    for start, end in [(0, 10), (10, 20), (20, 30), (30, 40), (40, 49), (0, 49)]:
        mask = (visual_t >= start) & (visual_t < end)
        t, v = visual_t[mask], visual[mask]
        correlations = []
        for lag in lags:
            w = np.interp(t + lag, imu_t, gyro_mag, left=np.nan, right=np.nan)
            keep = np.isfinite(w) & np.isfinite(v)
            if np.count_nonzero(keep) < 30 or np.std(v[keep]) < .01 or np.std(w[keep]) < .01:
                correlations.append(np.nan)
            else:
                correlations.append(float(np.corrcoef(v[keep], w[keep])[0, 1]))
        correlations = np.asarray(correlations)
        if np.isfinite(correlations).any():
            best = int(np.nanargmax(correlations))
            result.append({'start_s': start, 'end_s': end, 'pairs': int(mask.sum()),
                           'best_shift_ms': float(lags[best] * 1000),
                           'correlation': float(correlations[best]),
                           'at_search_boundary': bool(best in (0, len(lags)-1))})
        else:
            result.append({'start_s': start, 'end_s': end, 'pairs': int(mask.sum()), 'best_shift_ms': None})
    return result


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--video', type=Path, required=True)
    ap.add_argument('--observations', type=Path, required=True)
    ap.add_argument('--intrinsics', type=Path, required=True)
    ap.add_argument('--telemetry', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    data = json.loads(args.observations.read_text())
    intr = json.loads(args.intrinsics.read_text())['intrinsics']
    telemetry = json.loads(args.telemetry.read_text())
    f = intr['focal_length']
    K = np.array([[f, 0, intr['principal_pt_x']], [0, f*intr['aspect_ratio'], intr['principal_pt_y']], [0, 0, 1.]])
    D = np.array([intr['radial_distortion_{}'.format(i)] for i in range(1, 5)])
    board = cv2.aruco.CharucoBoard((10, 8), .05, .025, cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL))
    if hasattr(board, 'setLegacyPattern'):
        board.setLegacyPattern(True)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    cv2.setNumThreads(2)
    frames = data['frames']; times = np.array([x['timestamp_ns'] for x in frames], dtype=float) / 1e9
    errors_old = np.full(len(frames), np.nan); errors_new = errors_old.copy()
    rots = np.full((len(frames), 3), np.nan)
    dups_old = np.zeros(len(frames), dtype=int); dups_new = dups_old.copy()
    corner_counts = np.zeros((len(frames), 2), dtype=int)
    independent = []
    examples = {int(np.argmin(abs(times-t))): t for t in (2, 8, 12, 17, 23, 32, 42)}
    with av.open(str(args.video)) as container:
        stream = container.streams.video[0]
        if (stream.width, stream.height) != (data['width'], data['height']): raise ValueError('Resolution mismatch')
        count = 0
        for index, frame in enumerate(container.decode(stream)):
            if index >= len(frames): raise ValueError('More frames than recorded corners')
            image = frame.to_ndarray(format='gray')
            old = frames[index]['image_points']
            marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(image, dictionary)
            fresh = {}
            if marker_ids is not None:
                n, corners, ids = cv2.aruco.interpolateCornersCharuco(marker_corners, marker_ids, image, board)
                if n and ids is not None:
                    fresh = {str(int(k)): p.tolist() for k, p in zip(ids.flatten(), corners.reshape(-1, 2))}
            dups_old[index] = duplicate_pairs(list(old.values())); dups_new[index] = duplicate_pairs(list(fresh.values()))
            corner_counts[index] = len(old), len(fresh)
            fit_old = fit_pose(old, data['scene_pts'], K, D)
            fit_new = fit_pose(fresh, data['scene_pts'], K, D)
            if fit_old is not None: errors_old[index] = fit_old[3]
            if fit_new is not None: rots[index], errors_new[index] = fit_new[0], fit_new[3]
            independent.append({'timestamp_ns': frames[index]['timestamp_ns'], 'image_points': fresh})
            if index in examples:
                overlay = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
                # Green: independently detected corners. Red circles/labels:
                # previous IDs which share a pixel. Yellow: previous fit projection.
                for p in fresh.values(): cv2.circle(overlay, tuple(np.round(p).astype(int)), 4, (0, 220, 0), 2)
                ids = list(old); p = np.array([old[k] for k in ids])
                for i in range(len(ids)):
                    for j in range(i+1, len(ids)):
                        if np.linalg.norm(p[i]-p[j]) < .1:
                            center = tuple(np.round(p[i]).astype(int));cv2.circle(overlay, center, 16, (0,0,255), 3)
                            cv2.putText(overlay, ids[i]+'/'+ids[j], (center[0]+12,center[1]-12), cv2.FONT_HERSHEY_SIMPLEX,.65,(0,0,255),2)
                if fit_old:
                    for point in fit_old[2]: cv2.circle(overlay, tuple(np.round(point).astype(int)), 2, (0,255,255), -1)
                cv2.putText(overlay, 't={:.2f}s old duplicate pairs={} old RMSE={:.2f}px new RMSE={:.2f}px'.format(times[index], dups_old[index],errors_old[index],errors_new[index]),(25,55),cv2.FONT_HERSHEY_SIMPLEX,1.2,(255,255,255),3)
                cv2.imwrite(str(args.output / ('frame_{:02d}s.jpg'.format(examples[index]))), overlay, [cv2.IMWRITE_JPEG_QUALITY, 90])
            count = index + 1
            if count % 300 == 0: print('Processed {}/{} frames'.format(count,len(frames)), flush=True)
        if count != len(frames): raise ValueError('Frame count mismatch')
    (args.output/'independent_corners.json').write_text(json.dumps({'width':data['width'],'height':data['height'],'scene_pts':data['scene_pts'],'frames':independent},separators=(',', ':')))
    with (args.output/'per_frame.csv').open('w',newline='') as out:
        w=csv.writer(out);w.writerow(['frame','time_s','old_corners','new_corners','old_duplicate_pairs','new_duplicate_pairs','old_pnp_rmse_px','new_pnp_rmse_px','new_rot_x','new_rot_y','new_rot_z'])
        for i in range(len(frames)):w.writerow([i,times[i],*corner_counts[i],dups_old[i],dups_new[i],errors_old[i],errors_new[i],*rots[i]])
    imu_t=np.array(telemetry['timestamps_ns'],dtype=float)/1e9;gyro=np.array(telemetry['gyroscope'],dtype=float)
    offsets=offset_analysis(times,rots,errors_new,imu_t,gyro)
    summary={'frames':len(frames),'old_duplicate_frames':int(np.count_nonzero(dups_old)), 'old_duplicate_pairs':int(dups_old.sum()),
             'new_duplicate_frames':int(np.count_nonzero(dups_new)), 'new_duplicate_pairs':int(dups_new.sum()),
             'old_rmse_percentiles_px':np.nanpercentile(errors_old,[50,90,95,99]).tolist(),
             'new_rmse_percentiles_px':np.nanpercentile(errors_new,[50,90,95,99]).tolist(),
             'new_pose_fit_fraction_below_2px':float(np.mean(errors_new<2)),
             'speed_correlation_shift_convention':'gyro evaluated at video_time + shift',
             'speed_correlation_diagnostic_only':offsets}
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2))
    fig,ax=plt.subplots(2,1,figsize=(14,7),sharex=True)
    ax[0].plot(times,errors_old,label='existing OpenICC corners',alpha=.65,linewidth=.6);ax[0].plot(times,errors_new,label='independent OpenCV corners',alpha=.7,linewidth=.6);ax[0].set_ylim(0,100);ax[0].set_ylabel('Per-frame PnP RMSE (px)');ax[0].legend()
    ax[1].plot(times,dups_old,label='existing duplicates',linewidth=.6);ax[1].plot(times,dups_new,label='independent duplicates',linewidth=.6);ax[1].set_ylabel('Duplicate ID pairs');ax[1].set_xlabel('Image timestamp (seconds)');ax[1].legend();fig.tight_layout();fig.savefig(args.output/'corner_diagnostics.png',dpi=150);plt.close(fig)
    print(json.dumps(summary,indent=2),flush=True)

if __name__ == '__main__':main()
