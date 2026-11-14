#!/usr/bin/env python3
"""Build a full-resolution ROS1 bag and ChArUco observation file from OpenICC data."""
import argparse
import json
from pathlib import Path

import av
import numpy as np
import ubjson
from rosbags.rosbag1 import Writer
from rosbags.typesys import Stores, get_typestore


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--video', required=True, type=Path)
    parser.add_argument('--telemetry', required=True, type=Path, help='OpenICC *_gen.json')
    parser.add_argument('--corners', required=True, type=Path, help='full resolution OpenICC .uson')
    parser.add_argument('--intrinsics', required=True, type=Path, help='full resolution OpenICC FISHEYE JSON')
    parser.add_argument('--output', required=True, type=Path, help='new output directory')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    bag_path = args.output / 'gopro_fullres.bag'
    if bag_path.exists():
        raise FileExistsError(f'Refusing to overwrite {bag_path}')
    with args.telemetry.open() as f:
        telem = json.load(f)
    with args.corners.open('rb') as f:
        corner = ubjson.load(f)
    with args.intrinsics.open() as f:
        intr = json.load(f)
    if intr['intrinsic_type'] != 'FISHEYE':
        raise ValueError('Expected OpenICC FISHEYE intrinsics')
    w, h = intr['image_width'], intr['image_height']
    if (corner['image_width'], corner['image_height']) != (w, h):
        raise ValueError('Corner and calibration image resolutions differ')
    if abs(corner['square_size_meter'] - 0.05) > 1e-6:
        raise ValueError('Corner board square length is not the expected 50 mm')
    times = np.asarray(telem['img_timestamps_ns'], dtype=np.float64)
    imu_times = np.asarray(telem['timestamps_ns'], dtype=np.float64)
    accel = np.asarray(telem['accelerometer'], dtype=np.float64)
    gyro = np.asarray(telem['gyroscope'], dtype=np.float64)
    views = sorted(corner['views'].items(), key=lambda x: float(x[0]))
    if len(views) != len(times) or len(accel) != len(imu_times) or len(gyro) != len(imu_times):
        raise ValueError('Frame or IMU sample counts do not agree')
    if not np.isfinite(times).all() or not np.isfinite(imu_times).all() or np.any(np.diff(times) <= 0) or np.any(np.diff(imu_times) <= 0):
        raise ValueError('Timestamps must be finite and strictly increasing')
    if not np.isfinite(accel).all() or not np.isfinite(gyro).all():
        raise ValueError('IMU contains non-finite values')
    offsets_ns = np.array([float(key) * 1000 for key, _ in views])
    mismatch_ns = np.abs(times - times[0] - offsets_ns)
    if np.max(mismatch_ns) > 5e6:
        raise ValueError(f'Image telemetry and video timestamps disagree by up to {np.max(mismatch_ns)/1e6:.3f} ms')
    if imu_times[0] >= times[-1] or imu_times[-1] <= times[0]:
        raise ValueError('IMU and camera timestamp intervals do not overlap')

    # Original ChArUco IDs are OpenCV row-major: 7 rows, 9 columns.
    # Kalibr checkerboard geometry uses x=row, y=column, so transpose the index.
    pts = corner['scene_pts']
    for idx in range(63):
        expected = np.array([(idx % 9 + 1) * .05, (idx // 9 + 1) * .05, 0])
        if not np.allclose(pts[str(idx)], expected, atol=1e-6):
            raise ValueError(f'Unexpected ChArUco coordinate for ID {idx}')
    data = {'source_video': str(args.video.resolve()), 'width': w, 'height': h,
            'square_size_m': .05, 'squares_x': 10, 'squares_y': 8,
            'scene_pts': pts,
            'frames': [{'timestamp_ns': int(round(t)), 'image_points': view['image_points']}
                       for t, (_, view) in zip(times, views)]}
    (args.output / 'charuco_observations.json').write_text(json.dumps(data, separators=(',', ':')))
    x = intr['intrinsics']
    if abs(x.get('skew', 0)) > 1e-12:
        raise ValueError('Kalibr pinhole-equi camera does not represent nonzero skew')
    f = x['focal_length']
    cam = {'cam0': {'camera_model': 'pinhole', 'distortion_model': 'equidistant',
                    'intrinsics': [f, f*x['aspect_ratio'], x['principal_pt_x'], x['principal_pt_y']],
                    'distortion_coeffs': [x[f'radial_distortion_{i}'] for i in range(1,5)],
                    'resolution': [w, h], 'rostopic': '/cam0/image_raw', 'cam_overlaps': []}}
    import yaml
    (args.output / 'camchain.yaml').write_text(yaml.safe_dump(cam, sort_keys=False))
    # These are provisional defaults from this repo's ORB-SLAM3 GoPro configuration,
    # not measurements of the user's individual IMU.
    imu = {'accelerometer_noise_density': 0.017, 'accelerometer_random_walk': 0.0055,
           'gyroscope_noise_density': 0.0015, 'gyroscope_random_walk': 0.00005,
           'rostopic': '/imu0', 'update_rate': float(1e9 / np.median(np.diff(imu_times)))}
    (args.output / 'imu_provisional.yaml').write_text(yaml.safe_dump(imu, sort_keys=False))
    # Geometry only; image observations are provided by charuco_adapter.py.
    target = {'target_type': 'checkerboard', 'targetRows': 9, 'targetCols': 7,
              'rowSpacingMeters': .05, 'colSpacingMeters': .05}
    (args.output / 'target_geometry.yaml').write_text(yaml.safe_dump(target, sort_keys=False))

    store = get_typestore(Stores.ROS1_NOETIC)
    types = store.types
    Time = types['builtin_interfaces/msg/Time']; Header = types['std_msgs/msg/Header']
    Image = types['sensor_msgs/msg/Image']; Imu = types['sensor_msgs/msg/Imu']
    Vec = types['geometry_msgs/msg/Vector3']; Quat = types['geometry_msgs/msg/Quaternion']
    def header(ns, seq, name):
        return Header(seq, Time(ns // 1_000_000_000, ns % 1_000_000_000), name)
    def write_imu(writer, connection, idx):
        ns = int(round(imu_times[idx])); a, g = accel[idx], gyro[idx]
        cov = np.zeros(9, dtype=np.float64); cov[0] = -1
        msg = Imu(header(ns, idx, 'imu0'), Quat(0.,0.,0.,1.), cov,
                  Vec(*map(float, g)), np.zeros(9), Vec(*map(float, a)), np.zeros(9))
        writer.write(connection, ns, store.serialize_ros1(msg, 'sensor_msgs/msg/Imu'))
    # ROS Noetic's roslz4 rejects the LZ4 framing emitted by some rosbags
    # versions. Uncompressed ROS1 chunks are interoperable without re-encoding
    # or rescaling any image and avoid an expensive bzip2 pass.
    writer = Writer(bag_path)
    imu_idx = 0
    try:
        with writer, av.open(str(args.video)) as container:
            stream = container.streams.video[0]
            if (stream.width, stream.height) != (w, h):
                raise ValueError('Video and intrinsics resolution mismatch')
            cam_conn = writer.add_connection('/cam0/image_raw', 'sensor_msgs/msg/Image', typestore=store)
            imu_conn = writer.add_connection('/imu0', 'sensor_msgs/msg/Imu', typestore=store)
            n = 0
            first_video_pts = None
            max_video_pts_mismatch_ns = 0.0
            for n, frame in enumerate(container.decode(stream), 1):
                if n > len(times): raise ValueError('Video has more frames than telemetry')
                ns = int(round(times[n-1]))
                if frame.pts is None or frame.time_base is None:
                    raise ValueError('Decoded video frame is missing its presentation timestamp')
                video_pts = float(frame.pts * frame.time_base) * 1e9
                if first_video_pts is None:
                    first_video_pts = video_pts
                pts_mismatch_ns = abs((video_pts - first_video_pts) - (times[n-1] - times[0]))
                max_video_pts_mismatch_ns = max(max_video_pts_mismatch_ns, pts_mismatch_ns)
                if pts_mismatch_ns > 5e6:
                    raise ValueError(f'Frame {n-1} presentation timestamp differs from telemetry by {pts_mismatch_ns/1e6:.3f} ms')
                while imu_idx < len(imu_times) and int(round(imu_times[imu_idx])) <= ns:
                    write_imu(writer, imu_conn, imu_idx); imu_idx += 1
                image = np.ascontiguousarray(frame.to_ndarray(format='gray'))
                msg = Image(header(ns, n-1, 'cam0'), h, w, 'mono8', 0, w, image.reshape(-1))
                writer.write(cam_conn, ns, store.serialize_ros1(msg, 'sensor_msgs/msg/Image'))
                if n % 100 == 0: print(f'frames {n}/{len(times)}', flush=True)
            if n != len(times): raise ValueError(f'Video has {n} frames, expected {len(times)}')
            while imu_idx < len(imu_times): write_imu(writer, imu_conn, imu_idx); imu_idx += 1
    except BaseException:
        if bag_path.exists(): bag_path.unlink()
        raise
    print(f'Wrote {n} full-resolution frames and {imu_idx} IMU samples to {bag_path}')
    print(f'Maximum difference between image timestamps and corner view timestamps: {np.max(mismatch_ns)/1e6:.3f} ms')
    print(f'Maximum difference between image timestamps and decoded video PTS: {max_video_pts_mismatch_ns/1e6:.3f} ms')

if __name__ == '__main__':
    main()
