"""Regression checks for session config and preservation of raw inputs."""
import json
from pathlib import Path
import tempfile

import click
import yaml

from scripts_slam_pipeline.configured_pipeline import link_raw_files, read_config


def test():
    with tempfile.TemporaryDirectory() as tmp:
        session = Path(tmp) / 'session'
        calibration = session / 'calibration'
        calibration.mkdir(parents=True)
        for name in ('intrinsics.json', 'aruco.yaml', 'slam.yaml'):
            (calibration / name).write_text('{}')
        intr = {'focal_length': 800, 'aspect_ratio': 1, 'principal_pt_x': 1352, 'principal_pt_y': 1014}
        intr.update({f'radial_distortion_{i}': 0 for i in range(1, 5)})
        (calibration / 'intrinsics.json').write_text(json.dumps({
            'intrinsic_type': 'FISHEYE', 'image_width': 2704, 'image_height': 2028, 'intrinsics': intr}))
        scale = 960 / 2704
        (calibration / 'slam.yaml').write_text(f'''%YAML:1.0
Camera.width: 960
Camera.height: 720
Camera1.fx: {800*scale}
Camera1.fy: {800*scale}
Camera1.cx: 480
Camera1.cy: 360
Camera1.k1: 0
Camera1.k2: 0
Camera1.k3: 0
Camera1.k4: 0
IMU.T_b_c1: !!opencv-matrix
    rows: 4
    cols: 4
    dt: f
    data: [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]
''')
        (calibration / 'mask.json').write_text(json.dumps({
            'reference_resolution': [2704, 2028],
            'polygon': [[0, 1800], [1352, 1930], [2703, 1800], [2703, 2027], [0, 2027]]}))
        config = {'version': 1, 'camera_intrinsics': 'calibration/intrinsics.json',
                  'aruco_config': 'calibration/aruco.yaml', 'slam_setting': 'calibration/slam.yaml',
                  'training_mask': 'calibration/mask.json', 'slam_docker_image': 'slam:test',
                  'imu_docker_image': 'imu:test', 'pose_cam_tcp': [0, .096, .138, 0, 0, 0],
                  'actual_gripper_range_m': [0, .076], 'nominal_z_m': .0505, 'workers': 2,
                  'mask_mirror_regions': False, 'no_docker_pull': True, 'kalibr_time_shift_applied': False}
        path = calibration / 'pipeline.yaml'
        path.write_text(yaml.safe_dump(config))
        _, _, pose, width, depth, workers = read_config(session, path)
        assert pose == [0, .096, .138, 0, 0, 0] and width == [0, .076] and depth == .0505 and workers == 2
        config['inpaint_tag_ids'] = [0, 1]
        path.write_text(yaml.safe_dump(config))
        assert read_config(session, path)[0]['inpaint_tag_ids'] == [0, 1]
        config['inpaint_tag_ids'] = [True]
        path.write_text(yaml.safe_dump(config))
        try:
            read_config(session, path)
        except click.ClickException as exc:
            assert 'inpaint_tag_ids' in str(exc)
        else:
            raise AssertionError('Invalid marker IDs must not be silently accepted')
        config['inpaint_tag_ids'] = [0, 1]
        path.write_text(yaml.safe_dump(config))
        slam_path = calibration / 'slam.yaml'
        correct_slam = slam_path.read_text()
        slam_path.write_text(correct_slam.replace('Camera1.cx: 480', 'Camera1.cx: 400'))
        try:
            read_config(session, path)
        except click.ClickException as exc:
            assert 'do not match' in str(exc)
        else:
            raise AssertionError('Mismatched camera intrinsics must not be silently accepted')
        slam_path.write_text(correct_slam)
        raw = session / 'raw_videos'
        (raw / 'gripper_calibration').mkdir(parents=True)
        source = raw / 'gripper_calibration/recording.MP4'
        source.write_bytes(b'original video bytes')
        output = session / 'processed_runs/new'
        output.mkdir(parents=True)
        records = link_raw_files(session, output)
        copied = output / 'raw_videos/gripper_calibration/recording.MP4'
        assert copied.read_bytes() == source.read_bytes() and len(records) == 1
        # Removing the run's own link must not affect the original capture.
        copied.unlink()
        assert source.read_bytes() == b'original video bytes'
        config['kalibr_time_shift_applied'] = True
        path.write_text(yaml.safe_dump(config))
        try:
            read_config(session, path)
        except click.ClickException as exc:
            assert 'legacy SLAM time alignment' in str(exc)
        else:
            raise AssertionError('Unsupported time correction must not be silently accepted')


if __name__ == '__main__':
    test()
    print('Configured pipeline validation passed')
