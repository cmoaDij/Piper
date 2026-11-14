"""Run a configured UMI session in a new output directory without changing inputs."""
import datetime
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys

import click
import yaml

ROOT_DIR = Path(__file__).resolve().parent.parent
SCRIPTS_DIR = ROOT_DIR / 'scripts_slam_pipeline'


def resolve_file(session, value, name):
    if not isinstance(value, str) or not value:
        raise click.ClickException(f'{name} must be a nonempty path')
    path = Path(os.path.expanduser(value))
    if not path.is_absolute():
        path = session / path
    path = path.resolve()
    if not path.is_file():
        raise click.ClickException(f'{name} file does not exist: {path}')
    return path


def numeric_list(value, count, name):
    import math
    if (not isinstance(value, (list, tuple)) or len(value) != count
            or any(not isinstance(x, (int, float)) or isinstance(x, bool) or not math.isfinite(x) for x in value)):
        raise click.ClickException(f'{name} must have {count} finite numeric values')
    return list(value)


def read_config(session, config_path):
    with config_path.open() as f:
        config = yaml.safe_load(f)
    if not isinstance(config, dict) or config.get('version') != 1:
        raise click.ClickException('Expected pipeline config version: 1')
    paths = {key: resolve_file(session, config.get(key), key) for key in
             ('camera_intrinsics', 'aruco_config', 'slam_setting', 'training_mask')}
    pose = numeric_list(config.get('pose_cam_tcp'), 6, 'pose_cam_tcp')
    width = numeric_list(config.get('actual_gripper_range_m'), 2, 'actual_gripper_range_m')
    if width[0] != 0 or width[1] <= width[0]:
        raise click.ClickException('actual_gripper_range_m must start at 0 and have a positive maximum')
    z = numeric_list([config.get('nominal_z_m')], 1, 'nominal_z_m')[0]
    if z <= 0:
        raise click.ClickException('nominal_z_m must be positive')
    workers = config.get('workers', 2)
    if not isinstance(workers, int) or isinstance(workers, bool) or workers < 1:
        raise click.ClickException('workers must be a positive integer')
    tag_ids = config.get('inpaint_tag_ids')
    if tag_ids is not None and (not isinstance(tag_ids, list) or not tag_ids
                                or any(not isinstance(x, int) or isinstance(x, bool) or x < 0 for x in tag_ids)):
        raise click.ClickException('inpaint_tag_ids must be a nonempty list of nonnegative integers')
    for key in ('mask_mirror_regions', 'no_docker_pull', 'kalibr_time_shift_applied'):
        if not isinstance(config.get(key), bool):
            raise click.ClickException(f'{key} must be true or false')
    if config['kalibr_time_shift_applied']:
        raise click.ClickException('Configured pipeline currently supports only the legacy SLAM time alignment')
    for key in ('slam_docker_image', 'imu_docker_image'):
        if not isinstance(config.get(key), str) or not config[key]:
            raise click.ClickException(f'{key} must be a nonempty Docker image name')
    with paths['training_mask'].open() as f:
        mask = json.load(f)
    points = mask.get('polygon')
    if (mask.get('reference_resolution') != [2704, 2028] or not isinstance(points, list) or len(points) < 3):
        raise click.ClickException('training_mask must contain a 2704 x 2028 reference polygon')
    for point in points:
        numeric_list(point, 2, 'training_mask polygon point')
        if not (0 <= point[0] < 2704 and 0 <= point[1] < 2028):
            raise click.ClickException('training_mask polygon point outside image')
    with paths['camera_intrinsics'].open() as f:
        intrinsics = json.load(f)
    if (intrinsics.get('intrinsic_type') != 'FISHEYE'
            or [intrinsics.get('image_width'), intrinsics.get('image_height')] != mask['reference_resolution']):
        raise click.ClickException('Camera intrinsics and training mask must use the same 2704 x 2028 fisheye image')
    # SLAM and ArUco use different files. Check that the SLAM file uses the
    # same intrinsics at its configured full-frame resize, instead of silently
    # running with a generic GoPro calibration.
    import cv2
    import numpy as np
    try:
        storage = cv2.FileStorage(str(paths['slam_setting']), cv2.FILE_STORAGE_READ)
        if not storage.isOpened():
            raise ValueError('Cannot open SLAM settings')
        def value(key):
            node = storage.getNode(key)
            if node.empty():
                raise ValueError(f'Missing {key}')
            return node.real()
        slam_width, slam_height = value('Camera.width'), value('Camera.height')
        ratio_x = slam_width / intrinsics['image_width']
        ratio_y = slam_height / intrinsics['image_height']
        if ratio_x <= 0 or not np.isclose(ratio_x, ratio_y, atol=1e-9):
            raise ValueError('SLAM settings do not use an equal full-frame resize')
        source = intrinsics['intrinsics']
        expected = [source['focal_length'] * ratio_x,
                    source['focal_length'] * source.get('aspect_ratio', 1.0) * ratio_y,
                    source['principal_pt_x'] * ratio_x, source['principal_pt_y'] * ratio_y]
        expected += [source[f'radial_distortion_{i}'] for i in range(1, 5)]
        actual = [value('Camera1.' + key) for key in ('fx', 'fy', 'cx', 'cy', 'k1', 'k2', 'k3', 'k4')]
        if not np.allclose(actual, expected, rtol=1e-6, atol=1e-8):
            raise ValueError('SLAM settings do not match the configured camera intrinsics')
        transform = storage.getNode('IMU.T_b_c1').mat()
        if (transform is None or transform.shape != (4, 4) or not np.isfinite(transform).all()
                or not np.allclose(transform[3], [0, 0, 0, 1], atol=1e-6)
                or not np.allclose(transform[:3,:3].T @ transform[:3,:3], np.eye(3), atol=1e-5)
                or not np.isclose(np.linalg.det(transform[:3,:3]), 1, atol=1e-5)):
            raise ValueError('SLAM IMU.T_b_c1 is not a valid rigid transform')
    except (KeyError, ValueError, cv2.error) as exc:
        raise click.ClickException(f'Invalid SLAM camera/IMU settings: {exc}') from exc
    finally:
        if 'storage' in locals():
            storage.release()
    return config, paths, pose, width, z, workers


def link_raw_files(session, output, validate_video=False):
    raw = session / 'raw_videos'
    if not raw.is_dir():
        raise click.ClickException(f'Missing raw video directory: {raw}')
    videos = sorted(p for p in raw.rglob('*') if p.is_file() and p.suffix.lower() == '.mp4')
    if not videos:
        raise click.ClickException(f'No MP4 videos under {raw}')
    if validate_video:
        import av
        if not any(p.parent == raw and p.name.lower() == 'mapping.mp4' for p in videos):
            raise click.ClickException('Place the mapping recording at raw_videos/mapping.MP4')
        if not any(p.is_relative_to(raw / 'gripper_calibration') for p in videos):
            raise click.ClickException('Place a gripper calibration recording under raw_videos/gripper_calibration/')
        if not any(p.parent == raw and p.name.lower() != 'mapping.mp4' for p in videos):
            raise click.ClickException('Place at least one demonstration recording directly under raw_videos/')
        for source in videos:
            try:
                with av.open(str(source)) as container:
                    stream = container.streams.video[0]
                    resolution = (stream.width, stream.height)
            except Exception as exc:
                raise click.ClickException(f'Cannot read video {source}: {exc}') from exc
            if resolution != (2704, 2028):
                raise click.ClickException(f'{source} has resolution {resolution}; configured calibration requires 2704 x 2028')
    records = []
    for source in videos:
        destination = output / 'raw_videos' / source.relative_to(raw)
        destination.parent.mkdir(parents=True, exist_ok=True)
        try:
            os.link(source.resolve(), destination)
            mode = 'hardlink'
        except OSError:
            shutil.copy2(source, destination)
            mode = 'copy'
        records.append({'source': str(source), 'size_bytes': source.stat().st_size, 'copy_mode': mode})
    return records


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def run_stage(name, *args):
    cmd = [sys.executable, str(SCRIPTS_DIR / name), *map(str, args)]
    print('\n>>>', ' '.join(cmd), flush=True)
    env = os.environ.copy()
    env['PATH'] = str(Path(sys.executable).parent) + os.pathsep + env.get('PATH', '')
    subprocess.run(cmd, cwd=ROOT_DIR, env=env, check=True)


def run_configured(session, config_path):
    session = Path(session).expanduser().resolve()
    config_path = Path(config_path).expanduser().resolve()
    config, paths, pose, width, z, workers = read_config(session, config_path)
    now = datetime.datetime.now().astimezone()
    output = session / 'processed_runs' / now.strftime('%Y%m%d_%H%M%S_%f')
    output.mkdir(parents=True, exist_ok=False)
    manifest_path = output / 'run_manifest.json'
    manifest = {'status': 'running', 'started_at': now.isoformat(), 'source_session': str(session),
                'config': str(config_path), 'settings': config,
                'input_file_sha256': {key: sha256(path) for key, path in paths.items()},
                'config_sha256': sha256(config_path), 'output': str(output)}
    def save_manifest():
        manifest_path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False)+'\n')
    save_manifest()
    try:
        # Preserve exactly the inputs used even if the session settings are
        # edited while this run is processing or after it has completed.
        snapshot_dir = output / 'calibration_snapshot'
        snapshot_dir.mkdir()
        snapshots = {}
        for key, source in paths.items():
            destination = snapshot_dir / (key + source.suffix)
            shutil.copy2(source, destination)
            if sha256(destination) != manifest['input_file_sha256'][key]:
                raise RuntimeError(f'Configuration changed while copying {source}')
            snapshots[key] = destination
        shutil.copy2(config_path, snapshot_dir / 'pipeline.yaml')
        if sha256(snapshot_dir / 'pipeline.yaml') != manifest['config_sha256']:
            raise RuntimeError('Pipeline configuration changed while being copied')
        manifest['calibration_snapshot'] = str(snapshot_dir)
        paths = snapshots
        manifest['raw_videos'] = link_raw_files(session, output, validate_video=True)
        save_manifest()
        pull_flag = ['-np'] if config['no_docker_pull'] else []
        no_mirror = ['--no-mirror-mask'] if not config['mask_mirror_regions'] else []
        run_stage('00_process_videos.py', output)
        run_stage('01_extract_gopro_imu.py', '--docker_image', config['imu_docker_image'],
                  '--num_workers', workers, *pull_flag, output)
        demos = output / 'demos';mapping = demos / 'mapping';map_path = mapping / 'map_atlas.osa'
        run_stage('02_create_map.py', '--input_dir', mapping, '--map_path', map_path,
                  '--docker_image', config['slam_docker_image'], '--slam-setting', paths['slam_setting'],
                  *pull_flag, *no_mirror)
        run_stage('03_batch_slam.py', '--input_dir', demos, '--map_path', map_path,
                  '--docker_image', config['slam_docker_image'], '--slam-setting', paths['slam_setting'],
                  '--num_workers', workers, '--skip-mapping', *pull_flag, *no_mirror)
        run_stage('04_detect_aruco.py', '--input_dir', demos, '--camera_intrinsics', paths['camera_intrinsics'],
                  '--aruco_yaml', paths['aruco_config'], '--num_workers', workers, *no_mirror)
        run_stage('05_run_calibrations.py', '--nominal-z', z, output)
        run_stage('06_generate_dataset_plan.py', '--input', output, '--nominal_z', z,
                  '--pose-cam-tcp', ','.join(map(str, pose)),
                  '--actual-gripper-range', ','.join(map(str, width)))
        mirror_flag = ['--no_mirror'] if config['mask_mirror_regions'] else []
        inpaint_flag = (['--inpaint-tag-ids', ','.join(map(str, config['inpaint_tag_ids']))]
                        if config.get('inpaint_tag_ids') is not None else [])
        run_stage('07_generate_replay_buffer.py', '--output', output/'dataset.zarr.zip',
                  '--num_workers', workers, '--mask-polygon-json', paths['training_mask'],
                  *mirror_flag, *inpaint_flag, output)
        if not (output/'dataset.zarr.zip').is_file():
            raise RuntimeError('Pipeline did not generate the dataset')
        manifest['status'] = 'complete'
        manifest['completed_at'] = datetime.datetime.now().astimezone().isoformat()
        manifest['dataset'] = str(output/'dataset.zarr.zip')
        save_manifest()
        print('\nConfigured run complete:', output, flush=True)
    except BaseException as exc:
        manifest['status'] = 'failed'
        manifest['error'] = str(exc)
        save_manifest()
        print('\nConfigured run failed; see:', output, file=sys.stderr, flush=True)
        raise
    return output
