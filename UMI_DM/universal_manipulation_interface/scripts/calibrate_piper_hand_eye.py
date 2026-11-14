#!/usr/bin/env python3
"""Preview, record, and directly solve GoPro-on-Piper ChArUco hand-eye calibration.

The board stays fixed on the table. Move the arm manually, then pause; a new
stable joint pose is recorded automatically. Previously saved poses are skipped,
including after restarting the recorder. This script only queries feedback; it never enables,
disables, changes control mode, or commands the arm/gripper. The solve command
uses a confirmed flange-to-TCP transform and reports both TCP-to-camera and
camera-to-TCP transforms.
"""
import argparse
from collections import deque
import json
import os
from pathlib import Path
import sys
import threading
import time

import cv2
import numpy as np
from scipy.spatial.transform import Rotation
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from umi.real_world.piper_camera_geometry import restore_camera_content
from umi.real_world.piper_hand_eye import (
    CharucoPoseDetector, interpolate_flange_feedback, is_new_joint_pose,
    sdk_flange_to_matrix, solve_hand_eye, stable_joint_window)


def write_json(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


class LatestCamera:
    def __init__(self, camera, size, fps):
        self.cap = cv2.VideoCapture(int(camera) if camera.isdecimal() else camera, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.cap.release()
            raise RuntimeError(f'Cannot open {camera}')
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, size[0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, size[1])
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        fourcc = int(self.cap.get(cv2.CAP_PROP_FOURCC))
        self.actual_format = {'width': int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                              'height': int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
                              'fps': self.cap.get(cv2.CAP_PROP_FPS),
                              'fourcc': ''.join(chr((fourcc >> (8 * i)) & 255) for i in range(4))}
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.sequence = 0
        self.latest = None
        self.error = None
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        try:
            while not self.stop.is_set():
                ok, image = self.cap.read()
                received = time.monotonic()
                if not ok:
                    raise RuntimeError('Camera read failed')
                with self.lock:
                    self.sequence += 1
                    self.latest = (received, image)
        except Exception as exc:
            self.error = exc

    def read(self):
        if self.error:
            raise self.error
        with self.lock:
            if self.latest is None:
                raise ValueError('Waiting for the first camera frame')
            return self.latest

    def close(self):
        self.stop.set()
        self.thread.join(timeout=2.)
        self.cap.release()


class FeedbackReader:
    """Poll feedback independently of the preview/detector; never owns robot motion."""
    def __init__(self, session, history_s):
        self.session, self.history_s = session, history_s
        self.wall_anchor, self.monotonic_anchor = time.time(), time.monotonic()
        self.rows = deque()
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.error = None
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        try:
            while not self.stop.is_set():
                state = self.session.read_state(include_gripper=False)
                joint_wall = state['feedback_timestamps_wall_s']['joints']
                flange_wall = state['feedback_timestamps_wall_s']['flange']
                row = {'read_monotonic_s': state['time'],
                       'joint_wall_s': joint_wall, 'flange_wall_s': flange_wall,
                       # A single clock anchor keeps repeated SDK packets at
                       # the same timestamp, so cached reads never count as new feedback.
                       'joint_monotonic_s': joint_wall - self.wall_anchor + self.monotonic_anchor,
                       'flange_monotonic_s': flange_wall - self.wall_anchor + self.monotonic_anchor,
                       'flange_xyz_rpy': state['flange'].tolist(),
                       'joint_rad': state['joints'].tolist()}
                with self.lock:
                    self.rows.append(row)
                    while len(self.rows) > 2 and self.rows[1]['read_monotonic_s'] < state['time'] - self.history_s:
                        self.rows.popleft()
                self.stop.wait(.01)
        except Exception as exc:
            self.error = exc

    def snapshot(self):
        if self.error:
            raise self.error
        with self.lock:
            rows = list(self.rows)
        if rows and time.monotonic() - rows[-1]['joint_monotonic_s'] > self.session.feedback_timeout:
            raise RuntimeError('Joint feedback became stale; no sample saved')
        return rows

    def close(self):
        self.stop.set()
        self.thread.join(timeout=2.)


def feedback_motion(rows):
    matrices = np.array([sdk_flange_to_matrix(r['flange_xyz_rpy']) for r in rows])
    # Report motion over the entire capture window instead of hiding it in an average.
    drift = {'position_span_mm': (np.ptp(matrices[:, :3, 3], axis=0) * 1000).tolist(),
             'joint_span_deg': np.rad2deg(np.ptp([r['joint_rad'] for r in rows], axis=0)).tolist(),
             'max_rotation_from_first_deg': float(np.rad2deg(Rotation.from_matrix(
                 np.array([matrices[0, :3, :3].T @ t[:3, :3] for t in matrices])).magnitude()).max())}
    return drift


def load_saved_joint_poses(observations, output):
    saved = []
    for observation in observations:
        joints = observation.get('sample_joint_rad')
        if joints is None:
            # First manually recorded samples stored joints in the feedback file.
            rows = json.loads((output / observation['feedback_file']).read_text())
            joints = np.median([r['joint_rad'] for r in rows], axis=0).tolist()
        saved.append(np.asarray(joints, dtype=float))
    return saved


def make_detector(args, restored):
    return CharucoPoseDetector(json.loads(args.intrinsics.read_text()), restored.shape[1::-1],
                               squares=(args.squares_x, args.squares_y), square_mm=args.square_mm,
                               marker_mm=args.marker_mm, dictionary=args.dictionary,
                               legacy_pattern=args.legacy_pattern)


def record_or_preview(args):
    setup = yaml.safe_load(args.setup.read_text())
    geometry = setup.get('camera_image_geometry', 'native_4_3')
    size = args.camera_size or setup.get('camera_output_size', [1280, 960])
    fps = args.camera_fps or setup.get('nominal_camera_fps', 60)
    camera_latency = float(setup['latency_compensation']['camera_observation_latency_s'])
    if args.command == 'preview' and args.image:
        raw = cv2.imread(str(args.image))
        if raw is None:
            raise ValueError(f'Cannot read {args.image}')
        restored = restore_camera_content(raw, geometry)
        result, annotated = make_detector(args, restored).detect(restored)
        args.output.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(args.output / 'preview_restored.jpg'), restored)
        cv2.imwrite(str(args.output / 'preview_rectified_detection.jpg'), annotated)
        write_json(args.output / 'preview.json', result)
        print(json.dumps(result, indent=2))
        return

    arm, camera, feedback = None, None, None
    try:
        # SDK import is deliberately restricted to recording; preview and solve
        # work offline in the umi environment, without Torch or robot access.
        firmware, session = None, None
        if args.command == 'record':
            piper_root = Path(os.environ.get('PIPER_DATA_COLLECT', '/home/l1/Piper_All/Piper/Data_Collect'))
            sys.path.insert(0, str(piper_root))
            from data_collect.cli.umi_piper_sdk import connect_arm
            from data_collect.control.piper_sdk_session import PiperSdkSession
            arm, grip, firmware = connect_arm(args.can)
            session = PiperSdkSession(arm, grip)
            print('Read-only SDK connection: no motor enable, mode changes or motion commands.', flush=True)
            deadline = time.monotonic() + 3.
            while True:
                try:
                    session.read_state(include_gripper=False)
                    break
                except ValueError:
                    if time.monotonic() >= deadline:
                        raise
                    time.sleep(.05)
        camera = LatestCamera(args.camera or setup['camera'], size, fps)
        deadline = time.monotonic() + 8.
        while camera.sequence < 30:
            if camera.error:
                raise camera.error
            if time.monotonic() > deadline:
                raise RuntimeError('Camera warmup timed out')
            time.sleep(.02)
        _, raw = camera.read()
        if list(raw.shape[1::-1]) != list(size):
            raise RuntimeError(f'Requested camera size {size}, delivered {list(raw.shape[1::-1])}')
        print(f'Camera actual format: {camera.actual_format}; decoded size: {list(raw.shape[1::-1])}', flush=True)
        restored = restore_camera_content(raw, geometry)
        detector = make_detector(args, restored)
        args.output.mkdir(parents=True, exist_ok=True)
        metadata = {'squares_x': args.squares_x, 'squares_y': args.squares_y,
                    'square_mm': args.square_mm, 'marker_mm': args.marker_mm,
                    'dictionary': args.dictionary, 'legacy_pattern': args.legacy_pattern,
                    'intrinsics': str(args.intrinsics.resolve()),
                    'intrinsics_calibration': json.loads(args.intrinsics.read_text()),
                    'camera_image_geometry': geometry, 'camera_output_size': size,
                    'camera_fps_requested': fps, 'restored_size': list(restored.shape[1::-1]),
                    'camera_actual_format': camera.actual_format,
                    'camera_observation_latency_s': camera_latency,
                    'camera_observation_latency_status': (
                        'deployment_reference_unverified_for_capture_override'
                        if list(size) != setup.get('camera_output_size', [1280, 960])
                        or fps != setup.get('nominal_camera_fps', 60) else 'deployment_reference'),
                    'robot_observation_latency_s': 0.,
                    'robot_observation_latency_status': 'unmeasured_zero_approximation',
                    'flange_pose_convention': 'SDK metres and XYZ extrinsic Euler radians',
                    'transform_convention': 'T_parent_child maps child coordinates into parent',
                    'arm_command_mode': 'read_only', 'firmware': firmware}
        meta_path = args.output / 'metadata.json'
        if meta_path.exists():
            existing = json.loads(meta_path.read_text())
            if any(key in existing and existing[key] != value for key, value in metadata.items()):
                raise ValueError('Output contains a different calibration setup; use a new output directory')
        write_json(meta_path, metadata)
        observations_path = args.output / 'observations.jsonl'
        previous = ([json.loads(line) for line in observations_path.read_text().splitlines() if line]
                    if observations_path.exists() else [])
        count = max([r['sample_id'] for r in previous], default=0)
        saved_poses = load_saved_joint_poses(previous, args.output)
        once = args.once if args.command == 'record' else False
        capture_settings = None
        if args.command == 'record':
            feedback = FeedbackReader(session, max(3., args.stable_s + camera_latency + 1.))
            capture_settings = {'mode': 'automatic_joint_stability', 'stable_s': args.stable_s,
                                'stable_range_deg': args.stable_deg,
                                'min_pose_change_deg': args.min_pose_change_deg}
            print(f'Automatic capture: all joints span <= {args.stable_deg:g} deg for {args.stable_s:g}s; '
                  f'new pose separation >= {args.min_pose_change_deg:g} deg on at least one joint.', flush=True)
        print('Keep board fixed and flat. Move arm manually, then pause; capture is automatic. q quits.', flush=True)
        previous_status = None
        while True:
            received, raw = camera.read()
            restored = restore_camera_content(raw, geometry)
            detection, annotated = detector.detect(restored)
            if args.command == 'preview':
                cv2.imwrite(str(args.output / 'preview_raw.jpg'), raw)
                cv2.imwrite(str(args.output / 'preview_restored.jpg'), restored)
                cv2.imwrite(str(args.output / 'preview_rectified_detection.jpg'), annotated)
                write_json(args.output / 'preview.json', detection)
            rows, status = [], 'preview only'
            if feedback is not None:
                rows, status = stable_joint_window(feedback.snapshot(), args.stable_s, args.stable_deg,
                                                   session.feedback_timeout)
                if rows:
                    sample_joints = np.median([r['joint_rad'] for r in rows], axis=0)
                    if not is_new_joint_pose(sample_joints, saved_poses, args.min_pose_change_deg):
                        rows, status = [], 'pose already saved; move to a new pose'
                    elif not detection['pose_valid']:
                        rows, status = [], 'stable; board pose not visible'
                if status != previous_status:
                    print(f"Capture status: {status}; corners={detection['charuco_corner_count']}", flush=True)
                    previous_status = status
                    if once:
                        # Headless runs also leave a view of why capture is waiting.
                        cv2.imwrite(str(args.output / 'waiting_raw.jpg'), raw)
                        cv2.imwrite(str(args.output / 'waiting_detection.jpg'), annotated)
            if not once:
                total_corners = (args.squares_x - 1) * (args.squares_y - 1)
                display = annotated.copy()
                mode = 'AUTO' if feedback is not None else 'PREVIEW'
                text = f"corners={detection['charuco_corner_count']}/{total_corners} saved={count} | {mode} | q: quit"
                cv2.putText(display, text, (15, 30), cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 255, 255), 2)
                cv2.putText(display, status, (15, 58), cv2.FONT_HERSHEY_SIMPLEX, .6, (0, 255, 255), 2)
                cv2.imshow('Piper ChArUco hand-eye (read only)', display)
                key = cv2.waitKey(1) & 0xff
                if key in (ord('q'), 27):
                    break
            if not rows:
                if once:
                    time.sleep(.01)
                continue
            exposure = received - camera_latency
            if not rows[0]['joint_monotonic_s'] <= exposure <= rows[-1]['joint_monotonic_s']:
                # Even with stable current joints, an old frame might still
                # have been exposed during motion. Wait for a newer frame.
                continue
            try:
                matrix, bracket = interpolate_flange_feedback(rows, exposure)
            except ValueError:
                continue
            # Detection can take time. Reject a pose that started moving while
            # the current image was processed, using the independently polled feedback.
            latest, _ = stable_joint_window(feedback.snapshot(), args.stable_s, args.stable_deg,
                                              session.feedback_timeout)
            if not latest:
                continue
            combined_joints = np.array([r['joint_rad'] for r in rows + latest])
            if np.max(np.rad2deg(np.ptp(combined_joints, axis=0))) > args.stable_deg:
                continue
            drift = feedback_motion(rows)
            count += 1
            name = f'pose_{count:04d}'
            for suffix, frame in [('raw', raw), ('restored', restored), ('detection', annotated)]:
                path = args.output / f'{name}_{suffix}.png'
                if not cv2.imwrite(str(path), frame):
                    raise OSError(f'Cannot write {path}')
            write_json(args.output / f'{name}_feedback.json', rows)
            observation = dict(detection, sample_id=count, raw_image=f'{name}_raw.png',
                               restored_image=f'{name}_restored.png',
                               feedback_file=f'{name}_feedback.json',
                               frame_receive_monotonic_s=received,
                               estimated_exposure_monotonic_s=exposure,
                               T_base_flange=matrix.tolist(), feedback_bracket=bracket,
                               stationary_window=drift, sample_joint_rad=sample_joints.tolist(),
                               capture_settings=capture_settings)
            with observations_path.open('a') as file:
                file.write(json.dumps(observation, allow_nan=False) + '\n')
            saved_poses.append(sample_joints.copy())
            print(f"Saved {count}: corners={detection['charuco_corner_count']}, "
                  f"reprojection={detection['reprojection_rms_px']:.3f}px; "
                  f"position span={np.round(drift['position_span_mm'], 3).tolist()}mm, "
                  f"rotation drift={drift['max_rotation_from_first_deg']:.3f}deg", flush=True)
            if once:
                break
        print(f'Saved {count} paired poses to {args.output.resolve()}.', flush=True)
    finally:
        if feedback is not None:
            feedback.close()
        if camera is not None:
            camera.close()
        if arm is not None:
            # Do not call session.close()/stop(): this recorder never owns motion.
            arm.disconnect()
        if not (args.command == 'record' and args.once):
            cv2.destroyAllWindows()


def flange_tcp_matrix(translation_mm, rotation_rad=(0., 0., 0.)):
    """Build the confirmed TCP pose relative to the Piper SDK flange."""
    translation_mm = np.asarray(translation_mm, dtype=float)
    rotation_rad = np.asarray(rotation_rad, dtype=float)
    if translation_mm.shape != (3,) or rotation_rad.shape != (3,):
        raise ValueError('TCP flange offset must contain three translations and three rotation-vector values')
    if not np.isfinite(translation_mm).all() or not np.isfinite(rotation_rad).all():
        raise ValueError('TCP flange offset must be finite')
    result = np.eye(4)
    result[:3, :3] = Rotation.from_rotvec(rotation_rad).as_matrix()
    result[:3, 3] = translation_mm / 1000.
    return result


def direct_tcp_hand_eye(rows, flange_tcp, excluded, max_position_residual_mm,
                        max_rotation_residual_deg, max_outlier_rounds):
    """Solve directly in the TCP frame and iteratively reject bad samples."""
    active = [row for row in rows if row['sample_id'] not in excluded]
    outlier_rounds = []
    for round_index in range(max_outlier_rounds + 1):
        if len(active) < 3:
            raise ValueError('Outlier rejection left fewer than three paired poses')
        base_tcp = [np.asarray(row['T_base_flange'], dtype=float) @ flange_tcp for row in active]
        tcp_camera, diagnostics = solve_hand_eye(
            base_tcp, [row['T_camera_board'] for row in active])
        positions = np.asarray(diagnostics['board_position_residual_mm'], dtype=float)
        angles = np.asarray(diagnostics['board_rotation_residual_deg'], dtype=float)
        bad_mask = ((positions > max_position_residual_mm)
                    | (angles > max_rotation_residual_deg))
        bad = [row for row, reject in zip(active, bad_mask) if reject]
        outlier_rounds.append({
            'round': round_index + 1,
            'sample_ids': [row['sample_id'] for row in active],
            'position_residual_mm': positions.tolist(),
            'rotation_residual_deg': angles.tolist(),
            'rejected_sample_ids': [row['sample_id'] for row in bad],
        })
        if not bad:
            diagnostics = dict(diagnostics)
            diagnostics['tcp_position_span_mm'] = diagnostics.pop('flange_position_span_mm')
            diagnostics['tcp_rotation_motion_singular_values_rad'] = diagnostics.pop(
                'rotation_motion_singular_values_rad')
            return active, tcp_camera, diagnostics, outlier_rounds
        if len(active) - len(bad) < 3:
            raise ValueError('Outlier rejection would leave fewer than three paired poses')
        print(f'Outlier round {round_index + 1}: removing sample_ids '
              f'{[row["sample_id"] for row in bad]}', flush=True)
        bad_ids = {row['sample_id'] for row in bad}
        active = [row for row in active if row['sample_id'] not in bad_ids]
    raise RuntimeError('Outlier rejection did not converge within the configured rounds')


def solve(args):
    rows = [json.loads(line) for line in (args.input / 'observations.jsonl').read_text().splitlines() if line]
    requested_excluded = set(args.exclude or [])
    flange_tcp = flange_tcp_matrix(args.flange_tcp_translation_mm,
                                   args.flange_tcp_rotation_rad)
    active, tcp_camera, diagnostics, outlier_rounds = direct_tcp_hand_eye(
        rows, flange_tcp, requested_excluded, args.max_position_residual_mm,
        args.max_rotation_residual_deg, args.max_outlier_rounds)
    camera_tcp = np.linalg.inv(tcp_camera)
    flange_camera = flange_tcp @ tcp_camera
    auto_excluded = {sample_id for round_info in outlier_rounds
                     for sample_id in round_info['rejected_sample_ids']}
    excluded = sorted(requested_excluded | auto_excluded)
    patch = {'reference_frame': 'link6',
             'translation_m': flange_tcp[:3, 3].tolist(),
             'rotation': Rotation.from_matrix(flange_tcp[:3, :3]).as_rotvec().tolist(),
             'rotation_convention': 'rotation_vector_rad',
             'translation_source': 'user_confirmed_flange_to_tcp_195mm',
             'rotation_source': 'user_confirmed_flange_and_tcp_axes_aligned',
             'rotation_verified': True, 'independent_hardware_validation': False}
    result = {'method': 'cv2.CALIB_HAND_EYE_PARK',
              'frame_definition': 'T_parent_child maps child coordinates into parent',
              'sample_count': len(active),
              'sample_ids': [r['sample_id'] for r in active],
              'excluded_sample_ids': excluded,
              'T_flange_tcp_confirmed': flange_tcp.tolist(),
              'T_tcp_camera': tcp_camera.tolist(),
              'T_camera_tcp': camera_tcp.tolist(),
              'T_flange_camera': flange_camera.tolist(),
              'setup_candidate': {'flange_to_policy_tcp': patch},
              'outlier_rejection': {
                  'max_position_residual_mm': args.max_position_residual_mm,
                  'max_rotation_residual_deg': args.max_rotation_residual_deg,
                  'max_outlier_rounds': args.max_outlier_rounds,
                  'rounds': outlier_rounds,
              },
              'diagnostics': diagnostics,
              'reprojection_rms_px_by_sample': [r['reprojection_rms_px'] for r in active],
              'stationary_windows_by_sample': [r['stationary_window'] for r in active],
              'deployment_config_modified': False,
              'limitations': 'Fixed-board fit residuals are consistency checks, not independent accuracy. '
                             'The TCP flange offset is user supplied; independent hardware validation remains pending.'}
    write_json(args.output, result)
    positions = diagnostics['board_position_residual_mm']
    angles = diagnostics['board_rotation_residual_deg']
    print(f'Paired poses after outlier rejection: {len(active)}; '
          f'fixed-board position residual median/max: {np.median(positions):.3f}/{np.max(positions):.3f} mm; '
          f'rotation: {np.median(angles):.3f}/{np.max(angles):.3f} deg')
    print('Confirmed T_flange_tcp (mm):', np.round(flange_tcp[:3, 3] * 1000, 3).tolist())
    print('T_tcp_camera:\n', tcp_camera)
    print('T_camera_tcp:\n', camera_tcp)
    print('T_flange_camera:\n', flange_camera)
    print(f'Saved {args.output.resolve()}; deployment configuration was not changed.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='command', required=True)
    for name in ('preview', 'record'):
        child = commands.add_parser(name)
        child.add_argument('--setup', type=Path, default=ROOT.parent / 'deploy/current/piper_policy_setup.yaml')
        child.add_argument('--intrinsics', type=Path,
                           default=ROOT.parent / 'my_session2/calibration/gopro_intrinsics_2_7k.json')
        child.add_argument('--camera')
        child.add_argument('--camera-size', type=int, nargs=2, metavar=('WIDTH', 'HEIGHT'),
                           help='Optional calibration-only capture size; deployment config is unchanged')
        child.add_argument('--camera-fps', type=float, help='Optional calibration-only capture rate')
        child.add_argument('--squares-x', type=int, default=5)
        child.add_argument('--squares-y', type=int, default=7)
        child.add_argument('--square-mm', type=float, default=50.)
        child.add_argument('--marker-mm', type=float, default=37.5)
        child.add_argument('--dictionary', default='DICT_4X4_50')
        child.add_argument('--legacy-pattern', action='store_true')
        child.add_argument('--output', type=Path, required=True)
        if name == 'preview':
            child.add_argument('--image', type=Path, help='Offline raw HDMI frame; skips camera/CAN access')
        else:
            child.add_argument('--can', default='can0')
            child.add_argument('--once', action='store_true', help='Wait for one new stable pose, save, exit without a window')
            child.add_argument('--stable-s', type=float, default=1., help='Required continuous stationary duration')
            child.add_argument('--stable-deg', type=float, default=.1, help='Maximum per-joint peak-to-peak range in stable window')
            child.add_argument('--min-pose-change-deg', type=float, default=2.,
                               help='A new pose must differ this much on one joint from every saved pose')
    child = commands.add_parser('solve')
    child.add_argument('--input', type=Path, required=True)
    child.add_argument('--output', type=Path, required=True)
    child.add_argument('--flange-tcp-translation-mm', type=float, nargs=3, default=[0., 0., 195.],
                       metavar=('X', 'Y', 'Z'),
                       help='Confirmed TCP origin relative to link6/flange, in mm (default: 0 0 195)')
    child.add_argument('--flange-tcp-rotation-rad', type=float, nargs=3, default=[0., 0., 0.],
                       metavar=('RX', 'RY', 'RZ'),
                       help='Confirmed TCP rotation vector relative to link6/flange, in radians')
    child.add_argument('--max-position-residual-mm', type=float, default=8.,
                       help='Reject a pose when fixed-board position residual exceeds this value')
    child.add_argument('--max-rotation-residual-deg', type=float, default=1.,
                       help='Reject a pose when fixed-board rotation residual exceeds this value')
    child.add_argument('--max-outlier-rounds', type=int, default=3,
                       help='Maximum iterative residual-rejection rounds')
    # Kept as a compatibility option for old command lines. Direct TCP solving
    # intentionally does not read or combine this legacy camera-to-TCP guess.
    child.add_argument('--training-pipeline', type=Path, default=None,
                       help='Deprecated compatibility option; ignored by direct TCP solving')
    child.add_argument('--exclude', type=int, nargs='*', help='Explicitly omit identified bad sample IDs')
    args = parser.parse_args()
    if args.command in ('record', 'preview'):
        if args.camera_size is not None and min(args.camera_size) <= 0:
            parser.error('--camera-size must be positive')
        if args.camera_fps is not None and (not np.isfinite(args.camera_fps) or args.camera_fps <= 0):
            parser.error('--camera-fps must be positive and finite')
    if args.command == 'record':
        values = [args.stable_s, args.stable_deg, args.min_pose_change_deg]
        if not np.isfinite(values).all() or min(values) <= 0:
            parser.error('Stability duration/range and pose separation must be positive and finite')
    if args.command == 'solve':
        values = [*args.flange_tcp_translation_mm, *args.flange_tcp_rotation_rad,
                  args.max_position_residual_mm, args.max_rotation_residual_deg]
        if not np.isfinite(values).all() or args.max_position_residual_mm <= 0 \
                or args.max_rotation_residual_deg <= 0 or args.max_outlier_rounds < 0:
            parser.error('TCP offset, residual thresholds and outlier rounds must be finite and valid')
    solve(args) if args.command == 'solve' else record_or_preview(args)


if __name__ == '__main__':
    main()
