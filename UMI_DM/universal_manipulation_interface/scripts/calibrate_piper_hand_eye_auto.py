#!/usr/bin/env python3
"""Automatically move Piper through verified joint poses and record ChArUco pairs.

This is an explicit-motion command. It requires ``--execute`` and uses only
one Piper SDK connection. Targets are taken from poses that were already
observed with the board visible, so the arm does not depend on manual posing.
For every target, the script requires a continuous fresh-feedback window,
holds the target for an additional settling interval, and only then records a
new camera frame plus a confirming robot state. The script never commands the
gripper and leaves the arm at the last pose when the run finishes. Press
Ctrl+C to request a position hold and stop.
"""
import argparse
import json
import os
from pathlib import Path
import signal
import sys
import time

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from scripts.calibrate_piper_hand_eye import LatestCamera, make_detector, write_json
from umi.real_world.piper_camera_geometry import restore_camera_content


PIPER_ROOT = Path(os.environ.get('PIPER_DATA_COLLECT', '/home/l1/Piper_All/Piper/Data_Collect'))


def load_targets(path, sample_ids=None, max_poses=20):
    path = Path(path)
    if path.suffix.lower() == '.jsonl':
        rows = [json.loads(line) for line in path.read_text().splitlines() if line.strip()]
    else:
        payload = json.loads(path.read_text())
        rows = payload['targets'] if isinstance(payload, dict) else payload
    if sample_ids:
        wanted = set(sample_ids)
        rows = [row for row in rows if int(row.get('sample_id', -1)) in wanted]
    if max_poses > 0:
        rows = rows[:max_poses]
    targets = []
    for row in rows:
        joints = row.get('sample_joint_rad', row.get('target_joint_rad'))
        if joints is None:
            raise ValueError(f'Target {row.get("sample_id")} has no six-joint pose')
        joints = np.asarray(joints, dtype=float)
        if joints.shape != (6,) or not np.isfinite(joints).all():
            raise ValueError(f'Target {row.get("sample_id")} has invalid joint angles')
        targets.append({'source_sample_id': row.get('sample_id', row.get('source_sample_id')),
                        'target_joint_rad': joints.tolist(),
                        'target_joint_deg': np.rad2deg(joints).tolist()})
    if len(targets) < 3:
        raise ValueError('Need at least three automatic poses')
    return targets


def wait_for_pose(session, target, timeout_s, tolerance_deg, resend_s=.10):
    deadline = time.monotonic() + timeout_s
    tolerance = np.deg2rad(tolerance_deg)
    last_send = float('-inf')
    last_state = None
    while time.monotonic() < deadline:
        state = session.read_state(include_gripper=False)
        last_state = state
        if np.max(np.abs(state['joints'] - target)) <= tolerance:
            return state
        if time.monotonic() - last_send >= resend_s:
            session.send_joints(target)
            last_send = time.monotonic()
        time.sleep(.02)
    raise TimeoutError(f'Pose was not reached within {timeout_s:g}s: '
                       f'{np.rad2deg(last_state["joints"] if last_state else target).round(2).tolist()}')


def wait_for_stability(session, target, duration_s, max_range_deg, timeout_s,
                       tolerance_deg, max_feedback_gap_s=.10):
    """Wait for a fresh, continuous, in-target feedback window.

    A cached SDK packet must never extend the settling window.  Every sample
    therefore needs a strictly newer joint-feedback timestamp, and a packet
    gap resets the window.  The target tolerance is checked for every sample,
    rather than only for the final sample, so a pose that is still drifting
    into the target cannot be captured.
    """
    deadline = time.monotonic() + timeout_s
    stamps, joints = [], []
    last_send = float('-inf')
    last_stamp = None
    while time.monotonic() < deadline:
        if time.monotonic() - last_send >= .10:
            session.send_joints(target)
            last_send = time.monotonic()
        state = session.read_state(include_gripper=False)
        stamp = float(state['feedback_timestamps_monotonic_s']['joints'])
        if not np.isfinite(stamp):
            raise ValueError('Joint feedback timestamp is not finite')
        # PiperSdkSession rejects backward timestamps, but cached reads can
        # still repeat a packet. Repeated packets do not count as settling.
        if last_stamp is not None:
            if stamp <= last_stamp:
                time.sleep(.01)
                continue
            if stamp - last_stamp > max_feedback_gap_s:
                stamps.clear()
                joints.clear()
        last_stamp = stamp
        stamps.append(stamp)
        joints.append(np.asarray(state['joints'], dtype=float).copy())
        cutoff = stamps[-1] - duration_s
        # Keep the sample immediately before the cutoff so the measured
        # interval is at least duration_s instead of one feedback period short.
        start = max(0, int(np.searchsorted(stamps, cutoff, side='right')) - 1)
        window = np.asarray(joints[start:])
        target_error_deg = np.rad2deg(np.max(np.abs(window - target), axis=1)) if len(window) else []
        if (len(window) >= 3
                and stamps[-1] - stamps[start] >= duration_s
                and np.max(np.rad2deg(np.ptp(window, axis=0))) <= max_range_deg
                and np.max(target_error_deg) <= tolerance_deg):
            stability = {
                'stable_start_feedback_monotonic_s': float(stamps[start]),
                'stable_end_feedback_monotonic_s': float(stamps[-1]),
                'stable_sample_count': int(len(window)),
                'max_joint_span_deg': float(np.max(np.rad2deg(np.ptp(window, axis=0)))),
                'max_target_error_deg': float(np.max(target_error_deg)),
                'max_feedback_gap_s': float(max_feedback_gap_s),
            }
            return state, window, stability
        time.sleep(.01)
    raise TimeoutError('Pose did not remain stable within the timeout')


def capture_after_stability(session, camera, target, stability, camera_latency_s,
                            post_stable_s, timeout_s, tolerance_deg):
    """Capture a new camera frame only after the stable window has ended.

    The target is resent while waiting so the SDK watchdog keeps holding the
    arm. A fresh state is read after the frame and must still be within the
    target tolerance before the pair is accepted.
    """
    deadline = time.monotonic() + timeout_s
    last_send = float('-inf')
    initial_sequence = int(camera.sequence)
    stable_end = float(stability['stable_end_feedback_monotonic_s'])
    required_exposure = stable_end + post_stable_s
    last_frame_sequence = initial_sequence
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now - last_send >= .10:
            session.send_joints(target)
            last_send = now
        received, raw = camera.read()
        sequence = int(camera.sequence)
        exposure = float(received) - camera_latency_s
        if sequence <= last_frame_sequence or exposure < required_exposure:
            time.sleep(.01)
            continue
        last_frame_sequence = sequence
        state = session.read_state(include_gripper=False)
        stamp = float(state['feedback_timestamps_monotonic_s']['joints'])
        error_deg = float(np.rad2deg(np.max(np.abs(state['joints'] - target))))
        if stamp < required_exposure or error_deg > tolerance_deg:
            time.sleep(.01)
            continue
        capture = dict(stability,
                       capture_frame_sequence=sequence,
                       frame_receive_monotonic_s=float(received),
                       estimated_exposure_monotonic_s=exposure,
                       capture_feedback_monotonic_s=stamp,
                       capture_target_error_deg=error_deg,
                       exposure_after_stable_s=exposure - stable_end,
                       feedback_after_exposure_s=stamp - exposure)
        return received, raw, state, capture
    raise TimeoutError('Stable pose was reached, but no post-stability camera/state pair was captured')


def restore_stop_when_ready(session, timeout_s=3.):
    deadline = time.monotonic() + timeout_s
    last_error = None
    while time.monotonic() < deadline:
        try:
            session.restore_electronic_stop()
            return
        except ValueError as exc:
            last_error = exc
            if 'feedback missing' not in str(exc) and 'feedback is stale' not in str(exc):
                raise
            time.sleep(.1)
    raise TimeoutError(f'Piper status feedback did not become ready: {last_error}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true', help='Required explicit motion authorization')
    parser.add_argument('--can', default='can0')
    parser.add_argument('--camera', default='/dev/video0')
    parser.add_argument('--setup', type=Path, default=ROOT.parent / 'deploy/current/piper_policy_setup.yaml')
    parser.add_argument('--config', type=Path, default=PIPER_ROOT / 'config/collection.yaml')
    parser.add_argument('--intrinsics', type=Path,
                        default=ROOT.parent / 'my_session2/calibration/gopro_intrinsics_2_7k.json')
    parser.add_argument('--targets', type=Path, required=True,
                        help='JSONL observations or JSON target file from a verified board-visible run')
    parser.add_argument('--sample-ids', type=int, nargs='*', default=None)
    parser.add_argument('--max-poses', type=int, default=20)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--camera-size', type=int, nargs=2, default=[1920, 1080], metavar=('WIDTH', 'HEIGHT'))
    parser.add_argument('--camera-fps', type=float, default=30.)
    parser.add_argument('--speed-percent', type=int, default=10)
    parser.add_argument('--move-timeout-s', type=float, default=20.)
    parser.add_argument('--stable-s', type=float, default=1.)
    parser.add_argument('--stable-deg', type=float, default=.1)
    parser.add_argument('--tolerance-deg', type=float, default=1.)
    parser.add_argument('--post-stable-s', type=float, default=.15,
                        help='Additional hold after the stable window before capture')
    parser.add_argument('--max-feedback-gap-s', type=float, default=.10,
                        help='Maximum joint-feedback packet gap during settling')
    parser.add_argument('--capture-timeout-s', type=float, default=3.,
                        help='Timeout for the post-stability camera/state pair')
    args = parser.parse_args()
    if not args.execute:
        parser.error('Automatic motion requires the explicit --execute flag')
    if min(args.camera_size) <= 0 or args.camera_fps <= 0:
        parser.error('Camera size and FPS must be positive')
    if min(args.speed_percent, args.move_timeout_s, args.stable_s,
           args.stable_deg, args.tolerance_deg, args.post_stable_s,
           args.max_feedback_gap_s, args.capture_timeout_s) <= 0:
        parser.error('Motion and stability parameters must be positive')
    args.output.mkdir(parents=True, exist_ok=True)
    targets = load_targets(args.targets, args.sample_ids, args.max_poses)
    setup = yaml.safe_load(args.setup.read_text())
    geometry = setup.get('camera_image_geometry', 'native_4_3')
    camera_latency = float(setup.get('latency_compensation', {}).get(
        'camera_observation_latency_s', .0))
    detector_args = argparse.Namespace(
        intrinsics=args.intrinsics, squares_x=5, squares_y=7, square_mm=50., marker_mm=37.5,
        dictionary='DICT_4X4_50', legacy_pattern=False)

    sys.path.insert(0, str(PIPER_ROOT))
    from data_collect.common.config import load_config
    from data_collect.cli.umi_piper_sdk import build_guard, connect_arm
    from data_collect.control.piper_sdk_session import PiperSdkSession

    arm = session = camera = None
    observations_path = args.output / 'observations.jsonl'
    try:
        arm, grip, firmware = connect_arm(args.can)
        cfg = load_config(str(args.config))
        guard, limits = build_guard(cfg, setup, arm)
        lower = np.array([limits[f'joint{i}'][0] for i in range(1, 7)])
        upper = np.array([limits[f'joint{i}'][1] for i in range(1, 7)])
        for row in targets:
            q = np.asarray(row['target_joint_rad'])
            if np.any(q < lower) or np.any(q > upper):
                raise ValueError(f"Target {row['source_sample_id']} is outside model/controller limits")
        session = PiperSdkSession(
            arm, grip, command_timeout=cfg['command_timeout_s'],
            gripper_max_m=float(setup['gripper']['nominal_training_range_m'][1])
                         - float(setup['gripper']['feedback_to_fingertip_offset_m']),
            gripper_feedback_floor_m=float(setup['gripper'].get('feedback_raw_floor_m', -.010)))
        session.configure_joint_limits(limits)
        # Verify the camera stream before enabling or moving the arm.
        camera = LatestCamera(args.camera, args.camera_size, args.camera_fps)
        while camera.sequence < 30:
            if camera.error:
                raise camera.error
            time.sleep(.02)
        detector = make_detector(detector_args, restore_camera_content(
            camera.read()[1], geometry))
        restore_stop_when_ready(session)
        arm.set_speed_percent(args.speed_percent)
        initial = session.read_state(include_gripper=False)
        guard.guard.reset(np.clip(initial['joints'], guard.guard.lower, guard.guard.upper))
        session.activate()
        metadata = {
            'mode': 'automatic_motion_hand_eye_record', 'firmware': firmware,
            'can': args.can, 'camera': args.camera, 'camera_actual_format': camera.actual_format,
            'camera_image_geometry': geometry, 'camera_size_requested': args.camera_size,
            'camera_fps_requested': args.camera_fps, 'camera_observation_latency_s': camera_latency,
            'camera_observation_latency_status': 'used_for_post_stability_exposure_estimate_only',
            'stable_s': args.stable_s, 'stable_deg': args.stable_deg,
            'tolerance_deg': args.tolerance_deg, 'post_stable_s': args.post_stable_s,
            'max_feedback_gap_s': args.max_feedback_gap_s,
            'capture_timeout_s': args.capture_timeout_s,
            'speed_percent': args.speed_percent,
            'targets': targets, 'arm_command_mode': 'execute',
            'target_source': str(args.targets.resolve()),
        }
        write_json(args.output / 'metadata.json', metadata)
        if observations_path.exists():
            observations_path.unlink()
        print(f'Automatic hand-eye run: {len(targets)} poses, speed={args.speed_percent}%.', flush=True)
        for index, target_row in enumerate(targets, 1):
            target = np.asarray(target_row['target_joint_rad'], dtype=float)
            print(f'Pose {index}/{len(targets)} target_deg={np.round(np.rad2deg(target), 2).tolist()}', flush=True)
            session.send_joints(target)
            state, stable_joints, stability = wait_for_stability(
                session, target, args.stable_s, args.stable_deg,
                args.move_timeout_s, args.tolerance_deg, args.max_feedback_gap_s)
            print(f'Pose {index}: stable for {args.stable_s:g}s; '
                  f'feedback_samples={stability["stable_sample_count"]}, '
                  f'max_span={stability["max_joint_span_deg"]:.3f}deg; '
                  'capturing after post-stability hold.', flush=True)
            received, raw, state, capture = capture_after_stability(
                session, camera, target, stability, camera_latency,
                args.post_stable_s, args.capture_timeout_s, args.tolerance_deg)
            print(f'Pose {index}: paired frame sequence={capture["capture_frame_sequence"]}, '
                  f'exposure_after_stable={capture["exposure_after_stable_s"]:.3f}s, '
                  f'feedback_error={capture["capture_target_error_deg"]:.3f}deg.', flush=True)
            restored = restore_camera_content(raw, geometry)
            detection, annotated = detector.detect(restored)
            if not detection['pose_valid']:
                raise RuntimeError(f'Pose {index} reached but board pose was not visible '
                                   f'(corners={detection["charuco_corner_count"]})')
            base_flange = np.eye(4)
            from umi.real_world.piper_hand_eye import sdk_flange_to_matrix
            base_flange = sdk_flange_to_matrix(state['flange'])
            name = f'pose_{index:04d}'
            for suffix, frame in [('raw', raw), ('restored', restored), ('detection', annotated)]:
                if not cv2.imwrite(str(args.output / f'{name}_{suffix}.png'), frame):
                    raise OSError(f'Cannot write {name}_{suffix}.png')
            stable = dict(capture,
                          joint_span_deg=np.rad2deg(np.ptp(stable_joints, axis=0)).tolist(),
                          stable_duration_s=args.stable_s)
            observation = dict(detection, sample_id=index,
                               source_sample_id=target_row['source_sample_id'],
                               raw_image=f'{name}_raw.png', restored_image=f'{name}_restored.png',
                               frame_receive_monotonic_s=received,
                               estimated_exposure_monotonic_s=capture['estimated_exposure_monotonic_s'],
                               T_base_flange=base_flange.tolist(),
                               sample_joint_rad=state['joints'].tolist(),
                               target_joint_rad=target.tolist(), stationary_window=stable,
                               pairing_mode='post_stability_fresh_frame_and_feedback',
                               capture_settings=metadata)
            with observations_path.open('a') as file:
                file.write(json.dumps(observation, allow_nan=False) + '\n')
            print(f"Saved pose {index}: corners={detection['charuco_corner_count']} "
                  f"reprojection={detection['reprojection_rms_px']:.3f}px", flush=True)
    except KeyboardInterrupt:
        print('Interrupted by operator; requesting position hold.', flush=True)
        raise
    finally:
        if camera is not None:
            camera.close()
        if session is not None:
            session.close()
        elif arm is not None:
            arm.disconnect()
    print(f'Saved automatic hand-eye run to {args.output.resolve()}.', flush=True)


if __name__ == '__main__':
    main()
