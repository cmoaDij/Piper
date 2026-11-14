#!/usr/bin/env python3
"""Live Piper/GoPro UMI predictions, without any robot commands.

The timestamps are host receive/dequeue times, not sensor exposure times.
The JSON output is diagnostic data and is not a robot-control protocol.
"""
import argparse
from collections import deque
import json
from pathlib import Path
import signal
import socket
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import cv2
import numpy as np
import torch
import yaml
from scipy.spatial.transform import Rotation, Slerp

from scripts.piper_shadow_snapshot import PassiveCan, load_policy, preprocess
from umi.common.pose_util import mat_to_pose
from umi.real_world.piper_policy_conversion import (
    GripperPolicyLimits, check_gripper_target, physical_width_from_feedback,
    relative_policy_action_to_flange,
)
from umi.real_world.diagnostic_bus import AsyncJsonlWriter, DiagnosticPublisher, DiagnosticReceiver
from umi.real_world.piper_policy_timing import latency_compensated_action_index
from umi.real_world.piper_camera_preview import CameraPreview, CameraReader
from umi.real_world.piper_training_reference import load_training_reference
from umi.real_world.real_inference_util import get_real_umi_obs_dict
from umi.real_world.piper_recording import AsyncFrameRecorder, AsyncModelInputRecorder


def tcp_pose(flange, flange_tcp):
    flange = np.asarray(flange, dtype=float)
    if flange.shape != (6,) or not np.isfinite(flange).all():
        raise ValueError('Invalid flange feedback')
    base_flange = np.eye(4)
    base_flange[:3, :3] = Rotation.from_euler('xyz', flange[3:]).as_matrix()
    base_flange[:3, 3] = flange[:3]
    return mat_to_pose(base_flange @ flange_tcp)


class TransientObservationError(ValueError):
    """A current frame cannot yet be paired with sufficiently fresh feedback."""


def interpolate_feedback(samples, timestamp, max_skew_s):
    """Interpolate a numeric CAN state at a camera timestamp.

    The legacy test/dummy samples contain only ``(time, value)`` and retain
    nearest-sample behavior.  Real Piper samples contain joint, gripper, and
    flange fields and are interpolated on the common host monotonic clock.
    """
    if not samples:
        raise TransientObservationError('Robot feedback is unavailable')
    if len(samples[0]) < 4 or not isinstance(samples[0][1], np.ndarray):
        nearest = min(samples, key=lambda item: abs(item[0] - timestamp))
        if abs(nearest[0] - timestamp) > max_skew_s:
            raise TransientObservationError('Camera and CAN receive times are too far apart')
        return nearest
    times = np.asarray([item[0] for item in samples], dtype=float)
    if timestamp < times[0] - max_skew_s or timestamp > times[-1] + max_skew_s:
        raise TransientObservationError('Camera and CAN receive times are too far apart')
    right = int(np.searchsorted(times, timestamp, side='left'))
    if right <= 0:
        return samples[0]
    if right >= len(samples):
        return samples[-1]
    left = right - 1
    t0, t1 = times[left], times[right]
    if t1 <= t0:
        return samples[left]
    alpha = float((timestamp - t0) / (t1 - t0))
    joints = (1.0 - alpha) * np.asarray(samples[left][1]) + alpha * np.asarray(samples[right][1])
    left_width_mm = float(samples[left][2])
    right_width_mm = float(samples[right][2])
    # Preserve a constant feedback value exactly, including the 76 mm limit.
    raw_mm = left_width_mm + alpha * (right_width_mm - left_width_mm)
    pose0 = np.asarray(samples[left][3], dtype=float)
    pose1 = np.asarray(samples[right][3], dtype=float)
    position = (1.0 - alpha) * pose0[:3] + alpha * pose1[:3]
    rotations = Rotation.from_euler('xyz', np.stack([pose0[3:], pose1[3:]]))
    rotation = Slerp([0.0, 1.0], rotations)([alpha])[0].as_euler('xyz')
    return (float(timestamp), joints, raw_mm, np.r_[position, rotation])


def select_samples(frames, samples, now, interval_s=.05005, max_age_s=.25,
                   max_skew_s=.025, feedback_time_offset_s=0.0,
                   camera_latency_s=0.0):
    """Match camera scene times to interpolated CAN state on the same clock.

    CAN receive time minus robot observation delay must equal camera receive
    time minus camera delay. Action tracking delay does not enter this pairing.
    """
    if len(frames) < 2 or len(samples) < 2:
        raise ValueError('Insufficient camera or CAN feedback')
    latest = frames[-1]
    if not 0 <= now - latest[0] <= max_age_s:
        raise TransientObservationError('Latest camera frame is stale')
    previous = min(frames[:-1], key=lambda item: abs(item[0] - (latest[0] - interval_s)))
    if abs((latest[0] - previous[0]) - interval_s) > max_skew_s:
        raise TransientObservationError('Camera history interval is unavailable')
    chosen = [interpolate_feedback(samples, frame[0] - camera_latency_s + feedback_time_offset_s, max_skew_s)
              for frame in (previous, latest)]
    return (previous, latest), chosen


def observation_skip_record(frames, samples, now, reason, interval_s=.05005,
                            camera_latency_s=0.0, feedback_time_offset_s=0.0):
    """Describe a failed pairing on both receive and corrected time axes."""
    latest = frames[-1]
    previous = min(frames[:-1], key=lambda item: abs(item[0] - (latest[0] - interval_s)))
    skews = [1000 * (frame[0] - min(samples, key=lambda item: abs(item[0] - frame[0]))[0])
             for frame in (previous, latest)]
    requested = [frame[0] - camera_latency_s + feedback_time_offset_s
                 for frame in (previous, latest)]
    corrected_skews = [1000 * (t - min(samples, key=lambda item: abs(item[0] - t))[0])
                       for t in requested]
    return {'reason': str(reason), 'time_monotonic_s': now,
            'frame_age_ms': 1000 * (now - latest[0]),
            'latest_can_age_ms': 1000 * (now - samples[-1][0]),
            'frame_interval_ms': 1000 * (latest[0] - previous[0]),
            'camera_minus_nearest_can_ms': skews,
            'corrected_camera_minus_nearest_can_ms': corrected_skews}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--checkpoint', type=Path, required=True)
    parser.add_argument('--mask', type=Path, required=True)
    parser.add_argument('--setup', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--camera', default='/dev/video0')
    parser.add_argument('--can', default='can0')
    parser.add_argument('--duration', type=float, default=5.0,
                        help='Run time in seconds; 0 runs until Ctrl+C')
    parser.add_argument('--seed', type=int, default=1234)
    parser.add_argument('--stream-socket', type=Path, default=None,
                        help='Optional local datagram destination for guarded controller integration')
    parser.add_argument('--preview-port', type=int, default=None,
                        help='Serve camera and policy input preview on localhost at this port')
    parser.add_argument('--diagnostic-socket', type=Path, default=None,
                        help='Optional non-blocking socket for controller diagnostics')
    parser.add_argument('--disable-inference-augmentations', action='store_true',
                        help='Disable RandomCrop and ColorJitter during inference')
    parser.add_argument('--record-dir', type=Path, default=None,
                        help='Optional directory for raw GoPro frames and exact model inputs')
    parser.add_argument('--match-dataset', type=Path, help='Optional training Zarr dataset for first-frame overlay')
    parser.add_argument('--match-episode', type=int, default=0, help='Initial reference episode index, starting at 0')
    parser.add_argument('--match-alpha', type=float, default=.5, help='Initial reference opacity, 0..1')
    args = parser.parse_args()
    if not np.isfinite(args.duration) or args.duration < 0:
        parser.error('--duration must be nonnegative')
    if args.preview_port is not None and not 1 <= args.preview_port <= 65535:
        parser.error('--preview-port must be between 1 and 65535')
    if args.match_dataset is not None and args.preview_port is None:
        parser.error('--match-dataset requires --preview-port')
    def interrupted(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupted)
    args.output.mkdir(parents=True, exist_ok=True)
    mask = json.loads(args.mask.read_text())
    setup = yaml.safe_load(args.setup.read_text())
    camera_image_geometry = setup.get('camera_image_geometry', 'native_4_3')
    print(f'Camera image geometry: {camera_image_geometry}', flush=True)
    training_reference = None
    if args.match_dataset is not None:
        training_reference = load_training_reference(args.match_dataset)
        training_reference.save(args.output, args.match_episode, args.match_alpha)
        print(f'Training reference: {len(training_reference.images_rgb)} episodes; '
              f'initial episode {args.match_episode}', flush=True)
    gripper = setup['gripper']
    latency = setup.get('latency_compensation', {})
    camera_latency_s = float(latency.get(
        'camera_observation_latency_s', latency.get('camera_receive_latency_s', 0.0)))
    robot_observation_latency_s = float(latency.get('robot_observation_latency_s', 0.0))
    robot_action_latency_s = float(latency.get(
        'robot_action_latency_s', latency.get('piper_tracking_latency_s', 0.0)))
    gripper_action_latency_s = float(latency.get('gripper_action_latency_s', robot_action_latency_s))
    lead_s = float(latency.get('additional_lead_s', 0.0))
    action_period_s = float(latency.get('action_period_s', .05005))
    if (not np.isfinite([camera_latency_s, robot_observation_latency_s,
                         robot_action_latency_s, gripper_action_latency_s, lead_s, action_period_s]).all()
            or camera_latency_s < 0 or robot_observation_latency_s < 0
            or robot_action_latency_s < 0 or gripper_action_latency_s < 0 or lead_s < 0 or action_period_s <= 0):
        raise ValueError('Invalid latency compensation configuration')
    gripper_limits = GripperPolicyLimits(
        feedback_offset_m=-float(gripper['feedback_to_fingertip_offset_m']),
        training_max_m=float(gripper['nominal_training_range_m'][1]),
        controller_max_m=(float(gripper['nominal_training_range_m'][1])
                          - float(gripper['feedback_to_fingertip_offset_m'])),
    )
    tool = setup['flange_to_policy_tcp']
    if not tool.get('rotation_verified'):
        raise ValueError('Flange-to-TCP orientation is missing')
    flange_tcp = np.eye(4)
    flange_tcp[:3, :3] = Rotation.from_rotvec(tool['rotation']).as_matrix()
    flange_tcp[:3, 3] = tool['translation_m']
    print('Loading policy; this program never sends robot commands.', flush=True)
    cfg, model = load_policy(
        args.checkpoint,
        args.seed,
        disable_inference_augmentations=args.disable_inference_augmentations)
    removed_augmentations = list(getattr(
        model, '_piper_removed_inference_augmentations', ()))
    if args.disable_inference_augmentations:
        print('Inference augmentations disabled: '
              f'{removed_augmentations}', flush=True)
    if cfg.task.pose_repr.obs_pose_repr != 'relative' or cfg.task.pose_repr.action_pose_repr != 'relative':
        raise ValueError('Only the trained relative pose representation is supported')
    frame_recorder = None
    model_input_recorder = None
    if args.record_dir is not None:
        args.record_dir.mkdir(parents=True, exist_ok=True)
        frame_recorder = AsyncFrameRecorder(args.record_dir / 'camera')
        model_input_recorder = AsyncModelInputRecorder(args.record_dir)
        print(f'Optional recording enabled: {args.record_dir}', flush=True)
    bus = PassiveCan(args.can)
    camera = CameraReader(args.camera, recorder=frame_recorder)
    stream = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) if args.stream_socket else None
    if stream is not None:
        stream.setblocking(False)
    diagnostic_publisher = DiagnosticPublisher(args.diagnostic_socket)
    start_pose = None
    bus.start()
    camera.start()
    prediction_log = None
    skip_log = None
    preview = None
    diagnostic_receiver = None
    try:
        if args.preview_port is not None:
            preview = CameraPreview(args.preview_port, training_reference=training_reference,
                                    match_episode=args.match_episode, match_alpha=args.match_alpha)
            preview.start()
            print(f'GoPro preview: http://127.0.0.1:{preview.port}/', flush=True)
        if args.diagnostic_socket is not None:
            diagnostic_receiver = DiagnosticReceiver(
                args.diagnostic_socket,
                lambda event: preview.update_status(event) if preview is not None else None)
            diagnostic_receiver.start()
        deadline = time.monotonic() + args.duration if args.duration else float('inf')
        # Continuous runs retain all packets on disk and a bounded in-memory tail.
        records = [] if args.duration else deque(maxlen=120)
        count = 0
        skipped = 0
        last_skip_report = float('-inf')
        prediction_log = AsyncJsonlWriter(args.output / 'live_observation.jsonl')
        last_frame_time = 0.0
        while time.monotonic() < deadline:
            if bus.error:
                raise RuntimeError(f'CAN receive failed: {bus.error}')
            if camera.error:
                raise RuntimeError(f'Camera failed: {camera.error}')
            frames, samples = camera.recent(), bus.recent()
            if len(frames) < 4 or len(samples) < 2 or frames[-1][0] <= last_frame_time:
                time.sleep(.01)
                continue
            now = time.monotonic()
            try:
                pair, feedback = select_samples(
                    frames, samples, now, interval_s=action_period_s,
                    camera_latency_s=camera_latency_s,
                    feedback_time_offset_s=robot_observation_latency_s)
            except TransientObservationError as exc:
                # Do not run inference or send commands from an invalid pair.
                # Wait for another frame; controller timeouts remain in force.
                last_frame_time = frames[-1][0]
                skipped += 1
                record = observation_skip_record(
                    frames, samples, now, exc, interval_s=action_period_s,
                    camera_latency_s=camera_latency_s,
                    feedback_time_offset_s=robot_observation_latency_s)
                if skip_log is None:
                    skip_log = (args.output / 'observation_skips.jsonl').open('w', buffering=1)
                skip_log.write(json.dumps(record, allow_nan=False) + '\n')
                skip_event = {'source': 'model', 'level': 'warn', **record}
                diagnostic_publisher.send(skip_event)
                if preview is not None:
                    preview.update_status(skip_event)
                if now - last_skip_report >= 1.0:
                    print(f'Observation skipped ({skipped} total): {exc}; '
                          f"frame_age={record['frame_age_ms']:.1f} ms "
                          f"can_age={record['latest_can_age_ms']:.1f} ms "
                          f"frame_interval={record['frame_interval_ms']:.1f} ms "
                          f"corrected_camera_minus_can={[round(x, 1) for x in record['corrected_camera_minus_nearest_can_ms']]} ms",
                          flush=True)
                    last_skip_report = now
                time.sleep(.005)
                continue
            last_frame_time = pair[-1][0]
            images = np.stack([preprocess(
                item[1], mask, camera_image_geometry=camera_image_geometry)[0]
                for item in pair])
            if preview is not None:
                preview.update(pair[-1][1], images[-1])
            poses = np.stack([tcp_pose(item[3], flange_tcp) for item in feedback])
            if start_pose is None:
                start_pose = poses[-1].copy()
            widths = [physical_width_from_feedback(item[2] / 1000, gripper_limits)[0] for item in feedback]
            env_obs = {'camera0_rgb': images, 'robot0_eef_pos': poses[:, :3],
                       'robot0_eef_rot_axis_angle': poses[:, 3:],
                       'robot0_gripper_width': np.asarray(widths)[:, None]}
            obs = get_real_umi_obs_dict(env_obs, cfg.task.shape_meta,
                                        obs_pose_repr='relative', episode_start_pose=[start_pose])
            obs = {k: torch.from_numpy(np.asarray(v, dtype=np.float32)).unsqueeze(0).to('cuda')
                   for k, v in obs.items()}
            begin = time.monotonic()
            with torch.inference_mode():
                action = model.predict_action(obs)['action_pred'][0].cpu().numpy()
            finished = time.monotonic()
            if action.shape != (16, 10) or not np.isfinite(action).all():
                raise RuntimeError('Invalid model prediction')
            base_flange = np.eye(4)
            base_flange[:3, :3] = Rotation.from_euler('xyz', feedback[-1][3][3:]).as_matrix()
            base_flange[:3, 3] = feedback[-1][3][:3]
            targets = [relative_policy_action_to_flange(base_flange, flange_tcp, step[:9],
                                                        tcp_verified=True).tolist() for step in action]
            rejected = []
            for index, step in enumerate(action):
                try:
                    check_gripper_target(step[9], 1.0, gripper_limits)
                except ValueError as exc:
                    rejected.append({'step': index, 'reason': str(exc)})
            planned_at = time.monotonic()
            try:
                future_index = latency_compensated_action_index(
                    pair[-1][0], planned_at, len(action),
                    camera_latency_s=camera_latency_s,
                    robot_action_latency_s=robot_action_latency_s,
                    gripper_action_latency_s=gripper_action_latency_s,
                    lead_s=lead_s,
                    action_period=action_period_s)
                timing_rejection = None
            except ValueError as exc:
                future_index, timing_rejection = None, str(exc)
            record = {'camera_times_monotonic_s': [item[0] for item in pair],
                      'camera_image_geometry': camera_image_geometry,
                      'observation_times_monotonic_s': [item[0] - camera_latency_s for item in pair],
                      'can_times_monotonic_s': [item[0] for item in feedback],
                      'can_effective_times_monotonic_s': [item[0] - robot_observation_latency_s for item in feedback],
                      'inference_s': finished - begin,
                      'frame_age_at_prediction_s': finished - pair[-1][0],
                      'joint_degrees': feedback[-1][1].tolist(),
                      'flange_xyz_rpy': feedback[-1][3].tolist(),
                      'tcp_poses_xyz_rotvec': poses.tolist(),
                      'model_gripper_width_m': widths,
                      'predicted_flange_xyz_rpy': targets,
                      'predicted_physical_width_m': action[:, 9].tolist(),
                      'gripper_rejections': rejected,
                      'timing_checked_at_monotonic_s': planned_at,
                      'first_future_action_index': future_index,
                      'latency_compensation': {
                          'camera_receive_latency_s': camera_latency_s,
                          'camera_observation_latency_s': camera_latency_s,
                          'robot_observation_latency_s': robot_observation_latency_s,
                          'robot_observation_latency_status': latency.get('robot_observation_latency_status', 'unspecified'),
                          'robot_action_latency_s': robot_action_latency_s,
                          'gripper_action_latency_s': gripper_action_latency_s,
                          # Legacy key retained for old analysis files.
                          'piper_tracking_latency_s': robot_action_latency_s,
                          'additional_lead_s': lead_s,
                          'action_period_s': action_period_s,
                          'reference_scene_time_monotonic_s': pair[-1][0] - camera_latency_s,
                      },
                      'timing_rejection': timing_rejection}
            count += 1
            record['prediction_id'] = count
            if model_input_recorder is not None:
                model_input_recorder.write(count, obs, {
                    'camera_times_monotonic_s': record['camera_times_monotonic_s'],
                    'can_times_monotonic_s': record['can_times_monotonic_s'],
                    'flange_xyz_rpy': record['flange_xyz_rpy'],
                    'flange_tcp': flange_tcp.tolist(),
                    'predicted_flange_xyz_rpy': record['predicted_flange_xyz_rpy'],
                    'predicted_physical_width_m': record['predicted_physical_width_m'],
                    'first_future_action_index': record['first_future_action_index'],
                    'camera_image_geometry': camera_image_geometry,
                    'inference_augmentations_disabled': args.disable_inference_augmentations,
                })
            records.append(record)
            prediction_log.write(record)
            status_event = {
                'source': 'model', 'level': 'ok', 'prediction_id': count,
                'camera_time_monotonic_s': pair[-1][0],
                'frame_age_ms': 1000 * record['frame_age_at_prediction_s'],
                'inference_ms': 1000 * record['inference_s'],
                'selected_action_index': future_index,
                'predicted_flange_xyz_rpy': targets[future_index] if future_index is not None else None,
                'predicted_gripper_width_m': (float(action[future_index, 9])
                                              if future_index is not None else None),
            }
            diagnostic_publisher.send(status_event)
            if preview is not None:
                preview.update_status(status_event)
            if stream is not None:
                packet = {'version': 1, 'kind': 'umi_piper_prediction', **record}
                try:
                    stream.sendto(json.dumps(packet, allow_nan=False).encode('utf-8'), str(args.stream_socket))
                except (FileNotFoundError, ConnectionRefusedError):
                    print('Prediction controller socket closed; stopping inference.', flush=True)
                    break
            if count == 1 or count % 20 == 0:
                print(f"prediction={count} inference={record['inference_s']:.3f}s "
                      f"frame_age={record['frame_age_at_prediction_s']:.3f}s "
                      f"rejected_widths={len(rejected)}", flush=True)
        if not records:
            raise RuntimeError('No complete observations received')
        result = {'mode': 'receive-only diagnostic, not robot commands',
                  'camera_and_can_timestamp_note': 'host dequeue/receive times, not exposure times',
                  'camera_geometry_verified': False,
                  'camera_image_geometry': camera_image_geometry,
                  'tcp_independently_validated': False,
                  'record_dir': str(args.record_dir.resolve()) if args.record_dir else None,
                  'raw_camera_recording': bool(frame_recorder is not None),
                  'model_input_recording': bool(model_input_recorder is not None),
                  'inference_augmentations_disabled': args.disable_inference_augmentations,
                  'removed_inference_augmentations': removed_augmentations,
                  'predictions': list(records)}
        out = args.output / 'live_observation.json'
        out.write_text(json.dumps(result, indent=2) + '\n')
        print('Saved', out, flush=True)
    except KeyboardInterrupt:
        print('Inference stopped; predictions saved in', args.output / 'live_observation.jsonl', flush=True)
    finally:
        if preview is not None:
            preview.close()
        if prediction_log is not None:
            prediction_log.close()
        if skip_log is not None:
            skip_log.close()
        camera.close()
        bus.close()
        if model_input_recorder is not None:
            model_input_recorder.close()
        if frame_recorder is not None:
            frame_recorder.close()
        if stream is not None:
            stream.close()
        if diagnostic_receiver is not None:
            diagnostic_receiver.close()
        diagnostic_publisher.close()


if __name__ == '__main__':
    main()
