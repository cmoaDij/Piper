#!/usr/bin/env python3
"""Measure repeated policy output variation without sending robot commands."""
import argparse
from pathlib import Path
import json
import sys
import time

import numpy as np
import torch
import yaml
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from scripts.piper_live_observation import (  # noqa: E402
    CameraReader,
    PassiveCan,
    TransientObservationError,
    interpolate_feedback,
    preprocess,
    select_samples,
    tcp_pose,
)
from scripts.piper_shadow_snapshot import load_policy  # noqa: E402
from umi.real_world.piper_policy_conversion import (  # noqa: E402
    GripperPolicyLimits,
    physical_width_from_feedback,
    relative_policy_action_to_flange,
)
from umi.real_world.real_inference_util import get_real_umi_obs_dict  # noqa: E402


def stats(values):
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if values.size == 0:
        return {'count': 0, 'median': None, 'p95': None, 'max': None, 'std': None}
    return {
        'count': int(values.size),
        'median': float(np.percentile(values, 50)),
        'p95': float(np.percentile(values, 95)),
        'max': float(np.max(values)),
        'std': float(np.std(values)),
    }


def make_flange_matrix(flange):
    flange = np.asarray(flange, dtype=float)
    result = np.eye(4)
    result[:3, :3] = Rotation.from_euler('xyz', flange[3:]).as_matrix()
    result[:3, 3] = flange[:3]
    return result


def build_observation(pair, feedback, mask, gripper_limits, start_pose, shape_meta,
                      flange_tcp, camera_image_geometry='native_4_3'):
    images = np.stack([preprocess(
        item[1], mask, camera_image_geometry=camera_image_geometry)[0]
        for item in pair])
    poses = np.stack([tcp_pose(item[3], flange_tcp) for item in feedback])
    widths = [physical_width_from_feedback(item[2] / 1000, gripper_limits)[0]
              for item in feedback]
    env_obs = {
        'camera0_rgb': images,
        'robot0_eef_pos': poses[:, :3],
        'robot0_eef_rot_axis_angle': poses[:, 3:],
        'robot0_gripper_width': np.asarray(widths)[:, None],
    }
    obs = get_real_umi_obs_dict(
        env_obs,
        shape_meta,
        obs_pose_repr='relative',
        episode_start_pose=[start_pose],
    )
    return {
        key: torch.from_numpy(np.asarray(value, dtype=np.float32)).unsqueeze(0).to('cuda')
        for key, value in obs.items()
    }, poses, widths, images


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--checkpoint', type=Path, required=True)
    parser.add_argument('--setup', type=Path, required=True)
    parser.add_argument('--mask', type=Path, required=True)
    parser.add_argument('--camera', default='/dev/video0')
    parser.add_argument('--can', default='can0')
    parser.add_argument('--camera-only', action='store_true',
                        help='Use a stationary zero-pose placeholder when CAN feedback is unavailable')
    parser.add_argument('--gripper-mm', type=float, default=59.4,
                        help='Synthetic gripper width for --camera-only')
    parser.add_argument('--samples', type=int, default=40)
    parser.add_argument('--action-index', type=int, default=9)
    parser.add_argument('--seed', type=int, default=1234)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--disable-inference-augmentations', action='store_true',
                        help='Disable RandomCrop and ColorJitter during inference')
    parser.add_argument('--freeze-observation', action='store_true',
                        help='Repeat predictions on the first valid observation')
    parser.add_argument('--max-seconds', type=float, default=60.0)
    args = parser.parse_args()
    if args.samples <= 0 or not 0 <= args.action_index < 16:
        parser.error('--samples must be positive and --action-index must be in [0, 15]')
    args.output.mkdir(parents=True, exist_ok=True)

    setup = yaml.safe_load(args.setup.read_text())
    latency = setup.get('latency_compensation', {})
    camera_latency_s = float(latency.get(
        'camera_observation_latency_s', latency.get('camera_receive_latency_s', 0.0)))
    robot_observation_latency_s = float(latency.get('robot_observation_latency_s', 0.0))
    action_period_s = float(latency.get('action_period_s', 0.05005))
    gripper = setup['gripper']
    gripper_limits = GripperPolicyLimits(
        feedback_offset_m=-float(gripper['feedback_to_fingertip_offset_m']),
        training_max_m=float(gripper['nominal_training_range_m'][1]),
        controller_max_m=(float(gripper['nominal_training_range_m'][1])
                          - float(gripper['feedback_to_fingertip_offset_m'])),
    )
    tool = setup['flange_to_policy_tcp']
    flange_tcp = np.eye(4)
    flange_tcp[:3, :3] = Rotation.from_rotvec(tool['rotation']).as_matrix()
    flange_tcp[:3, 3] = np.asarray(tool['translation_m'], dtype=float)
    mask = json.loads(args.mask.read_text())

    print('Loading policy; receive-only test, no robot commands will be sent.', flush=True)
    cfg, model = load_policy(
        args.checkpoint,
        args.seed,
        disable_inference_augmentations=args.disable_inference_augmentations,
    )
    removed = list(getattr(model, '_piper_removed_inference_augmentations', ()))
    if args.disable_inference_augmentations:
        print(f'Inference augmentations disabled: {removed}', flush=True)
    else:
        print('Inference augmentations enabled', flush=True)
    print(f'freeze_observation={args.freeze_observation} action_index={args.action_index}', flush=True)
    if cfg.task.pose_repr.obs_pose_repr != 'relative' or cfg.task.pose_repr.action_pose_repr != 'relative':
        raise ValueError('Only the trained relative pose representation is supported')

    if args.camera_only and not 0 <= args.gripper_mm <= 76:
        parser.error('--gripper-mm must be between 0 and 76')
    bus = None if args.camera_only else PassiveCan(args.can)
    camera = CameraReader(args.camera)
    if bus is not None:
        bus.start()
    camera.start()
    rows = []
    skips = 0
    start_pose = None
    frozen_obs = None
    frozen_context = None
    last_frame_time = 0.0
    deadline = time.monotonic() + args.max_seconds
    try:
        while len(rows) < args.samples and time.monotonic() < deadline:
            if bus is not None and bus.error:
                raise RuntimeError(f'CAN receive failed: {bus.error}')
            if camera.error:
                raise RuntimeError(f'Camera failed: {camera.error}')
            if frozen_obs is None:
                frames = camera.recent()
                samples = [] if bus is None else bus.recent()
                if len(frames) < 4 or (bus is not None and len(samples) < 2) or frames[-1][0] <= last_frame_time:
                    time.sleep(.01)
                    continue
                now = time.monotonic()
                if bus is None:
                    latest = frames[-1]
                    previous = min(frames[:-1], key=lambda item: abs(item[0] - (latest[0] - action_period_s)))
                    pair = (previous, latest)
                    fixed_flange = np.zeros(6, dtype=float)
                    feedback = [
                        (frame[0], np.zeros(6, dtype=float), args.gripper_mm, fixed_flange.copy())
                        for frame in pair
                    ]
                else:
                    try:
                        pair, feedback = select_samples(
                            frames,
                            samples,
                            now,
                            interval_s=action_period_s,
                            camera_latency_s=camera_latency_s,
                            feedback_time_offset_s=robot_observation_latency_s,
                        )
                    except TransientObservationError:
                        skips += 1
                        last_frame_time = frames[-1][0]
                        time.sleep(.005)
                        continue
                last_frame_time = pair[-1][0]
                poses = np.stack([tcp_pose(item[3], flange_tcp) for item in feedback])
                if start_pose is None:
                    start_pose = poses[-1].copy()
                obs, poses, widths, images = build_observation(
                    pair,
                    feedback,
                    mask,
                    gripper_limits,
                    start_pose,
                    cfg.task.shape_meta,
                    flange_tcp,
                    camera_image_geometry=setup.get('camera_image_geometry', 'native_4_3'),
                )
                frozen_context = {
                    'camera_time_monotonic_s': float(pair[-1][0]),
                    'can_time_monotonic_s': float(feedback[-1][0]),
                    'joint_degrees': np.asarray(feedback[-1][1]).tolist(),
                    'flange_xyz_rpy': np.asarray(feedback[-1][3]).tolist(),
                    'tcp_xyz_rotvec': poses[-1].tolist(),
                    'gripper_width_m': float(widths[-1]),
                    'image_shape': list(images.shape),
                }
                if args.freeze_observation:
                    frozen_obs = obs
            else:
                obs = frozen_obs
                poses = None

            begin = time.monotonic()
            with torch.inference_mode():
                action = model.predict_action(obs)['action_pred'][0].cpu().numpy()
            inference_s = time.monotonic() - begin
            if action.shape != (16, 10) or not np.isfinite(action).all():
                raise RuntimeError(f'Invalid model prediction shape={action.shape}')

            relative_xyz = action[args.action_index, :3].astype(float)
            base_flange = make_flange_matrix(frozen_context['flange_xyz_rpy'])
            target_flange = relative_policy_action_to_flange(
                base_flange,
                flange_tcp,
                action[args.action_index, :9],
                tcp_verified=True,
            )
            target_tcp = tcp_pose(target_flange, flange_tcp)
            row = {
                'prediction_id': len(rows) + 1,
                'action_index': args.action_index,
                'camera_time_monotonic_s': frozen_context['camera_time_monotonic_s'],
                'can_time_monotonic_s': frozen_context['can_time_monotonic_s'],
                'inference_s': float(inference_s),
                'relative_action_xyz_m': relative_xyz.tolist(),
                'relative_action_rot6d': action[args.action_index, 3:9].astype(float).tolist(),
                'predicted_gripper_width_m': float(action[args.action_index, 9]),
                'target_flange_xyz_rpy': target_flange.tolist(),
                'target_tcp_xyz_rotvec': target_tcp.tolist(),
                'observation_frozen': bool(args.freeze_observation),
            }
            rows.append(row)
            print(
                f"sample={len(rows)}/{args.samples} "
                f"index={args.action_index} "
                f"rel_xyz_mm={np.round(relative_xyz * 1000, 2).tolist()} "
                f"inference_ms={inference_s * 1000:.1f}",
                flush=True,
            )
    finally:
        camera.close()
        if bus is not None:
            bus.close()

    xyz = np.asarray([row['relative_action_xyz_m'] for row in rows], dtype=float)
    target_tcp_xyz = np.asarray([row['target_tcp_xyz_rotvec'][:3] for row in rows], dtype=float)
    summary = {
        'status': 'completed' if len(rows) == args.samples else 'incomplete',
        'checkpoint': str(args.checkpoint.resolve()),
        'setup': str(args.setup.resolve()),
        'camera': args.camera,
        'can': args.can,
        'camera_only': args.camera_only,
        'camera_image_geometry': setup.get('camera_image_geometry', 'native_4_3'),
        'feedback_source': 'synthetic_zero_pose' if args.camera_only else 'continuous_raw_CAN',
        'synthetic_gripper_mm': args.gripper_mm if args.camera_only else None,
        'sample_count': len(rows),
        'requested_samples': args.samples,
        'action_index': args.action_index,
        'freeze_observation': args.freeze_observation,
        'inference_augmentations_disabled': args.disable_inference_augmentations,
        'removed_inference_augmentations': removed,
        'observation_skips': skips,
        'camera_latency_s': camera_latency_s,
        'robot_observation_latency_s': robot_observation_latency_s,
        'relative_action_xyz_mm': {
            'mean': (np.mean(xyz, axis=0) * 1000).tolist() if len(xyz) else None,
            'std': (np.std(xyz, axis=0) * 1000).tolist() if len(xyz) else None,
            'range': ((np.max(xyz, axis=0) - np.min(xyz, axis=0)) * 1000).tolist() if len(xyz) else None,
        },
        'relative_action_step_norm_mm': stats(
            np.linalg.norm(np.diff(xyz, axis=0), axis=1) * 1000 if len(xyz) > 1 else []),
        'base_target_tcp_step_norm_mm': stats(
            np.linalg.norm(np.diff(target_tcp_xyz, axis=0), axis=1) * 1000
            if len(target_tcp_xyz) > 1 else []),
        'inference_ms': stats([1000 * row['inference_s'] for row in rows]),
        'observation_context': frozen_context,
    }
    (args.output / 'prediction_variance.jsonl').write_text(
        ''.join(json.dumps(row, allow_nan=False) + '\n' for row in rows))
    (args.output / 'prediction_variance_summary.json').write_text(
        json.dumps(summary, indent=2, allow_nan=False) + '\n')
    print(json.dumps(summary, indent=2), flush=True)


if __name__ == '__main__':
    main()
