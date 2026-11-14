#!/usr/bin/env python3
"""Replay recorded UMI model inputs without camera, CAN, or robot access."""
import argparse
import json
from pathlib import Path
import sys

import numpy as np
import torch
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from scripts.piper_shadow_snapshot import load_policy  # noqa: E402
from umi.real_world.piper_policy_conversion import relative_policy_action_to_flange  # noqa: E402


def make_base_flange(pose):
    pose = np.asarray(pose, dtype=float)
    result = np.eye(4)
    result[:3, :3] = Rotation.from_euler('xyz', pose[3:]).as_matrix()
    result[:3, 3] = pose[:3]
    return result


def stats(values):
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if values.size == 0:
        return {'count': 0, 'median': None, 'p95': None, 'max': None}
    return {
        'count': int(values.size),
        'median': float(np.percentile(values, 50)),
        'p95': float(np.percentile(values, 95)),
        'max': float(np.max(values)),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--recording-dir', type=Path, required=True)
    parser.add_argument('--checkpoint', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seed', type=int, default=1234)
    parser.add_argument('--max-samples', type=int, default=None)
    parser.add_argument('--disable-inference-augmentations', action='store_true')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    manifest_path = args.recording_dir / 'model_input_manifest.jsonl'
    if not manifest_path.is_file():
        parser.error(f'Missing model input manifest: {manifest_path}')
    rows = [json.loads(line) for line in manifest_path.read_text().splitlines() if line.strip()]
    if args.max_samples is not None:
        rows = rows[:args.max_samples]
    if not rows:
        parser.error('The model input manifest is empty')

    print('Loading policy for offline replay; no camera, CAN, or robot access is used.', flush=True)
    _, model = load_policy(
        args.checkpoint,
        args.seed,
        disable_inference_augmentations=args.disable_inference_augmentations,
    )
    replay_rows = []
    position_errors = []
    selected_errors = []
    gripper_errors = []
    for index, row in enumerate(rows, 1):
        input_path = args.recording_dir / row['file']
        with np.load(input_path, allow_pickle=False) as data:
            obs = {key: torch.from_numpy(np.asarray(data[key])).to('cuda') for key in data.files}
        with torch.inference_mode():
            action = model.predict_action(obs)['action_pred'][0].cpu().numpy()
        if action.shape != (16, 10) or not np.isfinite(action).all():
            raise RuntimeError(f'Invalid replay prediction at input {row["prediction_id"]}')

        base_flange = make_base_flange(row['flange_xyz_rpy'])
        flange_tcp = np.asarray(row['flange_tcp'], dtype=float)
        replay_targets = [
            relative_policy_action_to_flange(
                base_flange,
                flange_tcp,
                step[:9],
                tcp_verified=True,
            )
            for step in action
        ]
        original_targets = np.asarray(row.get('predicted_flange_xyz_rpy'), dtype=float)
        target_error = np.linalg.norm(
            np.asarray(replay_targets)[:, :3] - original_targets[:, :3], axis=1) * 1000
        index9 = int(row.get('first_future_action_index')) if row.get('first_future_action_index') is not None else 9
        selected_error = float(target_error[index9]) if 0 <= index9 < 16 else None
        original_widths = np.asarray(row.get('predicted_physical_width_m'), dtype=float)
        width_error = np.abs(action[:, 9] - original_widths) * 1000
        position_errors.extend(target_error.tolist())
        if selected_error is not None:
            selected_errors.append(selected_error)
        gripper_errors.extend(width_error.tolist())
        replay_rows.append({
            'prediction_id': row['prediction_id'],
            'input_file': row['file'],
            'first_future_action_index': row.get('first_future_action_index'),
            'replayed_action': action.tolist(),
            'replayed_flange_xyz_rpy': np.asarray(replay_targets).tolist(),
            'original_flange_xyz_rpy': original_targets.tolist(),
            'position_error_mm_by_horizon': target_error.tolist(),
            'selected_position_error_mm': selected_error,
            'gripper_error_mm_by_horizon': width_error.tolist(),
        })
        if index == 1 or index % 20 == 0 or index == len(rows):
            print(f'Replayed {index}/{len(rows)} inputs', flush=True)

    (args.output / 'replay_predictions.jsonl').write_text(
        ''.join(json.dumps(row, allow_nan=False) + '\n' for row in replay_rows))
    summary = {
        'recording_dir': str(args.recording_dir.resolve()),
        'checkpoint': str(args.checkpoint.resolve()),
        'samples': len(replay_rows),
        'inference_augmentations_disabled': args.disable_inference_augmentations,
        'position_error_mm_all_horizons': stats(position_errors),
        'position_error_mm_selected_index': stats(selected_errors),
        'gripper_error_mm_all_horizons': stats(gripper_errors),
        'note': 'Replay compares a new stochastic policy prediction against the original recorded prediction. '
                'It does not replay robot dynamics or establish task success.',
    }
    (args.output / 'replay_summary.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps(summary, indent=2), flush=True)


if __name__ == '__main__':
    main()
