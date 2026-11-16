#!/usr/bin/env python3
"""Evaluate every complete held-out UMI action window against recorded actions."""
import argparse
import csv
import json
import random
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import dill
import hydra
import numpy as np
import torch
from omegaconf import OmegaConf
from torch.utils.data import DataLoader
from umi.common.pose_util import rot6d_to_mat


def error_arrays(pred, target):
    position = np.linalg.norm(pred[..., :3] - target[..., :3], axis=-1) * 1000
    shape = pred.shape[:-1]
    rp = rot6d_to_mat(pred[..., 3:9].astype(np.float64).reshape(-1, 6)).reshape(*shape, 3, 3)
    rt = rot6d_to_mat(target[..., 3:9].astype(np.float64).reshape(-1, 6)).reshape(*shape, 3, 3)
    trace = np.einsum('...ij,...ij->...', rp, rt)
    rotation = np.degrees(np.arccos(np.clip((trace - 1) / 2, -1, 1)))
    width = np.abs(pred[..., 9] - target[..., 9]) * 1000
    return position, rotation, width


def stats(x):
    x = np.asarray(x, dtype=np.float64).ravel()
    if len(x) == 0 or not np.isfinite(x).all():
        raise ValueError('Empty or non-finite metric')
    return {'mean': float(x.mean()), 'median': float(np.median(x)),
            'p90': float(np.percentile(x, 90)), 'p95': float(np.percentile(x, 95)),
            'rmse': float(np.sqrt(np.mean(x*x))), 'max': float(x.max())}


def summarize(pred, target, baseline):
    result = {}
    for name, data in [('policy', pred), ('hold', baseline)]:
        pos, rot, width = error_arrays(data, target)
        result[name] = {'position_mm': stats(pos), 'rotation_deg': stats(rot),
                        'gripper_mm': stats(width),
                        'position_by_horizon_mean_mm': np.mean(pos, axis=0).tolist(),
                        'rotation_by_horizon_mean_deg': np.mean(rot, axis=0).tolist(),
                        'gripper_by_horizon_mean_mm': np.mean(width, axis=0).tolist()}
    result['predicted_gripper_outside_0_76mm_fraction'] = float(np.mean((pred[..., 9] < 0) | (pred[..., 9] > .076)))
    return result


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--checkpoint', type=Path, required=True)
    ap.add_argument('--dataset', type=Path, default=None,
                    help='Optional local dataset override for the absolute path embedded in the checkpoint')
    ap.add_argument('--manifest', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--batch-size', type=int, default=16)
    ap.add_argument('--workers', type=int, default=2)
    ap.add_argument('--seed', type=int, default=1234)
    ap.add_argument('--disable-image-augmentation', action='store_true',
                    help='Diagnostic ablation: bypass image crop, resize and color augmentation')
    ap.add_argument('--max-batches', type=int, default=None, help='Diagnostic smoke test only')
    args = ap.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    torch.manual_seed(args.seed); np.random.seed(args.seed); random.seed(args.seed)
    torch.set_num_threads(2)
    OmegaConf.register_new_resolver('eval', eval, replace=True)
    payload = torch.load(str(args.checkpoint), map_location='cpu', pickle_module=dill, mmap=True)
    epoch = dill.loads(payload['pickles']['epoch'])
    config = payload['cfg']
    expected_epochs = int(config.training.num_epochs)
    # Diffusion-policy workspaces commonly serialize the final zero-based
    # epoch (N-1) while the config stores the requested count N.
    if epoch not in (expected_epochs, expected_epochs - 1):
        raise RuntimeError(f'Checkpoint is at epoch {epoch}, expected {expected_epochs} or {expected_epochs - 1}')
    if args.dataset is not None:
        dataset_path = args.dataset.expanduser().resolve()
        if not dataset_path.is_file():
            raise FileNotFoundError(f'Offline evaluation dataset does not exist: {dataset_path}')
        OmegaConf.update(config, 'task.dataset.dataset_path', str(dataset_path), merge=False)
        print('Using local dataset override:', dataset_path, flush=True)
    config.policy.obs_encoder.pretrained = False
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print('Loading EMA policy on', device, 'checkpoint epoch', epoch, flush=True)
    policy = hydra.utils.instantiate(config.policy)
    policy.load_state_dict(payload['state_dicts']['ema_model'], strict=True)
    del payload
    if args.disable_image_augmentation:
        for key in policy.obs_encoder.rgb_keys:
            policy.obs_encoder.key_transform_map[key] = torch.nn.Identity()
    policy.eval().to(device)
    dataset = hydra.utils.instantiate(config.task.dataset).get_validation_dataset()
    indices = np.asarray(dataset.sampler.indices, dtype=np.int64)
    if len(indices) != len(dataset):
        raise RuntimeError('Sample indices and validation dataset lengths differ')
    manifest = json.loads(args.manifest.read_text())
    episode_ends = np.cumsum([e['frames'] for e in manifest['episodes']])
    episode_idx = np.searchsorted(episode_ends, indices[:, 0], side='right')
    expected = {i for i, e in enumerate(manifest['episodes']) if e['split'] == 'validation'}
    if set(episode_idx.tolist()) != expected:
        raise RuntimeError('Dataset validation split does not match the saved manifest')
    loader = DataLoader(dataset, batch_size=args.batch_size, shuffle=False,
                        num_workers=args.workers, pin_memory=device.type == 'cuda')
    pred_parts, target_parts, baseline_parts = [], [], []
    with torch.inference_mode():
        for bi, batch in enumerate(loader):
            obs = {k: v.to(device, non_blocking=True) for k, v in batch['obs'].items()}
            target = batch['action'].numpy()
            pred = policy.predict_action(obs)['action_pred'].cpu().numpy()
            if pred.shape != target.shape or not np.isfinite(pred).all():
                raise RuntimeError(f'Invalid prediction in batch {bi}')
            hold = np.zeros_like(target)
            hold[..., 3] = 1; hold[..., 7] = 1
            hold[..., 9] = batch['obs']['robot0_gripper_width'][:, -1, 0].numpy()[:, None]
            pred_parts.append(pred); target_parts.append(target); baseline_parts.append(hold)
            if (bi + 1) % 10 == 0 or bi + 1 == len(loader):
                print(f'Processed {sum(len(x) for x in pred_parts)}/{len(dataset)} windows ({bi+1}/{len(loader)} batches)', flush=True)
            if args.max_batches is not None and bi + 1 >= args.max_batches:
                break
    pred = np.concatenate(pred_parts); target = np.concatenate(target_parts); hold = np.concatenate(baseline_parts)
    episode_idx = episode_idx[:len(pred)]; indices = indices[:len(pred)]
    complete = len(pred) == len(dataset)
    if args.max_batches is None and not complete:
        raise RuntimeError('Evaluation ended before all validation windows were processed')
    result = {'complete': complete, 'checkpoint': str(args.checkpoint.resolve()),
              'checkpoint_epoch': epoch, 'model': 'EMA', 'device': str(device),
              'seed': args.seed, 'batch_size': args.batch_size, 'workers': args.workers,
              'windows': len(pred), 'total_validation_windows': len(dataset),
              'action_horizon': int(pred.shape[1]), 'action_downsample_frames': int(config.task.obs_down_sample_steps),
              'video_fps': 60000/1001, 'image_augmentation_disabled': args.disable_image_augmentation,
              'evaluation': 'one stochastic policy sample per window; episode-start noise retained',
              'all_windows_all_horizons': summarize(pred, target, hold), 'episodes': []}
    for ei in sorted(set(episode_idx.tolist())):
        mask = episode_idx == ei
        data = manifest['episodes'][ei]
        result['episodes'].append({'episode_index': ei+1, 'video': data['video'],
                                   'input_frames': data['frames'], 'windows': int(mask.sum()),
                                   'metrics': summarize(pred[mask], target[mask], hold[mask])})
    (args.output/'metrics.json').write_text(json.dumps(result, indent=2, ensure_ascii=False)+'\n')
    np.savez_compressed(args.output/'predictions.npz', prediction=pred, target=target,
                        hold_baseline=hold, episode_index=episode_idx+1, sampler_indices=indices)
    pos, rot, width = error_arrays(pred, target)
    with (args.output/'per_window.csv').open('w', newline='') as f:
        w = csv.writer(f);w.writerow(['episode_index','dataset_frame','episode_frame','position_mean_mm','position_p90_mm','rotation_mean_deg','gripper_mean_mm'])
        for i in range(len(pred)):
            w.writerow([int(episode_idx[i]+1), int(indices[i,0]), int(indices[i,0]-indices[i,1]),
                        float(pos[i].mean()),float(np.percentile(pos[i],90)),float(rot[i].mean()),float(width[i].mean())])
    print('Wrote', args.output, 'complete=',complete,flush=True)


if __name__ == '__main__':
    main()
