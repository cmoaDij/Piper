#!/usr/bin/env python3
"""Compare candidate policies on recorded predictions without robot control.

All strategies are open-loop transformations of the same closed-loop log.
They cannot predict the behavior of the robot under a different strategy.
"""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation, Slerp


PERIOD_S = .05005


def smooth(p, r, w, times, tau):
    out_p, out_r, out_w = [p[0]], [r[0]], [w[0]]
    for i in range(1, len(times)):
        alpha = -np.expm1(-(times[i] - times[i-1]) / tau)
        out_p.append(out_p[-1] + alpha * (p[i] - out_p[-1]))
        delta = (out_r[-1].inv() * r[i]).as_rotvec()
        out_r.append(out_r[-1] * Rotation.from_rotvec(alpha * delta))
        out_w.append(out_w[-1] + alpha * (w[i] - out_w[-1]))
    return np.array(out_p), Rotation.concatenate(out_r), np.array(out_w)


def steps(p, r, w):
    pos = np.linalg.norm(np.diff(p, axis=0), axis=1) * 1000
    angle = (r[:-1].inv() * r[1:]).magnitude() * 180 / np.pi
    width = np.abs(np.diff(w)) * 1000
    return pos, angle, width


def stats(values):
    return {name: float(x) for name, x in zip(('median', 'p95', 'max'),
                                               np.percentile(values, [50, 95, 100]))}


def compare(rows, tcp_z=.195, tau=.15):
    rows = [row for row in rows if row.get('first_future_action_index') is not None]
    if len(rows) < 3:
        raise ValueError('Need at least three timestamped prediction packets')
    times = np.array([row['camera_times_monotonic_s'][-1] for row in rows])
    if not np.all(np.diff(times) > 0):
        raise ValueError('Camera timestamps must be strictly increasing')
    future = np.array([row['first_future_action_index'] for row in rows], dtype=int)
    f = np.array([row['predicted_flange_xyz_rpy'] for row in rows], dtype=float)
    widths = np.array([row['predicted_physical_width_m'] for row in rows], dtype=float)
    if f.shape != (len(rows), 16, 6) or widths.shape != (len(rows), 16):
        raise ValueError('Expected 16 flange and gripper predictions per packet')
    if not np.isfinite(f).all() or not np.isfinite(widths).all() or np.any((future < 0) | (future >= 16)):
        raise ValueError('Invalid prediction values or future indices')
    rot = Rotation.from_euler('xyz', f[:, :, 3:].reshape(-1, 3)).as_quat().reshape(-1, 16, 4)
    pos = f[:, :, :3] + Rotation.from_quat(rot.reshape(-1, 4)).apply([0, 0, tcp_z]).reshape(-1, 16, 3)
    ids = np.arange(len(rows))
    raw = (pos[:, 1], Rotation.from_quat(rot[:, 1]), widths[:, 1])
    next_target = (pos[ids, future], Rotation.from_quat(rot[ids, future]), widths[ids, future])
    filtered = smooth(*raw, times, tau)

    # Combine at the same absolute future time, using up to 3 past predictions.
    ensemble_p, ensemble_r, ensemble_w = [], [], []
    used = []
    for i, index in enumerate(future):
        target_time = times[i] + index * PERIOD_S
        ps, rs, ws, weights = [], [], [], []
        for j in range(max(0, i-2), i+1):
            grid = times[j] + np.arange(16) * PERIOD_S
            if not grid[0] <= target_time <= grid[-1]:
                continue
            ps.append([np.interp(target_time, grid, pos[j, :, k]) for k in range(3)])
            rs.append(Slerp(grid, Rotation.from_quat(rot[j]))([target_time]).as_quat()[0])
            ws.append(np.interp(target_time, grid, widths[j]))
            weights.append(np.exp(-(times[i] - times[j]) / tau))
        weights = np.asarray(weights) / np.sum(weights)
        ensemble_p.append(np.average(ps, axis=0, weights=weights))
        ensemble_r.append(Rotation.from_quat(rs).mean(weights=weights).as_quat())
        ensemble_w.append(np.average(ws, weights=weights))
        used.append(len(weights))
    ensemble = (np.asarray(ensemble_p), Rotation.from_quat(ensemble_r), np.asarray(ensemble_w))
    strategies = {'direct_action_1': raw, 'first_future': next_target,
                  'ema_action_1': filtered, 'future_ensemble': ensemble}
    summary = {}
    for name, (p, r, w) in strategies.items():
        delta_pos, delta_rot, delta_width = steps(p, r, w)
        summary[name] = {'position_step_mm': stats(delta_pos), 'rotation_step_deg': stats(delta_rot),
                         'gripper_step_mm': stats(delta_width),
                         'position_difference_from_direct_mm': stats(np.linalg.norm(p - raw[0], axis=1) * 1000),
                         'gripper_difference_from_direct_mm': stats(np.abs(w - raw[2]) * 1000)}
    return times - times[0], strategies, summary, used


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--tcp-z', type=float, default=.195)
    parser.add_argument('--tau', type=float, default=.15)
    args = parser.parse_args()
    if not np.isfinite(args.tcp_z) or not np.isfinite(args.tau) or args.tau <= 0:
        parser.error('TCP offset must be finite and tau must be positive')
    rows = [json.loads(line) for line in args.input.open()]
    t, strategies, summary, used = compare(rows, args.tcp_z, args.tau)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    results = {'input': str(args.input.resolve()), 'prediction_packets': len(t),
               'tcp_z_m': args.tcp_z, 'smoothing_tau_s': args.tau,
               'future_ensemble_median_chunks': float(np.median(used)), 'strategies': summary,
               'limitations': 'Fixed recorded observations, not closed-loop simulation or measured execution. '
                              'Future-index selection uses producer timestamps, without measured Piper latency.'}
    (args.output_dir / 'offline_strategy_metrics.json').write_text(json.dumps(results, indent=2) + '\n')
    names = {'direct_action_1': 'Current: action 1', 'first_future': 'First future point',
             'ema_action_1': f'Action 1 + EMA {args.tau:g} s', 'future_ensemble': 'Future: ensemble of up to 3'}
    fig, axes = plt.subplots(3, 1, figsize=(14, 9), sharex=True, constrained_layout=True)
    for name, (p, r, w) in strategies.items():
        dpos, _, dw = steps(p, r, w)
        axes[0].plot(t, p[:, 0] * 1000, label=names[name], linewidth=1)
        axes[1].plot(t[1:], dpos, label=names[name], linewidth=.85, alpha=.8)
        axes[2].plot(t[1:], dw, label=names[name], linewidth=.85, alpha=.8)
    axes[0].set_ylabel('TCP base X (mm)')
    axes[1].set_ylabel('TCP target step (mm)')
    axes[2].set_ylabel('Gripper target step (mm)')
    axes[2].set_xlabel('Time since first recorded frame (s)')
    axes[0].legend(ncol=2, fontsize=8)
    for ax in axes:
        ax.grid(alpha=.25)
    fig.suptitle('Offline replay of recorded predictions; not closed-loop execution')
    fig.savefig(args.output_dir / 'offline_strategy_comparison.png', dpi=140)
    plt.close(fig)
    print(json.dumps(results, indent=2))


if __name__ == '__main__':
    main()
