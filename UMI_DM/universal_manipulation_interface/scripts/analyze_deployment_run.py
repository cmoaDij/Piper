#!/usr/bin/env python3
"""Summarize a non-blocking Piper deployment recording.

This is diagnostic only.  It does not replay commands or infer task success.
"""
import argparse
from collections import Counter
import json
from pathlib import Path
import sys

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from scripts.plot_deployment_trajectory import build_data, load_tool


def read_jsonl(path):
    if not path.is_file():
        return []
    rows = []
    for line in path.read_text().splitlines():
        if line.strip():
            rows.append(json.loads(line))
    return rows


def stats(values):
    values = np.asarray([x for x in values if x is not None and np.isfinite(x)], dtype=float)
    if not len(values):
        return {'count': 0}
    return {'count': int(len(values)), 'median': float(np.percentile(values, 50)),
            'p95': float(np.percentile(values, 95)), 'max': float(np.max(values))}


def aligned_errors(expected, actual, key, *, max_feedback_gap_s=.1):
    """Compare at target times; never extrapolate or bridge a stale CAN gap."""
    if len(actual) < 2:
        return []
    times = np.asarray([row['time'] for row in actual], dtype=float)
    values = np.asarray([row[key] for row in actual], dtype=float)
    result = []
    for target in expected:
        stamp = target['time']
        if stamp < times[0] or stamp > times[-1]:
            continue
        right = min(int(np.searchsorted(times, stamp, side='right')), len(times) - 1)
        left = max(0, right - 1)
        if times[right] - times[left] > max_feedback_gap_s:
            continue
        alpha = (stamp - times[left]) / (times[right] - times[left]) if times[right] > times[left] else 0.
        observed = values[left] + alpha * (values[right] - values[left])
        result.append(float(np.linalg.norm(np.asarray(target[key]) - observed) * 1000))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input-dir', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path)
    parser.add_argument('--setup', type=Path)
    args = parser.parse_args()
    if not args.input_dir.is_dir():
        parser.error(f'Input directory does not exist: {args.input_dir}')
    output = args.output_dir or args.input_dir / 'analysis'
    output.mkdir(parents=True, exist_ok=True)
    predictions = read_jsonl(args.input_dir / 'live_observation.jsonl')
    controls = read_jsonl(args.input_dir / 'control_events.jsonl')
    skips = read_jsonl(args.input_dir / 'observation_skips.jsonl')

    returned = [x for x in controls if x.get('event') == 'sdk_calls_returned']
    failures = [x for x in controls if x.get('event') in ('prepare_failed', 'sdk_send_failed', 'dispatch_failed', 'session_error')]
    setup_path = args.setup or (args.input_dir / 'setup.yaml' if (args.input_dir / 'setup.yaml').is_file() else None)
    tool, _ = load_tool(setup_path)
    setup = yaml.safe_load(setup_path.read_text()) if setup_path is not None else {}
    observation_delay = float(setup.get('latency_compensation', {}).get('robot_observation_latency_s', 0.))
    data = build_data(args.input_dir, tool, observation_delay)
    errors = aligned_errors(data['expected_selected'], data['actual_feedback'], 'tcp')
    grip_errors = aligned_errors(data['gripper_expected'], data['gripper_actual'], 'width_m')

    sdk_durations = []
    for event in returned:
        timing = event.get('sdk_call_timing', {})
        if timing.get('arm_call_start_monotonic_s') is not None and timing.get('arm_call_return_monotonic_s') is not None:
            sdk_durations.append(1000.0 * (timing['arm_call_return_monotonic_s'] - timing['arm_call_start_monotonic_s']))
    ik_residuals = [x.get('ik_residual_position_m') for x in controls]
    ik_solvers = []
    for event in controls:
        solver = (event.get('ik_diagnostic') or {}).get('ik_solver') or {}
        if solver.get('solver'):
            ik_solvers.append(solver['solver'])
    result = {
        'input_dir': str(args.input_dir.resolve()),
        'prediction_packets': len(predictions),
        'controller_events': len(controls),
        'successful_sdk_events': len(returned),
        'gripper_sdk_events': sum(x.get('event') == 'gripper_sdk_calls_returned' for x in controls),
        'failure_events': len(failures),
        'observation_skips': len(skips),
        'inference_ms': stats([1000.0 * x.get('inference_s', float('nan')) for x in predictions]),
        'frame_age_ms': stats([1000.0 * x.get('frame_age_at_prediction_s', float('nan')) for x in predictions]),
        'sdk_call_ms': stats(sdk_durations),
        'ik_residual_mm': stats([1000.0 * x for x in ik_residuals if x is not None]),
        'ik_solver_counts': dict(Counter(ik_solvers)),
        'target_time_aligned_tcp_error_mm': stats(errors),
        'target_time_aligned_gripper_error_mm': stats(grip_errors),
        'arm_dispatch_lateness_ms': stats([1000 * row['lateness_s'] for row in data['component_dispatches']
                                          if row['component'] == 'arm']),
        'gripper_dispatch_lateness_ms': stats([1000 * row['lateness_s'] for row in data['component_dispatches']
                                              if row['component'] == 'gripper']),
        'continuous_tcp_feedback_points': len(data['actual_feedback']),
        'continuous_gripper_feedback_points': len(data['gripper_actual']),
        'feedback_source': data['actual_feedback_source'],
        'feedback_recording': data['feedback_recording'],
        'cancelled_components': sum(row.get('event') == 'component_cancelled' for row in controls),
        'latency_compensation': setup.get('latency_compensation', {}),
        'note': 'Metrics describe recorded timing and tracking; they do not establish task success or causality.'
    }
    (output / 'deployment_summary.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
