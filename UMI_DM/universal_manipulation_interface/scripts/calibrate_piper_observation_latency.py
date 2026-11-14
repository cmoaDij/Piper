#!/usr/bin/env python3
"""Record joint receive times or compare them with an INDEPENDENT reference.

This script never enables or moves Piper. Absolute observation delay cannot be
identified from command targets, CAN periods, or cached SDK timestamps alone.
The analysis requires independent physical joint angles on this host's monotonic
clock, with their timestamp meaning and synchronization uncertainty declared.
"""
import argparse
import csv
import json
from pathlib import Path
import socket
import struct
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from umi.common.piper_latency_util import estimate_signal_delay, summarize_ms


def decode_joints(frame, received):
    if len(frame) != 16:
        return []
    can_id, length, payload = struct.unpack('=IB3x8s', frame)
    if can_id not in (0x2A5, 0x2A6, 0x2A7) or length != 8:
        return []
    first = (can_id - 0x2A5) * 2 + 1
    return [{'host_recv_monotonic_s': received, 'joint_index': first + i,
             'position_rad': float(np.deg2rad(value / 1000))}
            for i, value in enumerate(struct.unpack('>ii', payload))]


def read_joint_csv(path, time_column):
    with Path(path).open(newline='') as f:
        reader = csv.DictReader(f)
        required = {time_column, 'joint_index', 'position_rad'}
        if not required.issubset(reader.fieldnames or []):
            raise ValueError(f'{path} needs CSV columns: {", ".join(sorted(required))}')
        grouped = {}
        for row in reader:
            joint = int(row['joint_index'])
            stamp, position = float(row[time_column]), float(row['position_rad'])
            if not 1 <= joint <= 6 or not np.isfinite([stamp, position]).all():
                raise ValueError(f'{path} has invalid joint data')
            grouped.setdefault(joint, []).append((stamp, position))
    # Do not silently sort bad clocks, convert epochs, or infer a time offset.
    for rows in grouped.values():
        if np.any(np.diff([row[0] for row in rows]) <= 0):
            raise ValueError(f'{path} timestamps must strictly increase for each joint')
    return grouped


def analyze_reference(reference, feedback, *, reference_latency_s, max_lag_s,
                      step_s, clock_uncertainty_ms):
    results = []
    for joint in sorted(reference.keys() & feedback.keys()):
        rt, rv = np.asarray(reference[joint], dtype=float).T
        ft, fv = np.asarray(feedback[joint], dtype=float).T
        row = {'joint': joint}
        try:
            # If reference timestamps denote receive time, remove its known
            # observation delay first. Source/sample timestamps need no shift.
            row.update(estimate_signal_delay(
                rt - reference_latency_s, rv, ft, fv,
                min_delay_s=-max_lag_s, max_delay_s=max_lag_s, step_s=step_s))
            row['reference_sample_period_ms'] = float(np.median(np.diff(rt)) * 1000)
            row['feedback_sample_period_ms'] = float(np.median(np.diff(ft)) * 1000)
            row['clock_uncertainty_ms'] = clock_uncertainty_ms
            if row['delay_ms'] < 0:
                row['note'] = 'Negative physical feedback delay: check clock alignment, reference delay and fit ambiguity'
        except ValueError as exc:
            row['delay_ms'], row['reason'] = None, str(exc)
        results.append(row)
    if not results:
        raise ValueError('Reference and feedback contain no common measured joint')
    return results


def record(args):
    if not np.isfinite(args.seconds) or args.seconds <= 0:
        raise ValueError('--seconds must be finite and positive')
    args.output.mkdir(parents=True, exist_ok=True)
    rows, interrupted = [], False
    with socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW) as sock:
        filters = b''.join(struct.pack('=II', cid, 0x7FF | socket.CAN_EFF_FLAG | socket.CAN_RTR_FLAG)
                           for cid in (0x2A5, 0x2A6, 0x2A7))
        sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, filters)
        sock.bind((args.can,))
        sock.settimeout(.05)
        started = time.monotonic()
        print(f'Read-only joint capture on {args.can}; duration {args.seconds:g} s.', flush=True)
        print('Record the independent position reference concurrently; this script sends no motion.', flush=True)
        try:
            while time.monotonic() - started < args.seconds:
                try:
                    frame = sock.recv(16)
                    received = time.monotonic()
                except socket.timeout:
                    continue
                # Each pair keeps its own actual receive time. No six-joint
                # grouping or mixed-age state interpolation is introduced.
                rows.extend(decode_joints(frame, received))
        except KeyboardInterrupt:
            interrupted = True
    output = args.output / 'joint_feedback.csv'
    with output.open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=['host_recv_monotonic_s', 'joint_index', 'position_rad'])
        writer.writeheader()
        writer.writerows(rows)
    summary = {'can': args.can, 'sample_count': len(rows), 'interrupted': interrupted,
               'timestamp_kind': 'host monotonic immediately after raw SocketCAN recv',
               'absolute_feedback_delay_measured': False,
               'clock_anchor': {'host_monotonic_s': time.monotonic(), 'wall_time_s': time.time()}}
    (args.output / 'capture.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(f'Saved {len(rows)} joint samples: {output}', flush=True)
    if not rows:
        raise RuntimeError('No joint feedback received; no delay estimate is available')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_subparsers(dest='mode', required=True)
    capture = modes.add_parser('record', help='Record raw CAN only, without enabling or moving')
    capture.add_argument('--can', default='can0')
    capture.add_argument('--seconds', type=float, default=30)
    capture.add_argument('--output', type=Path, required=True)
    analysis = modes.add_parser('analyze', help='Compare synchronized independent physical reference and CAN')
    analysis.add_argument('--reference-csv', type=Path, required=True)
    analysis.add_argument('--feedback-csv', type=Path, required=True)
    analysis.add_argument('--reference-timestamp', choices=['sample', 'receive'], required=True,
                          help='sample: physical sample time; receive: subtract separately known reference latency')
    analysis.add_argument('--reference-latency-ms', type=float, default=None)
    analysis.add_argument('--clock-uncertainty-ms', type=float, required=True,
                          help='Known reference-to-host clock synchronization uncertainty; not a fitted free offset')
    analysis.add_argument('--max-lag-ms', type=float, default=250)
    analysis.add_argument('--lag-step-ms', type=float, default=1)
    analysis.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.mode == 'record':
        record(args)
        return
    if (not np.isfinite([args.clock_uncertainty_ms, args.max_lag_ms, args.lag_step_ms]).all()
            or args.clock_uncertainty_ms < 0 or not 0 < args.lag_step_ms < args.max_lag_ms):
        parser.error('Use finite nonnegative clock uncertainty and 0 < lag step < max lag')
    if args.reference_timestamp == 'receive' and args.reference_latency_ms is None:
        parser.error('Receive-time references require --reference-latency-ms from an independent calibration')
    if args.reference_timestamp == 'sample' and args.reference_latency_ms not in (None, 0):
        parser.error('Physical sample timestamps must not have receive latency subtracted again')
    reference_latency = 0.0 if args.reference_latency_ms is None else args.reference_latency_ms
    if not np.isfinite(reference_latency) or reference_latency < 0:
        parser.error('Reference observation latency must be finite and nonnegative')
    time_column = ('host_sample_monotonic_s' if args.reference_timestamp == 'sample'
                   else 'host_recv_monotonic_s')
    reference = read_joint_csv(args.reference_csv, time_column)
    feedback = read_joint_csv(args.feedback_csv, 'host_recv_monotonic_s')
    results = analyze_reference(
        reference, feedback, reference_latency_s=reference_latency / 1000,
        max_lag_s=args.max_lag_ms / 1000, step_s=args.lag_step_ms / 1000,
        clock_uncertainty_ms=args.clock_uncertainty_ms)
    nominal = summarize_ms([row['delay_ms'] for row in results
                            if row.get('delay_ms') is not None and row['delay_ms'] >= 0
                            and not row.get('at_search_boundary')])
    summary = {'measurement_kind': 'independent physical position to CAN receive observation-delay estimate',
               'reference_csv': str(args.reference_csv.resolve()),
               'feedback_csv': str(args.feedback_csv.resolve()),
               'reference_timestamp': args.reference_timestamp,
               'reference_latency_ms': reference_latency,
               'clock_uncertainty_ms': args.clock_uncertainty_ms,
               'robot_observation_delay_ms': nominal, 'joint_estimates': results,
               'deployment_config_modified': False,
               'limitations': 'Accuracy is limited by independent-reference calibration, clock synchronization, sampling and fit identifiability. Search step is not measurement accuracy. Do not use command targets as the reference.'}
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / 'robot_observation_latency.json').write_text(json.dumps(summary, indent=2, allow_nan=False) + '\n')
    brief = {**summary, 'joint_estimates': [{k: v for k, v in row.items() if k != 'curve'} for row in results]}
    print(json.dumps(brief, indent=2, allow_nan=False), flush=True)
    if not nominal['count']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
