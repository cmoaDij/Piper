#!/usr/bin/env python3
"""Measure Piper effective tracking latency with a small automatic excitation.

Run this script from the Piper capture environment.  It sends motion only when
``--execute`` is supplied.  The default start pose is inside the Piper-X URDF
limits with margin, and the generated motion alternates small joint targets.
The output contains the exact SDK target timestamps, raw CAN feedback, and a
positive-lag cross-correlation estimate for every excited joint.  Joints are
excited one at a time so their delays are not confounded by simultaneous motion.
"""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import socket
import struct
import sys
import threading
import time

import numpy as np
import yaml

PIPER_ROOT = Path(os.environ.get('PIPER_DATA_COLLECT', '/home/l1/Piper_All/Piper/Data_Collect'))


class RawCanFeedback:
    def __init__(self, interface):
        self.interface = interface
        self.samples = []
        self.error = None
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.thread.start()

    def _run(self):
        try:
            sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            sock.bind((self.interface,))
            sock.settimeout(.1)
            pairs = {}
            stamps = {}
            while not self.stop_event.is_set():
                try:
                    frame = sock.recv(16)
                except socket.timeout:
                    continue
                if len(frame) != 16:
                    continue
                can_id, length, payload = struct.unpack('=IB3x8s', frame)
                if can_id not in (0x2a5, 0x2a6, 0x2a7) or length != 8:
                    continue
                pairs[can_id] = struct.unpack('>ii', payload)
                now = time.monotonic()
                stamps[can_id] = now
                if len(pairs) < 3 or max(now - stamps[k] for k in pairs) > .1:
                    continue
                degrees = np.array([pairs[k][j] / 1000.0
                                    for k in (0x2a5, 0x2a6, 0x2a7)
                                    for j in (0, 1)], dtype=float)
                self.samples.append((now, np.deg2rad(degrees)))
            sock.close()
        except Exception as exc:
            self.error = str(exc)

    def close(self):
        self.stop_event.set()
        self.thread.join(timeout=2)


def percentile_stats(values):
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if not len(values):
        return {'count': 0}
    return {'count': int(len(values)), 'median_ms': float(np.percentile(values, 50)),
            'p95_ms': float(np.percentile(values, 95)), 'max_ms': float(np.max(values))}


def estimate_latency(targets, feedback, resample_dt=.005):
    if len(targets) < 4 or len(feedback) < 20:
        return []
    target_t = np.asarray([x['send_start_monotonic_s'] for x in targets], dtype=float)
    target_q = np.asarray([x['target_joint_rad'] for x in targets], dtype=float)
    feedback_t = np.asarray([x[0] for x in feedback], dtype=float)
    feedback_q = np.asarray([x[1] for x in feedback], dtype=float)
    start = max(target_t[0], feedback_t[0])
    end = min(feedback_t[-1], target_t[-1] + 1.0)
    if end <= start:
        return []
    grid = np.arange(start, end, resample_dt)
    actual = np.column_stack([np.interp(grid, feedback_t, feedback_q[:, j]) for j in range(6)])
    indices = np.searchsorted(target_t, grid, side='right') - 1
    indices = np.clip(indices, 0, len(target_q) - 1)
    target = target_q[indices]
    results = []
    from scipy.signal import correlate, correlation_lags
    for joint in range(6):
        x = target[:, joint] - np.mean(target[:, joint])
        y = actual[:, joint] - np.mean(actual[:, joint])
        if np.std(x) < 1e-8 or np.std(y) < 1e-8:
            results.append({'joint': joint + 1, 'latency_ms': None, 'reason': 'insufficient excitation'})
            continue
        corr = correlate(y, x, mode='full')
        lags = correlation_lags(len(y), len(x), mode='full')
        valid = lags >= 0
        lag = int(lags[valid][np.argmax(corr[valid])])
        results.append({'joint': joint + 1, 'latency_ms': float(lag * resample_dt * 1000),
                        'target_std_deg': float(np.rad2deg(np.std(x))),
                        'actual_std_deg': float(np.rad2deg(np.std(y)))})
    return results


def wait_for_pose(session, target, timeout, tolerance_rad):
    deadline = time.monotonic() + timeout
    last_send = float('-inf')
    while time.monotonic() < deadline:
        state = session.read_state(include_gripper=False)
        if np.max(np.abs(state['joints'] - target)) <= tolerance_rad:
            return state
        if time.monotonic() - last_send >= .10:
            session.send_joints(target)
            last_send = time.monotonic()
        time.sleep(.02)
    raise TimeoutError('Start pose was not reached within the timeout')


def hold_target(session, target, duration, interval=.10):
    """Keep one target alive without adding heartbeat sends to the latency signal."""
    deadline = time.monotonic() + duration
    last_send = float('-inf')
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now - last_send >= interval:
            session.read_state(include_gripper=False)
            session.send_joints(target)
            last_send = time.monotonic()
        time.sleep(.02)


def restore_stop_when_feedback_ready(session, timeout=3.0):
    """Wait for the first fresh arm-status packet after SDK reconnect."""
    deadline = time.monotonic() + timeout
    last_error = None
    while time.monotonic() < deadline:
        try:
            return session.restore_electronic_stop()
        except ValueError as exc:
            last_error = exc
            if 'feedback missing' not in str(exc) and 'feedback is stale' not in str(exc):
                raise
            time.sleep(.10)
    raise TimeoutError(f'Piper feedback did not become ready: {last_error}')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true', help='Required: enable and move Piper')
    parser.add_argument('--setup', type=Path,
                        default=Path('/home/l1/Piper_All/UMI_DM/my_session/deployment_check/piper_policy_setup.yaml'))
    parser.add_argument('--config', type=Path, default=PIPER_ROOT / 'config/collection.yaml')
    parser.add_argument('--can', default='can0')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--start-joints-deg', nargs=6, type=float,
                        default=[0., 15., -15., 30., 0., 0.])
    parser.add_argument('--joints', nargs='+', type=int, default=[1, 2, 3, 4, 5, 6],
                        help='1-based joints to excite sequentially (default: all six)')
    parser.add_argument('--joint-index', type=int, default=None, choices=range(1, 7),
                        help='Compatibility shortcut for exciting one joint only')
    parser.add_argument('--amplitude-deg', type=float, default=3.0)
    parser.add_argument('--cycles', type=int, default=5)
    parser.add_argument('--dwell-s', type=float, default=.5)
    parser.add_argument('--speed-percent', type=int, default=10)
    parser.add_argument('--start-timeout-s', type=float, default=30.)
    args = parser.parse_args()
    if not args.execute:
        parser.error('This script sends motion only with the explicit --execute flag')
    if args.amplitude_deg <= 0 or args.cycles <= 0 or args.dwell_s <= 0:
        parser.error('amplitude, cycles, and dwell must be positive')
    if not 1 <= args.speed_percent <= 100:
        parser.error('--speed-percent must be 1..100')
    if args.joint_index is not None:
        args.joints = [args.joint_index]
    if not args.joints or any(j < 1 or j > 6 for j in args.joints) or len(set(args.joints)) != len(args.joints):
        parser.error('--joints must contain unique values from 1 to 6')
    args.output.mkdir(parents=True, exist_ok=True)
    sys.path.insert(0, str(PIPER_ROOT))
    try:
        from data_collect.common.config import load_config
        from data_collect.cli.umi_piper_sdk import build_guard, connect_arm
        from data_collect.control.piper_sdk_session import PiperSdkSession
    except ImportError as exc:
        raise RuntimeError('Run this script inside Piper Data_Collect/.runtime/capture') from exc
    setup = yaml.safe_load(args.setup.read_text())
    cfg = load_config(str(args.config))
    start_target = np.deg2rad(np.asarray(args.start_joints_deg, dtype=float))
    if start_target.shape != (6,) or not np.isfinite(start_target).all():
        parser.error('Invalid start pose')

    arm = session = feedback = None
    targets = []
    try:
        arm, grip, firmware = connect_arm(args.can)
        guard, limits = build_guard(cfg, setup, arm)
        lower = np.array([limits[f'joint{i}'][0] for i in range(1, 7)])
        upper = np.array([limits[f'joint{i}'][1] for i in range(1, 7)])
        if np.any(start_target < lower) or np.any(start_target > upper):
            raise ValueError('Requested start pose is outside the model/controller intersection')
        session = PiperSdkSession(
            arm, grip, command_timeout=cfg['command_timeout_s'],
            gripper_max_m=float(setup['gripper']['nominal_training_range_m'][1])
                         - float(setup['gripper']['feedback_to_fingertip_offset_m']),
            gripper_feedback_floor_m=float(setup['gripper'].get('feedback_raw_floor_m', -.010)))
        session.configure_joint_limits(limits)
        restore_stop_when_feedback_ready(session)
        # Check feedback before enabling or sending the start pose. Only the
        # six arm joints participate in this calibration.
        session.read_state(include_gripper=False)
        arm.set_speed_percent(args.speed_percent)
        feedback = RawCanFeedback(args.can)
        feedback.start()
        session.activate()
        session.send_joints(start_target)
        wait_for_pose(session, start_target, args.start_timeout_s, np.deg2rad(1.0))
        print('Start pose reached (deg):', np.rad2deg(start_target).round(3).tolist(), flush=True)

        sequence = [1.0, -1.0] * args.cycles
        for excited_joint in args.joints:
            joint = excited_joint - 1
            for sign in sequence:
                target = start_target.copy()
                target[joint] += np.deg2rad(args.amplitude_deg * sign)
                send_start = time.monotonic()
                session.send_joints(target)
                send_end = time.monotonic()
                targets.append({'target_id': len(targets), 'joint_index': excited_joint,
                                'phase': 'excitation',
                                'send_start_monotonic_s': send_start,
                                'send_return_monotonic_s': send_end,
                                'target_joint_rad': target.tolist(),
                                'target_joint_deg': np.rad2deg(target).tolist()})
                hold_target(session, target, args.dwell_s)

            send_start = time.monotonic()
            session.send_joints(start_target)
            targets.append({'target_id': len(targets), 'joint_index': excited_joint,
                            'phase': 'return_to_start',
                            'send_start_monotonic_s': send_start,
                            'send_return_monotonic_s': time.monotonic(),
                            'target_joint_rad': start_target.tolist(),
                            'target_joint_deg': np.rad2deg(start_target).tolist()})
            hold_target(session, start_target, args.dwell_s)
    finally:
        if feedback is not None:
            feedback.close()
        if session is not None:
            session.close()
        elif arm is not None:
            arm.disconnect()

    feedback_samples = [] if feedback is None else list(feedback.samples)
    latency = estimate_latency(targets, feedback_samples)
    with (args.output / 'targets.jsonl').open('w') as f:
        for row in targets:
            f.write(json.dumps(row) + '\n')
    with (args.output / 'feedback.csv').open('w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['host_recv_monotonic_s'] + [f'joint_{i}_rad' for i in range(1, 7)])
        for stamp, q in feedback_samples:
            writer.writerow([f'{stamp:.9f}', *[f'{x:.9f}' for x in q]])
    finite_latency = [row['latency_ms'] for row in latency
                      if row.get('latency_ms') is not None]
    result = {'firmware': firmware['software_version'], 'can': args.can,
              'start_joints_deg': np.rad2deg(start_target).tolist(),
              'excited_joints': args.joints, 'amplitude_deg': args.amplitude_deg,
              'speed_percent': args.speed_percent,
              'cycles': args.cycles, 'dwell_s': args.dwell_s,
              'target_count': len(targets), 'feedback_count': len(feedback_samples),
              'latency_estimates': latency,
              'robot_action_latency_ms': percentile_stats(finite_latency),
              'timestamps': 'host monotonic; CAN timestamps are immediately after recv()',
              'calibration_kind': 'command-to-feedback effective action delay',
              'limitations': 'Effective command-to-feedback estimate, not robot observation latency.'}
    (args.output / 'piper_latency.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2), flush=True)


if __name__ == '__main__':
    main()
