#!/usr/bin/env python3
"""Measure Piper CAN feedback freshness without commanding the robot.

This calibration listens directly on SocketCAN, so it does not require Piper
firmware identification or SDK motion mode. It measures host receive timing,
per-message update periods, and the time skew between the three joint feedback
messages. It cannot expose motor-sensor latency without a controller-side
timestamp.
"""
import argparse
import json
import math
from pathlib import Path
import socket
import struct
import time

import numpy as np


FEEDBACK_IDS = tuple(range(0x2A1, 0x2A9))
JOINT_IDS = (0x2A5, 0x2A6, 0x2A7)


def stats(values):
    values = np.asarray([x for x in values if math.isfinite(x)], dtype=float)
    if not len(values):
        return {'count': 0}
    return {'count': int(len(values)),
            'median_ms': float(np.percentile(values, 50)),
            'p95_ms': float(np.percentile(values, 95)),
            'max_ms': float(np.max(values)),
            'min_ms': float(np.min(values))}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--can', default='can0')
    parser.add_argument('--seconds', type=float, default=30.0)
    parser.add_argument('--sample-period-s', type=float, default=.005)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.seconds <= 0 or args.sample_period_s <= 0:
        parser.error('seconds and sample-period-s must be positive')

    try:
        sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        sock.bind((args.can,))
        sock.settimeout(.05)
    except OSError as exc:
        raise RuntimeError(f'Cannot open CAN interface {args.can}: {exc}') from exc

    started = time.monotonic()
    deadline = started + args.seconds
    next_sample = started
    last_packet = {}
    last_joint_group = None
    packet_periods = {cid: [] for cid in FEEDBACK_IDS}
    packet_counts = {f'0x{cid:03x}': 0 for cid in FEEDBACK_IDS}
    joint_group_periods = []
    joint_group_skews = []
    joint_group_ages = []
    samples = []
    try:
        while time.monotonic() < deadline:
            try:
                frame = sock.recv(16)
            except socket.timeout:
                frame = None
            if frame is not None and len(frame) == 16:
                can_id, length, _payload = struct.unpack('=IB3x8s', frame)
                if can_id in FEEDBACK_IDS and length == 8:
                    receive_time = time.monotonic()
                    key = f'0x{can_id:03x}'
                    packet_counts[key] += 1
                    previous = last_packet.get(can_id)
                    if previous is not None:
                        packet_periods[can_id].append(1000.0 * (receive_time - previous))
                    last_packet[can_id] = receive_time
                    if can_id == 0x2A7 and all(cid in last_packet for cid in JOINT_IDS):
                        joint_time = max(last_packet[cid] for cid in JOINT_IDS)
                        skew_ms = 1000.0 * (joint_time - min(last_packet[cid] for cid in JOINT_IDS))
                        joint_group_skews.append(skew_ms)
                        if last_joint_group is not None:
                            joint_group_periods.append(1000.0 * (joint_time - last_joint_group))
                        last_joint_group = joint_time

            now = time.monotonic()
            if now >= next_sample:
                latest_joint = max((last_packet.get(cid, float('-inf')) for cid in JOINT_IDS),
                                   default=float('-inf'))
                age_ms = (1000.0 * (now - latest_joint)
                          if math.isfinite(latest_joint) else float('nan'))
                if math.isfinite(age_ms):
                    joint_group_ages.append(age_ms)
                samples.append({
                    'host_monotonic_s': now,
                    'joint_feedback_age_ms': age_ms,
                    'packet_age_ms': {
                        f'0x{cid:03x}': (1000.0 * (now - last_packet[cid])
                                        if cid in last_packet else None)
                        for cid in FEEDBACK_IDS
                    },
                })
                next_sample += args.sample_period_s
    finally:
        sock.close()

    received = sum(packet_counts.values())
    if received == 0:
        output = args.output
        output.mkdir(parents=True, exist_ok=True)
        diagnostic = {
            'can': args.can,
            'duration_s': args.seconds,
            'packet_count': packet_counts,
            'error': f'No Piper feedback frames received on {args.can}',
            'next_check': 'Run ip -details link show can0 and candump -tz can0 while Piper is powered and connected.',
        }
        (output / 'piper_feedback_latency.json').write_text(json.dumps(diagnostic, indent=2) + '\n')
        raise RuntimeError(diagnostic['error'])
    output = args.output
    output.mkdir(parents=True, exist_ok=True)
    summary = {
        'can': args.can,
        'duration_s': args.seconds,
        'packet_count': packet_counts,
        'timestamp_note': 'All timestamps are host monotonic times immediately after SocketCAN recv().',
        'measurement_kind': 'feedback receive freshness and CAN timing',
        'cannot_measure': 'absolute motor-sensor latency without a Piper-side source timestamp',
        'feedback_age_ms': stats(joint_group_ages),
        'joint_group_period_ms': stats(joint_group_periods),
        'joint_group_skew_ms': stats(joint_group_skews),
        'packet_period_ms': {
            f'0x{cid:03x}': stats(packet_periods[cid]) for cid in FEEDBACK_IDS
        },
        'samples': samples,
    }
    (output / 'piper_feedback_latency.json').write_text(json.dumps(summary, indent=2) + '\n')
    print(json.dumps({key: summary[key] for key in (
        'can', 'duration_s', 'packet_count', 'feedback_age_ms',
        'joint_group_period_ms', 'joint_group_skew_ms')}, indent=2))


if __name__ == '__main__':
    main()
