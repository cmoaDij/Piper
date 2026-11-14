#!/usr/bin/env python3
"""Record Piper joint CAN feedback timestamps without transmitting commands.

The timestamps are the host monotonic time after recv(), not motor sampling
times or kernel hardware timestamps.  This process never enables the robot.
"""
import argparse
import csv
import json
from pathlib import Path
import signal
import socket
import struct
import time


def decode_joint_frame(frame):
    if len(frame) != 16:
        return None
    can_id, length, payload = struct.unpack('=IB3x8s', frame)
    if can_id not in (0x2a5, 0x2a6, 0x2a7) or length != 8:
        return None
    first, second = struct.unpack('>ii', payload)
    return can_id, first / 1000., second / 1000.


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--can', default='can0')
    parser.add_argument('--duration', type=float, default=30.)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.duration <= 0:
        parser.error('--duration must be positive')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    done = False

    def stop(signum, frame):
        nonlocal done
        done = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    try:
        sock.bind((args.can,))
        sock.settimeout(.1)
        start_wall, start_mono = time.time(), time.monotonic()
        metadata = {'can_interface': args.can, 'started_wall_s': start_wall,
                    'started_monotonic_s': start_mono, 'duration_requested_s': args.duration,
                    'note': 'Times are host monotonic immediately after socket recv; IDs 0x2a5/6/7 '
                            'report joints 1-2/3-4/5-6 in degrees. No commands are sent.'}
        args.output.with_suffix(args.output.suffix + '.metadata.json').write_text(json.dumps(metadata, indent=2) + '\n')
        count = 0
        with args.output.open('w', newline='', buffering=1) as file:
            writer = csv.writer(file)
            writer.writerow(['host_recv_monotonic_s', 'can_id', 'joint_a_deg', 'joint_b_deg'])
            while not done and time.monotonic() - start_mono < args.duration:
                try:
                    frame = sock.recv(16)
                except socket.timeout:
                    continue
                timestamp = time.monotonic()
                decoded = decode_joint_frame(frame)
                if decoded is None:
                    continue
                can_id, a, b = decoded
                writer.writerow([f'{timestamp:.9f}', hex(can_id), f'{a:.3f}', f'{b:.3f}'])
                count += 1
        print(f'Saved {count} joint-pair feedback frames to {args.output}; no commands sent.')
    finally:
        sock.close()


if __name__ == '__main__':
    main()
