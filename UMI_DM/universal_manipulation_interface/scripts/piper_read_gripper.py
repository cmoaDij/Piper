#!/usr/bin/env python3
"""Passively display Piper gripper feedback without sending CAN messages."""
import argparse
import socket
import struct
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--interface', default='can0')
    parser.add_argument('--seconds', type=float, default=60)
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error('--seconds must be positive')

    # A CAN_RAW socket can listen alongside another controller. This script
    # never writes to the socket or changes the interface state.
    can_socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    can_socket.bind((args.interface,))
    can_socket.settimeout(0.5)
    started = time.monotonic()
    next_output = started
    received = 0
    print(f'Listening on {args.interface} for {args.seconds:g} seconds; read only.', flush=True)
    try:
        while time.monotonic() - started < args.seconds:
            try:
                frame = can_socket.recv(16)
            except socket.timeout:
                continue
            if len(frame) != 16:
                continue
            can_id, length, data = struct.unpack('=IB3x8s', frame)
            if can_id != 0x2A8 or length != 8:
                continue
            received += 1
            now = time.monotonic()
            if now < next_output:
                continue
            raw_value = int.from_bytes(data[:4], byteorder='big', signed=True)
            mode = data[7]
            if mode == 0:
                value = f'feedback_width={raw_value / 1000:.3f} mm'
            elif mode == 1:
                value = f'feedback_angle={raw_value / 1000:.3f} deg'
            else:
                value = f'unknown_mode={mode} raw_value={raw_value}'
            print(f'{time.strftime("%H:%M:%S")}  {value}  status=0x{data[6]:02x}', flush=True)
            next_output = now + 1.0
    except KeyboardInterrupt:
        pass
    finally:
        can_socket.close()
    if received == 0:
        raise SystemExit('No gripper feedback received.')


if __name__ == '__main__':
    main()
