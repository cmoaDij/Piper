#!/usr/bin/env python3
"""Record Piper CAN feedback only. This program never sends CAN frames."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import socket
import struct
import time


def decode_frame(can_id, data):
    if len(data) != 8:
        raise ValueError('Expected an 8-byte CAN payload')
    if can_id == 0x2A1:
        return {'control_mode': data[0], 'arm_status': data[1], 'motion_mode': data[2],
                'teach_status': data[3], 'motion_status': data[4],
                'trajectory_number': data[5], 'error_code': int.from_bytes(data[6:8], 'big')}
    if can_id in (0x2A5, 0x2A6, 0x2A7):
        return [value / 1000 for value in struct.unpack('>ii', data)]
    if can_id == 0x2A8:
        return {'width_mm': int.from_bytes(data[:4], 'big', signed=True) / 1000,
                'force_feedback_n': int.from_bytes(data[4:6], 'big', signed=True) / 1000,
                'status_byte': data[6], 'width_mode': data[7] == 0,
                'enabled': bool(data[6] & 0x40)}
    if 0x261 <= can_id <= 0x266:
        flags = data[5]
        return {'voltage_v': int.from_bytes(data[:2], 'big') / 10,
                'driver_temp_c': int.from_bytes(data[2:4], 'big', signed=True),
                'motor_temp_c': int.from_bytes(data[4:5], 'big', signed=True),
                'status_byte': flags, 'enabled': bool(flags & 0x40),
                'low_voltage': bool(flags & 0x01), 'motor_overheating': bool(flags & 0x02),
                'overcurrent': bool(flags & 0x04), 'driver_overheating': bool(flags & 0x08),
                'collision': bool(flags & 0x10), 'driver_error': bool(flags & 0x20),
                'stall': bool(flags & 0x80)}
    return None


def capture(interface, duration):
    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.bind((interface,))
    sock.settimeout(.2)
    latest, samples, counts = {}, [], {}
    start = time.monotonic()
    next_sample = start
    started = datetime.now(timezone.utc).isoformat()
    try:
        while time.monotonic() - start < duration:
            try:
                frame = sock.recv(16)
            except socket.timeout:
                continue
            if len(frame) != 16:
                continue
            can_id, length, payload = struct.unpack('=IB3x8s', frame)
            if can_id & 0xE0000000 or length != 8:
                continue
            value = decode_frame(can_id, payload)
            if value is None:
                continue
            now = time.monotonic()
            latest[can_id] = (now, value)
            counts[can_id] = counts.get(can_id, 0) + 1
            if now >= next_sample:
                record = {'elapsed_s': round(now - start, 3)}
                ids = (0x2A5, 0x2A6, 0x2A7)
                if all(i in latest and now - latest[i][0] <= .25 for i in ids):
                    record['joint_degrees'] = sum((latest[i][1] for i in ids), [])
                if 0x2A1 in latest and now - latest[0x2A1][0] <= .25:
                    record['arm_status'] = latest[0x2A1][1]
                if 0x2A8 in latest and now - latest[0x2A8][0] <= .25:
                    record['gripper'] = latest[0x2A8][1]
                samples.append(record)
                next_sample = now + .1
    finally:
        sock.close()
    end = time.monotonic()
    joints = [row['joint_degrees'] for row in samples if 'joint_degrees' in row]
    ranges = ([round(max(row[i] for row in joints) - min(row[i] for row in joints), 3)
               for i in range(6)] if joints else None)
    drivers = {str(i - 0x260): latest[i][1] for i in range(0x261, 0x267)
               if i in latest and end - latest[i][0] <= .25}
    fresh_joints = all(i in latest and end - latest[i][0] <= .25 for i in (0x2A5, 0x2A6, 0x2A7))
    fresh_status = 0x2A1 in latest and end - latest[0x2A1][0] <= .25
    fresh_gripper = 0x2A8 in latest and end - latest[0x2A8][0] <= .25
    return {'mode': 'passive CAN receive only; no commands sent', 'interface': interface,
            'started_utc': started, 'duration_s': round(end - start, 3),
            'frame_counts': {hex(i): count for i, count in sorted(counts.items())},
            'arm_status': latest[0x2A1][1] if fresh_status else None,
            'joint_first_degrees': joints[0] if joints else None,
            'joint_last_degrees': (sum((latest[i][1] for i in (0x2A5, 0x2A6, 0x2A7)), [])
                                   if fresh_joints else None),
            'joint_range_degrees': ranges, 'drivers': drivers,
            'gripper': latest[0x2A8][1] if fresh_gripper else None, 'samples': samples}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--can', default='can0')
    parser.add_argument('--duration', type=float, default=2.0)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not 0 < args.duration <= 60:
        parser.error('--duration must be in (0, 60] seconds')
    result = capture(args.can, args.duration)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n')
    print('只读 CAN 诊断，未发送控制命令。')
    print('机械臂状态:', result['arm_status'])
    print('关节角度（度）:', result['joint_last_degrees'])
    print('采样期间关节变化（度）:', result['joint_range_degrees'])
    print('驱动器:', result['drivers'])
    print('结果:', args.output)
    if not result['joint_last_degrees'] or not result['arm_status'] or len(result['drivers']) != 6:
        raise SystemExit('CAN 反馈不完整，不能作为设备就绪的依据。')


if __name__ == '__main__':
    main()
