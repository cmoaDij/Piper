#!/usr/bin/env python3
"""Measure empty Piper gripper command-to-feedback response; requires --execute.

Only the gripper receives position targets (default 20--60 mm). CAN control is
initialized using the SDK speed/mode command; arm motors are not enabled and
no joint/pose targets are sent. The stationary arm is monitored.
CAN receive timestamps are captured in a separate, filtered SocketCAN thread.
This is an effective response estimate, not an absolute sensor delay.
"""
import argparse
import csv
import fcntl
import json
import os
from pathlib import Path
import signal
import socket
import struct
import sys
import threading
import time

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
PIPER_ROOT = Path(os.environ.get('PIPER_DATA_COLLECT', '/home/l1/Piper_All/Piper/Data_Collect'))
sys.path.insert(0, str(ROOT))
from umi.common.piper_latency_util import estimate_signal_delay, gripper_step_responses, summarize_ms


def decode_gripper(frame, received):
    if len(frame) != 16:
        return None
    can_id, length, payload = struct.unpack('=IB3x8s', frame)
    if can_id != 0x2A8 or length != 8:
        return None
    if payload[7] != 0:
        raise ValueError('Gripper reports angle mode; width mode is required')
    if payload[6] & 0x3F:
        raise ValueError(f'Gripper reports a fault: status=0x{payload[6]:02x}')
    return {'host_recv_monotonic_s': received,
            'width_m': struct.unpack('>i', payload[:4])[0] / 1e6,
            'force_n': struct.unpack('>h', payload[4:6])[0] / 1000,
            'status': payload[6]}


def decode_command(frame, received):
    if len(frame) != 16:
        return None
    can_id, length, payload = struct.unpack('=IB3x8s', frame)
    if can_id not in (0x151, 0x159) or length != 8:
        return None
    row = {'host_recv_monotonic_s': received, 'can_id': f'0x{can_id:03x}',
           'payload_hex': payload.hex(), 'note': 'Observed CAN command frame; not a controller acknowledgement'}
    if can_id == 0x159:
        row.update({'target_width_m': struct.unpack('>i', payload[:4])[0] / 1e6,
                    'force_n': struct.unpack('>h', payload[4:6])[0] / 1000,
                    'status_code': payload[6], 'set_zero': payload[7]})
    else:
        row.update({'control_mode': payload[0], 'move_mode': payload[1], 'speed_percent': payload[2]})
    return row


class GripperFeedback:
    def __init__(self, interface):
        self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        try:
            filters = b''.join(struct.pack('=II', cid, 0x7FF | socket.CAN_EFF_FLAG | socket.CAN_RTR_FLAG)
                               for cid in (0x2A8, 0x2A1, 0x151, 0x159))
            self.sock.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, filters)
            self.sock.bind((interface,))
            self.sock.settimeout(.05)
        except Exception:
            self.sock.close()
            raise
        self.samples = []
        self.commands = []
        self.controller_states = []
        self.error = None
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.thread.start()

    def _run(self):
        try:
            while not self.stop.is_set():
                try:
                    frame = self.sock.recv(16)
                    received = time.monotonic()
                except socket.timeout:
                    continue
                row = decode_gripper(frame, received)
                if row is not None:
                    self.samples.append(row)
                else:
                    command = decode_command(frame, received)
                    if command is not None:
                        self.commands.append(command)
                    elif len(frame) == 16:
                        can_id, length, payload = struct.unpack('=IB3x8s', frame)
                        if can_id == 0x2A1 and length == 8:
                            self.controller_states.append({'host_recv_monotonic_s': received,
                                'control_mode': payload[0], 'arm_status': payload[1],
                                'motion_mode_raw': payload[2], 'payload_hex': payload.hex()})
        except Exception as exc:
            self.error = exc

    def check(self, timeout_s):
        if self.error is not None:
            raise self.error
        if not self.samples or time.monotonic() - self.samples[-1]['host_recv_monotonic_s'] > timeout_s:
            raise RuntimeError('Gripper raw CAN feedback missing or stale')

    def close(self):
        self.stop.set()
        self.thread.join(timeout=1)
        self.sock.close()


def wait_for_arm_feedback(session, timeout_s=3.0):
    """Wait for SDK receive caches after reconnect; never clear an arm fault."""
    deadline = time.monotonic() + timeout_s
    while True:
        try:
            return session.read_state(include_gripper=False)
        except ValueError as exc:
            if ('feedback missing' not in str(exc) and 'feedback is stale' not in str(exc)) or time.monotonic() >= deadline:
                raise
            time.sleep(.01)


def wait_and_monitor(session, feedback, joints, arm_enabled, width, duration_s,
                     tolerance_rad, send_log, arm_log):
    deadline = time.monotonic() + duration_s
    last_send = time.monotonic()
    while time.monotonic() < deadline:
        feedback.check(session.feedback_timeout)
        state = session.read_state(include_gripper=False)
        delta = state['joints'] - joints
        arm_log.append({'host_monotonic_s': state['time'],
                        'joint_rad': state['joints'].tolist(), 'delta_rad': delta.tolist(),
                        'arm_enabled': state['arm_enabled'], 'control_mode': state['control_mode']})
        if state['control_mode'] != 1:
            raise RuntimeError(f'Controller left CAN control mode during gripper calibration: '
                               f'control_mode={state["control_mode"]}')
        if state['arm_enabled'] != arm_enabled:
            raise RuntimeError(f'Arm enable state changed during gripper calibration: '
                               f'initial={arm_enabled}, current={state["arm_enabled"]}; '
                               f'joint_delta_deg={np.rad2deg(delta).round(3).tolist()}')
        if np.max(np.abs(delta)) > tolerance_rad:
            joint = int(np.argmax(np.abs(delta)))
            raise RuntimeError(f'Arm position changed during gripper calibration: joint {joint + 1} '
                               f'delta={np.rad2deg(delta[joint]):.3f} deg, '
                               f'limit={np.rad2deg(tolerance_rad):.3f} deg; '
                               f'joint_delta_deg={np.rad2deg(delta).round(3).tolist()}, '
                               f'arm_enabled={state["arm_enabled"]}')
        if time.monotonic() - last_send >= session.command_timeout / 3:
            # The repeated target is unchanged. Only edge sends enter the
            # latency analysis; all SDK send timestamps are saved separately.
            timing = session.send_gripper_only(width)
            send_log.append({'phase': 'unchanged_target_heartbeat', 'target_width_m': width, **timing})
            last_send = time.monotonic()
        time.sleep(.01)


def analyze(targets, samples, max_lag_s, step_s):
    times = [row['host_recv_monotonic_s'] for row in samples]
    widths = [row['width_m'] for row in samples]
    steps = gripper_step_responses(targets, times, widths)
    fit = estimate_signal_delay(
        [row['send_start_monotonic_s'] for row in targets],
        [row['target_width_m'] for row in targets], times, widths,
        max_delay_s=max_lag_s, step_s=step_s, hold_reference=True)
    return {'effective_tracking': fit, 'step_responses': steps,
            'response_by_direction': {
                direction: {f't{p}_ms': summarize_ms([row.get(f't{p}_ms') for row in steps
                                                      if row.get('direction') == direction])
                            for p in (10, 50, 90)}
                for direction in ('opening', 'closing')}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--can', default='can0')
    parser.add_argument('--setup', type=Path, default=ROOT.parent / 'deploy/current/piper_policy_setup.yaml')
    parser.add_argument('--config', type=Path, default=PIPER_ROOT / 'config/collection.yaml')
    parser.add_argument('--closed-mm', type=float, default=20)
    parser.add_argument('--open-mm', type=float, default=60)
    parser.add_argument('--cycles', type=int, default=6)
    parser.add_argument('--dwell-s', type=float, default=1.5)
    parser.add_argument('--max-lag-ms', type=float, default=500)
    parser.add_argument('--lag-step-ms', type=float, default=1)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not args.execute:
        parser.error('Add --execute to move the EMPTY gripper; the arm must stay stationary')
    values = [args.closed_mm, args.open_mm, args.dwell_s, args.max_lag_ms, args.lag_step_ms]
    if (not np.isfinite(values).all() or not 0 <= args.closed_mm < args.open_mm
            or args.cycles < 2 or args.dwell_s <= 0
            or not 0 < args.lag_step_ms < args.max_lag_ms < args.dwell_s * 1000):
        parser.error('Use finite ordered widths, cycles >= 2 and 0 < lag step < max lag < dwell')
    setup = yaml.safe_load(args.setup.read_text())
    gripper_max = float(setup['gripper']['nominal_training_range_m'][1])
    if args.open_mm / 1000 > gripper_max:
        parser.error('Requested opening exceeds the configured training/controller command range')
    if float(setup['gripper']['feedback_to_fingertip_offset_m']) != 0:
        parser.error('This calibration records raw widths and requires the zero-offset configuration')
    args.output.mkdir(parents=True, exist_ok=True)
    sys.path.insert(0, str(PIPER_ROOT))
    from data_collect.common.config import load_config
    from data_collect.cli.umi_piper_sdk import connect_arm
    from data_collect.control.piper_sdk_session import PiperSdkSession

    def interrupted(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupted)
    cfg = load_config(str(args.config))
    targets, sends, samples, arm_log = [], [], [], []
    initial_arm_state = None
    initial_settle = None
    arm = session = feedback = None
    gripper_commanded = False
    disable_verified = None
    status, error, firmware = 'error', None, None
    try:
        with (Path('/tmp') / f'umi_piper_sdk_{args.can}.lock').open('a+') as lock:
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                raise RuntimeError('Another policy/calibration process owns this CAN control outlet')
            try:
                arm, grip, firmware = connect_arm(args.can, local_loopback=True)
                session = PiperSdkSession(arm, grip, command_timeout=cfg['command_timeout_s'],
                                          gripper_max_m=gripper_max)
                initial = wait_for_arm_feedback(session)
                # Read-only arm preflight. No arm activation or target sends
                # are needed for the SDK's independent gripper endpoint.
                joints = initial['joints'].copy()
                time.sleep(cfg['motion']['stationary_duration_s'])
                state = session.read_state(include_gripper=False)
                tolerance = cfg['motion']['stationary_tolerance_rad']
                if np.max(np.abs(state['joints'] - joints)) > tolerance:
                    raise RuntimeError('Arm must be stationary before calibration')
                joints = state['joints'].copy()
                initial_arm_state = {'joint_rad': joints.tolist(), 'arm_enabled': state['arm_enabled'],
                                     'control_mode': state['control_mode']}
                feedback = GripperFeedback(args.can)
                feedback.start()
                ready = time.monotonic() + 2
                while not feedback.samples and feedback.error is None and time.monotonic() < ready:
                    time.sleep(.01)
                feedback.check(session.feedback_timeout)
                session.start_gripper_control(speed_percent=setup['policy_motion']['sdk_speed_percent'])
                print(f'CAN control mode confirmed; arm motion feedback='
                      f'{session.gripper_control_setup["motion_mode_after"]}; '
                      'no arm joint/pose targets sent.', flush=True)
                # The first grip move enables its driver; it is a settling
                # phase and is excluded from latency fitting.
                opened = args.open_mm / 1000
                initial_settle = {'target_width_m': opened, 'before_width_m': feedback.samples[-1]['width_m']}
                first_sample = len(feedback.samples)
                gripper_commanded = True
                timing = session.send_gripper_only(opened)
                sends.append({'phase': 'initial_settle', 'target_width_m': opened, **timing})
                try:
                    wait_and_monitor(session, feedback, joints, initial_arm_state['arm_enabled'],
                                     opened, args.dwell_s, tolerance, sends, arm_log)
                finally:
                    recent = feedback.samples[first_sample:]
                    if recent:
                        initial_settle.update({'final_width_m': recent[-1]['width_m'],
                            'min_width_m': min(row['width_m'] for row in recent),
                            'max_width_m': max(row['width_m'] for row in recent),
                            'final_status': recent[-1]['status']})
                if abs(feedback.samples[-1]['width_m'] - opened) > cfg['motion']['gripper_open_tolerance_m']:
                    raise RuntimeError(f'Gripper did not reach initial target {opened * 1000:.1f} mm; '
                        f'actual={feedback.samples[-1]["width_m"] * 1000:.1f} mm, '
                        f'before={initial_settle["before_width_m"] * 1000:.1f} mm, '
                        f'status=0x{feedback.samples[-1]["status"]:02x}; inspect can_commands.jsonl')
                print('Read-only arm monitoring (deg):', np.rad2deg(joints).round(3).tolist(), flush=True)
                print(f'Empty gripper test: {args.closed_mm:g}--{args.open_mm:g} mm; force parameter 1 N', flush=True)
                # Unequal dwell times reduce periodic correlation ambiguity.
                for i in range(args.cycles * 2):
                    width = (args.closed_mm if i % 2 == 0 else args.open_mm) / 1000
                    timing = session.send_gripper_only(width)
                    sends.append({'phase': 'measured_step', 'target_width_m': width, **timing})
                    targets.append({'target_id': i, 'target_width_m': width,
                                    'send_start_monotonic_s': timing['gripper_call_start_monotonic_s'],
                                    'send_return_monotonic_s': timing['gripper_call_return_monotonic_s']})
                    print(f'Step {i + 1}/{args.cycles * 2}: target={width * 1000:.1f} mm', flush=True)
                    wait_and_monitor(session, feedback, joints, initial_arm_state['arm_enabled'], width,
                                     args.dwell_s * (1.0, 1.3, .9, 1.1)[i % 4], tolerance, sends, arm_log)
                status = 'completed'
            finally:
                # This is an empty-gripper test: disable prevents a remaining
                # grip target from continuing after an interruption. No zeroing.
                try:
                    if session is not None and gripper_commanded:
                        requested_at = time.monotonic()
                        for _ in range(3):
                            session.disable_gripper()
                            time.sleep(.02)
                        disable_verified = bool(feedback.samples
                            and feedback.samples[-1]['host_recv_monotonic_s'] >= requested_at
                            and not feedback.samples[-1]['status'] & 0x40)
                finally:
                    if feedback is not None:
                        feedback.close()
                        samples = list(feedback.samples)
                    if session is not None:
                        session.close()
                    elif arm is not None:
                        arm.disconnect()
    except KeyboardInterrupt:
        status, error = 'interrupted', 'Interrupted by operator; partial data retained'
    except Exception as exc:
        error = f'{type(exc).__name__}: {exc}'
    if status == 'completed' and disable_verified is not True:
        status, error = 'error', 'Gripper disable was requested but not confirmed by fresh feedback'
    result = {'status': status, 'error': error, 'can': args.can, 'firmware': firmware,
              'measurement_kind': 'gripper command-to-received-feedback effective response',
              'closed_mm': args.closed_mm, 'open_mm': args.open_mm, 'cycles': args.cycles,
              'nominal_dwell_s': args.dwell_s, 'target_count': len(targets), 'feedback_count': len(samples),
              'gripper_disable_requested': gripper_commanded,
              'gripper_disable_verified_by_feedback': disable_verified,
              'arm_command_mode': 'CAN_mode_setup_without_joint_targets', 'initial_arm_state': initial_arm_state,
              'control_setup': None if session is None else session.gripper_control_setup,
              'initial_gripper_settle': initial_settle,
              'timestamps': 'same host monotonic clock; actual SDK gripper-call start and immediate raw CAN recv time',
              'limitations': 'Includes actuator dynamics and feedback path; not absolute feedback delay. SDK force is a parameter, not independently measured contact force.',
              'deployment_config_modified': False}
    if len(samples) > 1:
        periods_ms = np.diff([row['host_recv_monotonic_s'] for row in samples]) * 1000
        result['feedback_period_ms'] = {
            'median_ms': float(np.median(periods_ms)),
            'p95_ms': float(np.percentile(periods_ms, 95)),
            'max_ms': float(periods_ms.max())}
    result['sdk_gripper_call_duration_ms'] = summarize_ms([
        (row['gripper_call_return_monotonic_s'] - row['gripper_call_start_monotonic_s']) * 1000
        for row in sends])
    if targets and samples:
        try:
            result.update(analyze(targets, samples, args.max_lag_ms / 1000, args.lag_step_ms / 1000))
        except ValueError as exc:
            result['analysis_error'] = str(exc)
    with (args.output / 'feedback.csv').open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=['host_recv_monotonic_s', 'width_m', 'force_n', 'status'])
        writer.writeheader()
        writer.writerows(samples)
    (args.output / 'targets.jsonl').write_text(''.join(json.dumps(row) + '\n' for row in targets))
    (args.output / 'sdk_calls.jsonl').write_text(''.join(json.dumps(row) + '\n' for row in sends))
    (args.output / 'arm_states.jsonl').write_text(''.join(json.dumps(row) + '\n' for row in arm_log))
    commands = [] if feedback is None else list(feedback.commands)
    (args.output / 'can_commands.jsonl').write_text(''.join(json.dumps(row) + '\n' for row in commands))
    result['observed_can_command_count'] = len(commands)
    controller_states = [] if feedback is None else list(feedback.controller_states)
    (args.output / 'controller_status.jsonl').write_text(
        ''.join(json.dumps(row) + '\n' for row in controller_states))
    (args.output / 'gripper_latency.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    brief = {k: v for k, v in result.items() if k not in ('step_responses', 'effective_tracking')}
    if 'effective_tracking' in result:
        brief['effective_tracking'] = {k: v for k, v in result['effective_tracking'].items() if k != 'curve'}
    print(json.dumps(brief, indent=2, allow_nan=False), flush=True)
    if status != 'completed' or 'analysis_error' in result:
        raise SystemExit(130 if status == 'interrupted' else 1)


if __name__ == '__main__':
    main()
