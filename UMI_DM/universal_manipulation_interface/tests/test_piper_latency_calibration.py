"""Numerical and packet regressions; never opens CAN or commands hardware."""
import csv
from pathlib import Path
import struct
import subprocess
import sys
import tempfile

import numpy as np

from umi.common.piper_latency_util import estimate_signal_delay, gripper_step_responses
from scripts.calibrate_piper_gripper_latency import decode_command, decode_gripper
from scripts.calibrate_piper_observation_latency import analyze_reference, decode_joints, read_joint_csv


def signal(t):
    return np.sin(3.1 * t) + .3 * np.sin(7.3 * t + .4) + .1 * np.cos(1.3 * t * t)


def test_delay_sign_and_reference_latency():
    t = np.arange(0, 8, .005)
    for delay in (.037, -.021):
        result = estimate_signal_delay(t, signal(t), t, 2 * signal(t - delay) + 4,
                                       min_delay_s=-.1, max_delay_s=.1, step_s=.001)
        assert abs(result['delay_ms'] - delay * 1000) <= 1.01, result
        assert not result['at_search_boundary']
    # A reference arriving 60 ms late is corrected before fitting, not added
    # to or confused with the robot's actual 37 ms observation delay.
    results = analyze_reference({1: list(zip(t + .060, signal(t)))},
                                {1: list(zip(t, signal(t - .037)))},
                                reference_latency_s=.060, max_lag_s=.1, step_s=.001,
                                clock_uncertainty_ms=.2)
    assert abs(results[0]['delay_ms'] - 37) <= 1.01


def test_unexcited_and_unrelated_time_ranges_fail():
    t = np.arange(0, 4, .005)
    for rt, rv, ft, fv in ((t, np.ones_like(t), t, np.ones_like(t)),
                            (t, signal(t), t + 1000, signal(t))):
        try:
            estimate_signal_delay(rt, rv, ft, fv)
        except ValueError:
            pass
        else:
            raise AssertionError('Missing excitation/overlap must not produce a delay')


def test_zero_order_hold_does_not_invent_command_ramps():
    target_t = np.array([0., .7, 1.8, 2.6, 3.7, 4.6, 5.3, 6.5, 7.4])
    target = np.array([.06, .02, .06, .02, .06, .02, .06, .02, .06])
    t = np.arange(0, 8, .005)
    actual = target[np.clip(np.searchsorted(target_t, t - .075, side='right') - 1, 0, len(target) - 1)]
    result = estimate_signal_delay(target_t, target, t, actual, max_delay_s=.3,
                                   step_s=.005, hold_reference=True)
    assert abs(result['delay_ms'] - 75) <= 5.01, result


def test_step_percentages_and_raw_packet_units():
    t = np.arange(0, 6, .005)
    width = .060 - .040 * (1 - np.exp(-np.maximum(t - 1.04, 0) / .1))
    after = t >= 3
    width[after] = .020 + .040 * (1 - np.exp(-np.maximum(t[after] - 3.04, 0) / .1))
    steps = gripper_step_responses(
        [{'target_id': 0, 'send_start_monotonic_s': 1., 'target_width_m': .020},
         {'target_id': 1, 'send_start_monotonic_s': 3., 'target_width_m': .060}], t, width)
    assert [row['direction'] for row in steps] == ['closing', 'opening']
    for row in steps:
        for p in (10, 50, 90):
            expected = 40 - 100 * np.log(1 - p / 100)
            assert abs(row[f't{p}_ms'] - expected) <= 5.01, row
    frame = struct.pack('=IB3x8s', 0x2A8, 8, struct.pack('>ihBB', -2300, 1000, 0x40, 0))
    decoded = decode_gripper(frame, 12.)
    assert decoded['width_m'] == -.0023 and decoded['host_recv_monotonic_s'] == 12.
    assert decoded['force_n'] == 1.
    command = struct.pack('=IB3x8s', 0x159, 8, bytes.fromhex('0000ea6003e80100'))
    decoded_command = decode_command(command, 12.)
    assert decoded_command['target_width_m'] == .060
    assert decoded_command['force_n'] == 1.0 and decoded_command['set_zero'] == 0
    mode = struct.pack('=IB3x8s', 0x151, 8, bytes.fromhex('01ff140000000000'))
    decoded_mode = decode_command(mode, 11.9)
    assert decoded_mode['control_mode'] == 1 and decoded_mode['move_mode'] == 255
    assert decoded_mode['speed_percent'] == 20
    frame = struct.pack('=IB3x8s', 0x2A5, 8, struct.pack('>ii', 3000, -2000))
    rows = decode_joints(frame, 10.)
    assert [row['joint_index'] for row in rows] == [1, 2]
    np.testing.assert_allclose([row['position_rad'] for row in rows], np.deg2rad([3, -2]))


def test_reference_csv_does_not_hide_bad_clock_order():
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / 'reference.csv'
        with path.open('w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['host_sample_monotonic_s', 'joint_index', 'position_rad'])
            writer.writerows([(2, 1, .1), (1, 1, .2)])
        try:
            read_joint_csv(path, 'host_sample_monotonic_s')
        except ValueError as exc:
            assert 'strictly increase' in str(exc)
        else:
            raise AssertionError('Time reversal must not be silently sorted away')


def test_independent_reference_cli_writes_the_expected_delay():
    import json
    root = Path(__file__).resolve().parents[1]
    with tempfile.TemporaryDirectory() as directory:
        output = Path(directory)
        t = np.arange(100, 108, .005)
        reference_path, feedback_path = output / 'reference.csv', output / 'feedback.csv'
        for path, column, values in ((reference_path, 'host_sample_monotonic_s', signal(t)),
                                      (feedback_path, 'host_recv_monotonic_s', signal(t - .037))):
            with path.open('w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([column, 'joint_index', 'position_rad'])
                writer.writerows(zip(t, np.ones(len(t), dtype=int), values))
        proc = subprocess.run([sys.executable, str(root / 'scripts/calibrate_piper_observation_latency.py'),
            'analyze', '--reference-csv', str(reference_path), '--reference-timestamp', 'sample',
            '--clock-uncertainty-ms', '.2', '--feedback-csv', str(feedback_path),
            '--max-lag-ms', '100', '--output', str(output)], capture_output=True, text=True)
        assert proc.returncode == 0, proc.stderr
        result = json.loads((output / 'robot_observation_latency.json').read_text())
        assert abs(result['robot_observation_delay_ms']['median_ms'] - 37) <= 1.01
        assert result['deployment_config_modified'] is False


if __name__ == '__main__':
    test_delay_sign_and_reference_latency()
    test_unexcited_and_unrelated_time_ranges_fail()
    test_zero_order_hold_does_not_invent_command_ramps()
    test_step_percentages_and_raw_packet_units()
    test_reference_csv_does_not_hide_bad_clock_order()
    test_independent_reference_cli_writes_the_expected_delay()
    print('Piper latency calibration checks passed (synthetic signals and packets only)')
