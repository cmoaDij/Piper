"""Fake SDK/receiver integration; run in capture Python, never opens CAN."""
import json
import os
from pathlib import Path
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from scripts import calibrate_piper_gripper_latency as calibration

sys.path.insert(0, str(calibration.PIPER_ROOT))
import data_collect.cli.umi_piper_sdk as sdk


def message(value):
    return SimpleNamespace(msg=value, hz=200, timestamp=time.time())


class FakeArm:
    OPTIONS = SimpleNamespace(PAYLOAD=SimpleNamespace(HALF='half'))

    def __init__(self, grip, mode, enabled):
        self.q = np.array([.1, .3, -.6, .2, .3, -.1])
        self.grip, self.mode, self.enabled = grip, mode, enabled
        self.targets = []
        self.disconnected = False
        self.control_mode = 0
        self.motion_mode = 1
        self.mode_requests = []

    def get_joint_angles(self):
        q = self.q.copy()
        if self.mode == 'motion' and self.grip.target < .04:
            q[1] += np.deg2rad(1.)
        return message(q.tolist())

    def get_flange_pose(self):
        return message([0.] * 6)

    def get_arm_status(self):
        control_mode = 0 if self.mode == 'control' and self.grip.target < .04 else self.control_mode
        return message(SimpleNamespace(arm_status=0, ctrl_mode=control_mode, mode_feedback=self.motion_mode,
                                       err_status=SimpleNamespace(error=False)))

    def get_joint_enable_status(self, index):
        return not self.enabled if self.mode == 'enable' and self.grip.target < .04 else self.enabled

    def get_driver_states(self, index):
        return message(SimpleNamespace(foc_status=SimpleNamespace(driver_enable_status=True)))

    def set_speed_percent(self, value):
        self.control_mode = 1
        self.motion_mode = 255
        self.mode_requests.append(value)

    def set_payload(self, value, timeout):
        raise AssertionError('Gripper calibration must not change arm payload')

    def enable(self):
        raise AssertionError('Gripper calibration must not enable the arm')

    def move_j(self, target):
        raise AssertionError('Gripper calibration must not send arm targets')

    def disconnect(self):
        self.disconnected = True


class FakeGrip:
    def __init__(self, mode=False):
        self.target = self.width = .060
        if mode == 'no_response':
            self.width = .0474
        self.mode, self.arm = mode, None
        self.enabled = False
        self.disabled = False

    def move_gripper_m(self, value, force):
        assert self.arm.control_mode == 1, 'CAN control must be initialized before gripper commands'
        assert 0 <= value <= .076 and 0 < force <= 1
        self.target, self.enabled = value, True

    def disable_gripper(self):
        self.disabled = True
        if self.mode != 'no_disable':
            self.enabled = False
        return True


class FakeFeedback:
    def __init__(self, grip, fail):
        self.grip, self.fail = grip, fail
        self.samples, self.error = [], None
        self.commands = []
        self.controller_states = []
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)

    def _run(self):
        previous = time.monotonic()
        while not self.stop.is_set():
            now = time.monotonic()
            if self.fail != 'no_response':
                self.grip.width += (self.grip.target - self.grip.width) * -np.expm1(-(now - previous) / .015)
            previous = now
            self.samples.append({'host_recv_monotonic_s': now, 'width_m': self.grip.width,
                                 'force_n': 0., 'status': 0x40 if self.grip.enabled else 0})
            if self.fail in (True, 'interrupt') and self.grip.target < .04:
                self.error = RuntimeError('Simulated receiver failure')
            time.sleep(.001)

    def start(self):
        self.thread.start()

    def check(self, timeout):
        if self.error:
            if self.fail == 'interrupt':
                raise KeyboardInterrupt
            raise self.error

    def close(self):
        self.stop.set()
        self.thread.join(timeout=1)


def run_case(fail, enabled=False):
    grip = FakeGrip(fail)
    arm = FakeArm(grip, fail, enabled)
    grip.arm = arm
    receiver = FakeFeedback(grip, fail)
    limits = {f'joint{i}': (-1., 1.) for i in range(1, 7)}
    with tempfile.TemporaryDirectory() as directory:
        argv = ['test', '--execute', '--can', f'fake_grip_{os.getpid()}', '--cycles', '2',
                '--dwell-s', '.12', '--max-lag-ms', '80', '--output', directory]
        with patch.object(sys, 'argv', argv), patch.object(calibration.signal, 'signal'), \
             patch.object(calibration, 'GripperFeedback', return_value=receiver), \
             patch.object(sdk, 'connect_arm', return_value=(arm, grip, {'software_version': 'fake'})), \
             patch.object(sdk, 'build_guard', return_value=(None, limits)):
            code = 0
            try:
                calibration.main()
            except SystemExit as exc:
                code = exc.code
        result = json.loads((Path(directory) / 'gripper_latency.json').read_text())
        assert grip.disabled and arm.disconnected
        assert not arm.targets
        assert result['arm_command_mode'] == 'CAN_mode_setup_without_joint_targets'
        assert arm.mode_requests == [20]
        assert result['control_setup']['control_mode_before'] == 0
        assert result['control_setup']['control_mode_after'] == 1
        assert result['control_setup']['motion_mode_before'] == 1
        assert result['control_setup']['motion_mode_after'] == 255
        assert result['control_setup']['confirmed']
        assert result['initial_arm_state']['arm_enabled'] == enabled
        assert (Path(directory) / 'arm_states.jsonl').stat().st_size > 0
        assert (Path(directory) / 'feedback.csv').stat().st_size > 100
        assert result['deployment_config_modified'] is False
        if fail:
            assert code == (130 if fail == 'interrupt' else 1)
            assert result['status'] == ('interrupted' if fail == 'interrupt' else 'error')
            expected = ('Interrupted by operator' if fail == 'interrupt' else
                        'joint 2 delta=1.000 deg' if fail == 'motion' else
                        'Arm enable state changed' if fail == 'enable' else
                        'left CAN control mode' if fail == 'control' else
                        'actual=47.4 mm' if fail == 'no_response' else
                        'disable was requested but not confirmed' if fail == 'no_disable' else
                        'Simulated receiver failure')
            assert expected in result['error']
            assert result['target_count'] == (0 if fail == 'no_response' else 4 if fail == 'no_disable' else 1)
        else:
            assert code == 0 and result['status'] == 'completed'
            assert result['gripper_disable_verified_by_feedback']
            assert result['target_count'] == 4 and len(result['step_responses']) == 4
            assert result['response_by_direction']['opening']['t90_ms']['count'] == 2
            assert result['response_by_direction']['closing']['t90_ms']['count'] == 2


if __name__ == '__main__':
    run_case(False)
    run_case(False, enabled=True)
    run_case(True)
    run_case('interrupt')
    run_case('motion')
    run_case('enable')
    run_case('control')
    run_case('no_response')
    run_case('no_disable')
    print('Gripper calibration fake integration passed, including partial-data recovery')
