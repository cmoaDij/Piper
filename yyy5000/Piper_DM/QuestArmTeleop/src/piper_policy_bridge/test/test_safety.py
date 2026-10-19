import unittest

import numpy as np

from piper_policy_bridge.contract import JOINT_NAMES, JOINT_UPPER, cameras_ready, state_by_name
from piper_policy_bridge.safety import Deadman, SafeTargetController, TrackingError


class SafetyTest(unittest.TestCase):
    def test_both_cameras_must_be_fresh_and_aligned(self):
        arrivals = {"wrist": 1.0, "global": 1.01}
        timestamps = {"wrist": 10.0, "global": 10.02}
        self.assertTrue(cameras_ready(1.1, arrivals, timestamps, 0.15, 0.025))
        self.assertFalse(cameras_ready(1.2, arrivals, timestamps, 0.15, 0.025))
        timestamps["global"] = 10.2
        self.assertFalse(cameras_ready(1.1, arrivals, timestamps, 0.15, 0.025))
        arrivals["global"] = None
        self.assertFalse(cameras_ready(1.1, arrivals, timestamps, 0.15, 0.025))

    def test_target_is_clamped_and_step_is_capped(self):
        controller = SafeTargetController(0.1)
        feedback = np.zeros(7, dtype=np.float32)
        raw = JOINT_UPPER + 1.0
        safe, clipped = controller.set_target(raw, feedback)
        self.assertTrue(clipped)
        np.testing.assert_array_less(safe, JOINT_UPPER + 1e-6)
        command = controller.tick(feedback)
        self.assertLessEqual(float(np.max(np.abs(command[:6]))), 0.005001)
        self.assertLessEqual(abs(float(command[6])), 0.000201)

    def test_absolute_cursor_does_not_chase_lagging_feedback(self):
        controller = SafeTargetController(0.1)
        feedback = np.zeros(7, dtype=np.float32)
        target = np.asarray(
            [0.2, 0.2, -0.2, 0.2, 0.2, 0.2, 0.01], dtype=np.float32
        )
        controller.set_target(target, feedback)
        first = controller.tick(feedback)
        second = controller.tick(feedback)
        np.testing.assert_allclose(
            first[:6], [0.005, 0.005, -0.005, 0.005, 0.005, 0.005], atol=1e-7
        )
        np.testing.assert_allclose(
            second[:6], [0.010, 0.010, -0.010, 0.010, 0.010, 0.010], atol=1e-7
        )
        self.assertAlmostEqual(float(second[6]), 0.0004, places=7)

    def test_cursor_can_be_held_before_hardware_gate_opens(self):
        controller = SafeTargetController(1.0)
        feedback = np.zeros(7, dtype=np.float32)
        controller.set_target(np.ones(7, dtype=np.float32), feedback)
        command = controller.tick(feedback, advance=False, enforce_tracking=False)
        np.testing.assert_array_equal(command, feedback)

    def test_excessive_tracking_error_stops_controller(self):
        controller = SafeTargetController(1.0)
        feedback = np.zeros(7, dtype=np.float32)
        controller.set_target(np.asarray([1.0] * 6 + [0.07]), feedback)
        for _ in range(6):
            controller.tick(feedback)
        with self.assertRaises(TrackingError):
            controller.tick(feedback)

    def test_deadman_requires_fresh_positive_heartbeat(self):
        deadman = Deadman(0.25)
        self.assertFalse(deadman.active(1.0))
        deadman.update(True, 1.0)
        self.assertTrue(deadman.active(1.24))
        self.assertFalse(deadman.active(1.26))
        deadman.update(False, 2.0)
        self.assertFalse(deadman.active(2.0))

    def test_state_order_is_canonical(self):
        names = list(reversed(JOINT_NAMES))
        positions = list(range(7))
        velocities = list(range(10, 17))
        state = state_by_name(names, positions, velocities)
        self.assertIsNotNone(state)
        expected_positions = [positions[names.index(name)] for name in JOINT_NAMES]
        expected_velocities = [velocities[names.index(name)] for name in JOINT_NAMES]
        np.testing.assert_array_equal(state, expected_positions + expected_velocities)


if __name__ == "__main__":
    unittest.main()
