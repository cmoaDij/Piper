import unittest

from piper_policy_bridge.contract import JOINT_NAMES
from piper_policy_bridge.home_guard import HomeVerifier


class HomeVerifierTest(unittest.TestCase):
    def test_requires_stable_zero_feedback(self):
        verifier = HomeVerifier(0.05, 0.05, 0.5, velocity_filter_samples=3)
        positions = [0.01] * 6 + [0.07]
        velocities = [0.01] * 7

        self.assertFalse(verifier.update(JOINT_NAMES, positions, velocities, 1.0))
        self.assertFalse(verifier.update(JOINT_NAMES, positions, velocities, 1.1))
        self.assertFalse(verifier.update(JOINT_NAMES, positions, velocities, 1.2))
        self.assertFalse(verifier.update(JOINT_NAMES, positions, velocities, 1.6))
        self.assertTrue(verifier.update(JOINT_NAMES, positions, velocities, 1.7))

    def test_out_of_tolerance_feedback_restarts_stability_window(self):
        verifier = HomeVerifier(0.05, 0.05, 0.5, velocity_filter_samples=1)
        home = [0.0] * 7
        moving = [0.0] * 7
        moving[2] = 0.06

        self.assertFalse(verifier.update(JOINT_NAMES, home, home, 1.0))
        self.assertFalse(verifier.update(JOINT_NAMES, moving, home, 1.4))
        self.assertFalse(verifier.update(JOINT_NAMES, home, home, 1.5))
        self.assertTrue(verifier.update(JOINT_NAMES, home, home, 2.0))

    def test_gripper_is_not_part_of_arm_home_check(self):
        verifier = HomeVerifier(0.05, 0.05, 0.1, velocity_filter_samples=1)
        positions = [0.0] * 6 + [0.07]
        velocities = [0.0] * 7

        verifier.update(JOINT_NAMES, positions, velocities, 1.0)
        self.assertTrue(verifier.update(JOINT_NAMES, positions, velocities, 1.1))

    def test_quantized_velocity_spikes_are_filtered(self):
        verifier = HomeVerifier(0.05, 0.05, 0.5, velocity_filter_samples=5)
        positions = [0.0] * 7
        samples = [0.034, 0.051, 0.017, 0.103, 0.034]

        for index, speed in enumerate(samples):
            velocities = [speed] + [0.0] * 6
            self.assertFalse(
                verifier.update(JOINT_NAMES, positions, velocities, index * 0.1)
            )
        self.assertTrue(
            verifier.update(JOINT_NAMES, positions, [0.034] * 6 + [0.0], 0.9)
        )
        self.assertIn("max_abs_velocity_filtered=0.034000", verifier.diagnostic())

    def test_sustained_velocity_still_rejects_home(self):
        verifier = HomeVerifier(0.05, 0.05, 0.5, velocity_filter_samples=5)
        positions = [0.0] * 7
        velocities = [0.069] * 6 + [0.0]

        for index in range(20):
            self.assertFalse(
                verifier.update(JOINT_NAMES, positions, velocities, index * 0.1)
            )
        self.assertIsNone(verifier.stable_since)

    def test_position_only_accepts_stationary_zero_with_noisy_velocity(self):
        verifier = HomeVerifier(
            0.05,
            0.05,
            0.5,
            velocity_filter_samples=3,
            verify_velocity=False,
        )
        positions = [0.0] * 6 + [0.07]
        velocities = [0.103] * 6 + [0.0]

        for timestamp in [0.0, 0.1, 0.2, 0.71]:
            accepted = verifier.update(
                JOINT_NAMES, positions, velocities, timestamp
            )
        self.assertTrue(accepted)

    def test_required_gripper_must_be_open(self):
        verifier = HomeVerifier(
            0.05,
            0.05,
            0.1,
            velocity_filter_samples=1,
            verify_velocity=False,
            gripper_min_position=0.06,
        )
        closed = [0.0] * 7
        almost_open = [0.0] * 6 + [0.059]
        opened = [0.0] * 6 + [0.06412]
        velocities = [0.0] * 7

        self.assertFalse(verifier.update(JOINT_NAMES, closed, velocities, 0.0))
        self.assertFalse(
            verifier.update(JOINT_NAMES, almost_open, velocities, 0.1)
        )
        self.assertFalse(verifier.update(JOINT_NAMES, opened, velocities, 0.2))
        self.assertTrue(verifier.update(JOINT_NAMES, opened, velocities, 0.31))


if __name__ == "__main__":
    unittest.main()
