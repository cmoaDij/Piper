import unittest

from piper_policy_bridge.command_mux import CommandRouter
from piper_policy_bridge.contract import JOINT_NAMES


class CommandMuxTest(unittest.TestCase):
    def test_policy_requires_fresh_heartbeat_and_complete_command(self):
        router = CommandRouter()
        action = [0.0] * 7
        self.assertIsNone(router.route_policy(JOINT_NAMES, action, 0.0))
        router.set_policy_enabled(True, 1.0)
        self.assertIsNotNone(router.route_policy(JOINT_NAMES, action, 1.2))
        self.assertIsNone(router.route_policy(JOINT_NAMES, action, 1.3))
        self.assertIsNone(router.route_policy(JOINT_NAMES[:-1], action[:-1], 1.1))

    def test_policy_has_priority_over_teleop(self):
        router = CommandRouter()
        router.set_teleop_enabled(True)
        router.set_policy_enabled(True, 1.0)
        self.assertIsNone(router.route_teleop(JOINT_NAMES, [0.0] * 7, 1.1))
        router.set_policy_enabled(False, 1.2)
        self.assertIsNone(router.route_teleop(JOINT_NAMES, [0.0] * 7, 1.2))

    def test_future_dated_heartbeat_is_not_active(self):
        router = CommandRouter()
        router.set_policy_enabled(True, 2.0)
        self.assertIsNone(router.route_policy(JOINT_NAMES, [0.0] * 7, 1.9))


if __name__ == "__main__":
    unittest.main()
