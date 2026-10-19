"""Select a complete enabled teleop or policy command for the hardware topic."""

from __future__ import annotations

import numpy as np

from .contract import JOINT_NAMES, complete_action_by_name
from .safety import Deadman


class PartialCommandAggregator:
    """Aggregate legacy partial teleop commands by joint name."""

    def __init__(self) -> None:
        self.values: dict[str, float] = {}

    def clear(self) -> None:
        self.values.clear()

    def update(self, names, positions) -> np.ndarray | None:
        if len(names) != len(positions):
            return None
        for name, position in zip(names, positions):
            if name in JOINT_NAMES and np.isfinite(position):
                self.values[str(name)] = float(position)
        try:
            return np.asarray([self.values[name] for name in JOINT_NAMES], dtype=np.float32)
        except KeyError:
            return None


class CommandRouter:
    """Give a fresh policy heartbeat priority over enabled teleoperation."""

    def __init__(self, timeout_s: float = 0.25) -> None:
        self.policy_deadman = Deadman(timeout_s)
        self.policy_session_latched = False
        self.teleop_enabled = False
        self.teleop = PartialCommandAggregator()

    def set_policy_enabled(self, enabled: bool, now: float) -> None:
        self.policy_deadman.update(enabled, now)
        if enabled:
            # Once policy control has been admitted, never fall through to
            # teleoperation merely because its heartbeat failed.  A process
            # restart after a confirmed hardware-gate close is the reset.
            self.policy_session_latched = True

    def set_teleop_enabled(self, enabled: bool) -> None:
        self.teleop_enabled = bool(enabled)
        if not enabled:
            self.teleop.clear()

    def route_policy(self, names, positions, now: float) -> np.ndarray | None:
        if not self.policy_deadman.active(now):
            return None
        return complete_action_by_name(names, positions)

    def route_teleop(self, names, positions, now: float) -> np.ndarray | None:
        action = self.teleop.update(names, positions)
        if not self.teleop_enabled or self.policy_session_latched:
            return None
        return action


def main() -> None:  # pragma: no cover - requires ROS graph
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool

    class PiperCommandMux(Node):
        def __init__(self) -> None:
            super().__init__("piper_command_mux")
            self.declare_parameter("teleop_topic", "/teleop/joint_states")
            self.declare_parameter("teleop_enabled_topic", "/teleop/enabled")
            self.declare_parameter("policy_topic", "/policy/joint_states")
            self.declare_parameter("policy_enabled_topic", "/policy/enabled")
            self.declare_parameter("hardware_topic", "/control/joint_states")
            self.router = CommandRouter()
            self.teleop_forward_logged = False
            self.policy_enabled_logged = False
            self.publisher = self.create_publisher(
                JointState, self.get_parameter("hardware_topic").value, 20
            )
            self.create_subscription(
                JointState, self.get_parameter("teleop_topic").value, self.on_teleop, 20
            )
            self.create_subscription(
                Bool,
                self.get_parameter("teleop_enabled_topic").value,
                self.on_teleop_enabled,
                10,
            )
            self.create_subscription(
                JointState, self.get_parameter("policy_topic").value, self.on_policy, 20
            )
            self.create_subscription(
                Bool,
                self.get_parameter("policy_enabled_topic").value,
                self.on_policy_enabled,
                10,
            )
            self.get_logger().info(
                "Command mux ready: teleop=%s, policy=%s, hardware=%s"
                % (
                    self.get_parameter("teleop_topic").value,
                    self.get_parameter("policy_topic").value,
                    self.get_parameter("hardware_topic").value,
                )
            )

        def now_seconds(self) -> float:
            return self.get_clock().now().nanoseconds * 1e-9

        def on_teleop_enabled(self, message) -> None:
            enabled = bool(message.data)
            self.router.set_teleop_enabled(enabled)
            self.teleop_forward_logged = False
            self.get_logger().info(
                "Teleoperation enabled" if enabled else "Teleoperation disabled"
            )

        def on_policy_enabled(self, message) -> None:
            enabled = bool(message.data)
            self.router.set_policy_enabled(enabled, self.now_seconds())
            if enabled and not self.policy_enabled_logged:
                self.get_logger().info(
                    "Policy heartbeat active; teleoperation commands are blocked"
                )
            self.policy_enabled_logged = enabled

        def on_teleop(self, message) -> None:
            action = self.router.route_teleop(message.name, message.position, self.now_seconds())
            if action is not None:
                if not self.teleop_forward_logged:
                    self.get_logger().info(
                        "Forwarding complete 7-joint teleoperation commands to hardware"
                    )
                    self.teleop_forward_logged = True
                self.publish(action)

        def on_policy(self, message) -> None:
            action = self.router.route_policy(message.name, message.position, self.now_seconds())
            if action is not None:
                self.publish(action)

        def publish(self, action) -> None:
            message = JointState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.name = list(JOINT_NAMES)
            message.position = action.tolist()
            self.publisher.publish(message)

    rclpy.init()
    node = PiperCommandMux()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
