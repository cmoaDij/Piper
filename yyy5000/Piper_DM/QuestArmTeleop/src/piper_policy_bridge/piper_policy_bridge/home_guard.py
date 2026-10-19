"""Move Piper home and require stable zero-position feedback before deployment."""

from __future__ import annotations

import argparse
from collections import deque
import sys
import time

import numpy as np

from .contract import JOINT_NAMES


ARM_JOINT_NAMES = JOINT_NAMES[:6]


class HomeVerifier:
    """Require all arm joints to remain near zero and stopped for a duration."""

    def __init__(
        self,
        position_tolerance: float,
        velocity_tolerance: float,
        stable_duration: float,
        velocity_filter_samples: int = 9,
        verify_velocity: bool = True,
        gripper_position: float | None = None,
        gripper_tolerance: float = 0.005,
        gripper_min_position: float | None = None,
    ) -> None:
        if min(position_tolerance, velocity_tolerance, stable_duration) <= 0.0:
            raise ValueError("home verification tolerances and duration must be positive")
        if velocity_filter_samples < 1 or velocity_filter_samples % 2 == 0:
            raise ValueError("velocity filter sample count must be a positive odd number")
        if gripper_position is not None and gripper_tolerance <= 0.0:
            raise ValueError("gripper tolerance must be positive")
        if gripper_position is not None and gripper_min_position is not None:
            raise ValueError(
                "gripper position and minimum position are mutually exclusive"
            )
        if gripper_min_position is not None and not (
            0.0 <= gripper_min_position <= 0.1
        ):
            raise ValueError("gripper minimum position must be in [0.0, 0.1]")
        self.position_tolerance = float(position_tolerance)
        self.velocity_tolerance = float(velocity_tolerance)
        self.stable_duration = float(stable_duration)
        self.velocity_filter_samples = int(velocity_filter_samples)
        self.verify_velocity = bool(verify_velocity)
        self.gripper_position = (
            None if gripper_position is None else float(gripper_position)
        )
        self.gripper_tolerance = float(gripper_tolerance)
        self.gripper_min_position = (
            None
            if gripper_min_position is None
            else float(gripper_min_position)
        )
        self.stable_since: float | None = None
        self.latest_positions: np.ndarray | None = None
        self.latest_velocities: np.ndarray | None = None
        self.filtered_velocities: np.ndarray | None = None
        self.latest_gripper_position: float | None = None
        self.velocity_window: deque[np.ndarray] = deque(
            maxlen=self.velocity_filter_samples
        )

    def reset(self) -> None:
        """Discard feedback observed before the home motion completed."""
        self.stable_since = None
        self.latest_positions = None
        self.latest_velocities = None
        self.filtered_velocities = None
        self.latest_gripper_position = None
        self.velocity_window.clear()

    def update(self, names, positions, velocities, now: float) -> bool:
        """Consume one complete feedback message and return stable-home status."""
        if len(names) != len(positions) or len(names) != len(velocities):
            self.stable_since = None
            self.velocity_window.clear()
            return False
        values = {
            str(name): (float(position), float(velocity))
            for name, position, velocity in zip(names, positions, velocities)
        }
        try:
            arm_positions = np.asarray(
                [values[name][0] for name in ARM_JOINT_NAMES], dtype=np.float64
            )
            arm_velocities = np.asarray(
                [values[name][1] for name in ARM_JOINT_NAMES], dtype=np.float64
            )
        except KeyError:
            self.stable_since = None
            self.velocity_window.clear()
            return False
        self.latest_positions = arm_positions
        self.latest_velocities = arm_velocities
        self.latest_gripper_position = values.get("gripper", (None, None))[0]
        finite = np.isfinite(arm_positions).all() and np.isfinite(arm_velocities).all()
        position_valid = finite and (
            np.max(np.abs(arm_positions)) <= self.position_tolerance
        )
        if self.gripper_position is not None:
            position_valid = position_valid and (
                self.latest_gripper_position is not None
                and np.isfinite(self.latest_gripper_position)
                and abs(self.latest_gripper_position - self.gripper_position)
                <= self.gripper_tolerance
            )
        if self.gripper_min_position is not None:
            position_valid = position_valid and (
                self.latest_gripper_position is not None
                and np.isfinite(self.latest_gripper_position)
                and self.latest_gripper_position >= self.gripper_min_position
            )
        if not position_valid:
            self.stable_since = None
            self.filtered_velocities = None
            self.velocity_window.clear()
            return False

        # Piper v158 reports zero-position velocity in coarse, zero-centred
        # increments with occasional one-frame spikes. A median of absolute
        # velocity rejects those outliers without accepting sustained motion.
        self.velocity_window.append(np.abs(arm_velocities))
        if len(self.velocity_window) < self.velocity_filter_samples:
            self.stable_since = None
            return False
        self.filtered_velocities = np.median(
            np.stack(tuple(self.velocity_window), axis=0), axis=0
        )
        if self.verify_velocity and (
            np.max(self.filtered_velocities) > self.velocity_tolerance
        ):
            self.stable_since = None
            return False
        if self.stable_since is None:
            self.stable_since = float(now)
        return float(now) - self.stable_since >= self.stable_duration

    def diagnostic(self) -> str:
        """Return the latest measured maxima for operator diagnostics."""
        if self.latest_positions is None or self.latest_velocities is None:
            return "no complete /feedback/joint_states message received"
        filtered_max = (
            float("nan")
            if self.filtered_velocities is None
            else float(np.max(self.filtered_velocities))
        )
        gripper_text = (
            ""
            if self.latest_gripper_position is None
            else f", gripper_position={self.latest_gripper_position:.6f} m"
        )
        return (
            f"max_abs_position={np.max(np.abs(self.latest_positions)):.6f} rad, "
            f"max_abs_velocity_raw={np.max(np.abs(self.latest_velocities)):.6f} rad/s, "
            f"max_abs_velocity_filtered={filtered_max:.6f} rad/s"
            f"{gripper_text}"
        )


def main() -> int:  # pragma: no cover - exercised through pure verifier tests
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check-only", action="store_true")
    parser.add_argument("--feedback-topic", default="/feedback/joint_states")
    parser.add_argument("--home-service", default="/move_home")
    parser.add_argument("--position-tolerance", type=float, default=0.05)
    parser.add_argument("--velocity-tolerance", type=float, default=0.05)
    parser.add_argument("--velocity-filter-samples", type=int, default=9)
    parser.add_argument(
        "--position-only",
        action="store_true",
        help="require stable near-zero arm position without gating on firmware velocity",
    )
    gripper_group = parser.add_mutually_exclusive_group()
    gripper_group.add_argument("--gripper-position", type=float)
    gripper_group.add_argument("--gripper-min-position", type=float)
    parser.add_argument("--gripper-tolerance", type=float, default=0.005)
    parser.add_argument("--stable-duration", type=float, default=0.5)
    parser.add_argument("--service-timeout", type=float, default=10.0)
    parser.add_argument("--verify-timeout", type=float, default=20.0)
    arguments, ros_arguments = parser.parse_known_args()

    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_srvs.srv import Empty

    verifier = HomeVerifier(
        arguments.position_tolerance,
        arguments.velocity_tolerance,
        arguments.stable_duration,
        arguments.velocity_filter_samples,
        verify_velocity=not arguments.position_only,
        gripper_position=arguments.gripper_position,
        gripper_tolerance=arguments.gripper_tolerance,
        gripper_min_position=arguments.gripper_min_position,
    )
    rclpy.init(args=ros_arguments)
    node = Node("piper_home_guard")
    node.create_subscription(
        JointState,
        arguments.feedback_topic,
        lambda message: verifier.update(
            message.name, message.position, message.velocity, time.monotonic()
        ),
        20,
    )
    try:
        if not arguments.check_only:
            client = node.create_client(Empty, arguments.home_service)
            if not client.wait_for_service(timeout_sec=arguments.service_timeout):
                print(f"home service unavailable: {arguments.home_service}", file=sys.stderr)
                return 1
            future = client.call_async(Empty.Request())
            rclpy.spin_until_future_complete(
                node, future, timeout_sec=arguments.service_timeout
            )
            if not future.done() or future.exception() is not None:
                print("/move_home call did not complete successfully", file=sys.stderr)
                return 1
            verifier.reset()

        deadline = time.monotonic() + arguments.verify_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if verifier.stable_since is not None and (
                time.monotonic() - verifier.stable_since >= arguments.stable_duration
            ):
                print(f"Piper home feedback verified: {verifier.diagnostic()}")
                return 0
        print(f"Piper home feedback verification failed: {verifier.diagnostic()}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
