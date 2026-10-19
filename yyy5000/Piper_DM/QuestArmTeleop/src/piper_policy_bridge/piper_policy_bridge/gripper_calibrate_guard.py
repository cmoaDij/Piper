"""Call the fail-closed one-shot gripper zero-calibration service."""

from __future__ import annotations

import argparse
import sys


def build_parser() -> argparse.ArgumentParser:
    """Build the explicit physical-confirmation command line contract."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--confirm-physically-closed",
        action="store_true",
        help="confirm both fingers are physically fully closed with no object between",
    )
    parser.add_argument(
        "--service",
        default="/calibrate_gripper_closed",
    )
    parser.add_argument("--timeout", type=float, default=10.0)
    return parser


def main() -> int:  # pragma: no cover - ROS service exercised on hardware
    """Require explicit confirmation, call calibration once, and report the result."""
    parser = build_parser()
    arguments, ros_arguments = parser.parse_known_args()
    if not arguments.confirm_physically_closed:
        parser.error(
            "refusing calibration without --confirm-physically-closed"
        )
    if arguments.timeout <= 0.0:
        parser.error("--timeout must be positive")

    import rclpy
    from rclpy.node import Node
    from std_srvs.srv import Trigger

    rclpy.init(args=ros_arguments)
    node = Node("piper_gripper_calibrate_guard")
    try:
        client = node.create_client(Trigger, arguments.service)
        if not client.wait_for_service(timeout_sec=arguments.timeout):
            print(
                f"gripper calibration service unavailable: {arguments.service}",
                file=sys.stderr,
            )
            return 1
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=arguments.timeout)
        if not future.done() or future.exception() is not None:
            print("gripper calibration service call timed out", file=sys.stderr)
            return 1
        response = future.result()
        if not response.success:
            print(f"gripper calibration refused: {response.message}", file=sys.stderr)
            return 1
        print(f"Piper gripper calibration verified: {response.message}")
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
