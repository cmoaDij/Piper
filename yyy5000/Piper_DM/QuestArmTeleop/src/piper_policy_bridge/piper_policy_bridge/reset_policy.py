"""Explicitly reset the LeRobot policy action queue over local IPC."""

from __future__ import annotations

import argparse

from .ipc_client import PolicyClient


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--endpoint", default="ipc:///tmp/piper_lerobot.sock")
    parser.add_argument("--timeout-ms", type=int, default=2000)
    arguments = parser.parse_args()
    PolicyClient(arguments.endpoint, arguments.timeout_ms).reset(0)
    print("Policy action queue reset successfully.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
