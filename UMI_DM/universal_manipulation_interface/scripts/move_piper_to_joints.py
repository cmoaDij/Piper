#!/usr/bin/env python3
"""Move Piper to one explicit six-joint startup pose.

The script is deliberately separate from policy execution: it sends only arm
joint targets, never gripper targets, and exits after fresh feedback confirms
the requested pose. Motion requires the explicit ``--execute`` flag.
"""
import argparse
import os
from pathlib import Path
import signal
import sys
import time

import numpy as np
import yaml


PIPER_ROOT = Path(os.environ.get(
    "PIPER_DATA_COLLECT", "/home/l1/Piper_All/Piper/Data_Collect"))
sys.path.insert(0, str(PIPER_ROOT))

from data_collect.common.config import load_config
from data_collect.cli.umi_piper_sdk import build_guard, connect_arm
from data_collect.control.piper_sdk_session import PiperSdkSession


def restore_stop_when_ready(session, timeout_s=3.0):
    deadline = time.monotonic() + timeout_s
    last_error = None
    while time.monotonic() < deadline:
        try:
            session.restore_electronic_stop()
            return
        except ValueError as exc:
            last_error = exc
            if "feedback missing" not in str(exc) and "feedback is stale" not in str(exc):
                raise
            time.sleep(0.1)
    raise TimeoutError(f"Piper feedback did not become ready: {last_error}")


def wait_for_pose(session, target, timeout_s, tolerance_deg, resend_s=0.10):
    deadline = time.monotonic() + timeout_s
    last_send = float("-inf")
    last_state = None
    while time.monotonic() < deadline:
        state = session.read_state(include_gripper=False)
        last_state = state
        error_deg = float(np.rad2deg(np.max(np.abs(state["joints"] - target))))
        if error_deg <= tolerance_deg:
            return state, error_deg
        now = time.monotonic()
        if now - last_send >= resend_s:
            session.send_joints(target)
            last_send = now
        time.sleep(0.02)
    current = np.rad2deg(last_state["joints"] if last_state is not None else target)
    raise TimeoutError(
        f"Target pose was not reached within {timeout_s:g}s; "
        f"current_deg={np.round(current, 3).tolist()}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true",
                        help="Required explicit motion authorization")
    parser.add_argument("--setup", type=Path, required=True,
                        help="Piper deployment setup.yaml")
    parser.add_argument("--config", type=Path,
                        default=PIPER_ROOT / "config/collection.yaml")
    parser.add_argument("--can", default="can0")
    parser.add_argument("--joints-deg", nargs=6, type=float, required=True,
                        metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
                        help="Six target joint angles in degrees")
    parser.add_argument("--speed-percent", type=int, default=5,
                        help="SDK move_j planning speed (1..100; default: 5)")
    parser.add_argument("--tolerance-deg", type=float, default=1.0,
                        help="Maximum absolute joint error for success")
    parser.add_argument("--timeout-s", type=float, default=60.0,
                        help="Maximum time allowed to reach target")
    args = parser.parse_args()
    if not args.execute:
        parser.error("motion requires the explicit --execute flag")
    if not 1 <= args.speed_percent <= 100:
        parser.error("--speed-percent must be between 1 and 100")
    if args.tolerance_deg <= 0 or args.timeout_s <= 0:
        parser.error("--tolerance-deg and --timeout-s must be positive")

    target_deg = np.asarray(args.joints_deg, dtype=float)
    if target_deg.shape != (6,) or not np.isfinite(target_deg).all():
        parser.error("--joints-deg must contain six finite values")
    target = np.deg2rad(target_deg)
    setup = yaml.safe_load(args.setup.read_text())
    cfg = load_config(str(args.config))

    arm = None
    session = None
    def on_signal(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    try:
        arm, grip, firmware = connect_arm(args.can)
        _, limits = build_guard(cfg, setup, arm)
        lower = np.asarray([limits[f"joint{i}"][0] for i in range(1, 7)])
        upper = np.asarray([limits[f"joint{i}"][1] for i in range(1, 7)])
        if np.any(target < lower) or np.any(target > upper):
            raise ValueError(
                "Requested startup pose is outside the URDF/controller joint limits: "
                f"target_deg={np.round(target_deg, 3).tolist()}")

        gripper_cfg = setup["gripper"]
        session = PiperSdkSession(
            arm, grip,
            command_timeout=cfg["command_timeout_s"],
            gripper_max_m=(float(gripper_cfg["nominal_training_range_m"][1])
                           - float(gripper_cfg["feedback_to_fingertip_offset_m"])),
            gripper_feedback_floor_m=float(gripper_cfg.get("feedback_raw_floor_m", -0.010)))
        session.configure_joint_limits(limits)
        restore_stop_when_ready(session)
        arm.set_speed_percent(args.speed_percent)
        initial = session.read_state(include_gripper=False)
        print("Piper firmware:", firmware["software_version"], flush=True)
        print("Initial joints (deg):",
              np.rad2deg(initial["joints"]).round(3).tolist(), flush=True)
        print("Target joints (deg):", target_deg.round(3).tolist(), flush=True)
        print(f"SDK move_j speed: {args.speed_percent}%", flush=True)
        print("Gripper command: none", flush=True)

        session.activate()
        session.send_joints(target)
        state, error_deg = wait_for_pose(
            session, target, args.timeout_s, args.tolerance_deg)
        print("Startup pose reached (deg):",
              np.rad2deg(state["joints"]).round(3).tolist(), flush=True)
        print(f"Maximum joint error: {error_deg:.3f} deg", flush=True)
    except KeyboardInterrupt:
        print("Interrupted; requesting SDK position hold.", flush=True)
        raise SystemExit(130)
    finally:
        if session is not None:
            session.close()
        elif arm is not None:
            arm.disconnect()


if __name__ == "__main__":
    main()
