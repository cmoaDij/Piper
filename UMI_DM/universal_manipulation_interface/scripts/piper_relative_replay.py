#!/usr/bin/env python3
"""Replay a recorded UMI TCP trajectory relative to the current Piper pose.

No tag or ``T_base_tag`` is used.  The first recorded TCP pose becomes the
identity motion, and that relative motion is applied to the TCP pose measured
from the Piper at replay start.  Hardware is never commanded unless
``--execute`` is supplied; the default is a read-only dry run.
"""

import argparse
import fcntl
import json
import sys
import time
from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation


UMI_ROOT = Path(__file__).resolve().parents[1]
PIPER_ROOT = UMI_ROOT.parent.parent / "Piper" / "Data_Collect"
sys.path.insert(0, str(UMI_ROOT))
sys.path.insert(0, str(PIPER_ROOT))

from umi.real_world.piper_relative_replay import (
    anchor_relative_pose,
    matrix_to_pose,
    resample_pose_trajectory,
    sdk_flange_pose_to_matrix,
)


def _latest_dataset():
    candidates = sorted((UMI_ROOT.parent / "my_session2" / "processed_runs").glob(
        "*/dataset.zarr.zip"
    ))
    return candidates[-1] if candidates else UMI_ROOT.parent / "my_session2/dataset.zarr.zip"


def _matrix_from_pose(pose):
    pose = np.asarray(pose, dtype=float)
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_rotvec(pose[3:]).as_matrix()
    matrix[:3, 3] = pose[:3]
    return matrix


def _load_episode(dataset_path, episode):
    dataset_path = Path(dataset_path).expanduser().resolve()
    if not dataset_path.is_file():
        raise FileNotFoundError(f"Dataset does not exist: {dataset_path}")
    if dataset_path.suffix == ".npz":
        with np.load(dataset_path, allow_pickle=False) as source:
            pose = np.asarray(source["pose"], dtype=float)
            width = np.asarray(source["gripper_width"], dtype=float).reshape(-1)
            episode_index = int(np.asarray(source.get("episode_index", episode)).item())
            episode_count = int(np.asarray(source.get("episode_count", 1)).item())
        if episode != episode_index:
            raise ValueError(
                f"NPZ contains episode {episode_index}, but episode {episode} was requested"
            )
        if pose.ndim != 2 or pose.shape[1] != 6 or len(width) != len(pose):
            raise ValueError("Replay NPZ contains incompatible trajectory arrays")
        if not np.isfinite(pose).all() or not np.isfinite(width).all():
            raise ValueError("Replay NPZ contains non-finite values")
        return pose, width, episode_count

    try:
        import zarr
        from diffusion_policy.codecs.imagecodecs_numcodecs import register_codecs
        from diffusion_policy.common.replay_buffer import ReplayBuffer
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "This Python environment has no zarr support. Run "
            "scripts/run_piper_relative_replay.sh so the capture environment "
            "can consume the temporary trajectory export."
        ) from exc
    register_codecs()

    with zarr.ZipStore(str(dataset_path), mode="r") as source:
        replay = ReplayBuffer.copy_from_store(src_store=source, store=zarr.MemoryStore())
    if not 0 <= episode < replay.n_episodes:
        raise ValueError(f"episode must be in [0, {replay.n_episodes - 1}], got {episode}")
    part = replay.get_episode(episode)
    required = ("robot0_eef_pos", "robot0_eef_rot_axis_angle", "robot0_gripper_width")
    missing = [key for key in required if key not in part]
    if missing:
        raise ValueError(f"Dataset episode is missing {missing}")
    position = np.asarray(part[required[0]], dtype=float)
    rotation = np.asarray(part[required[1]], dtype=float)
    width = np.asarray(part[required[2]], dtype=float).reshape(-1)
    if position.ndim != 2 or position.shape[1] != 3 or rotation.shape != position.shape:
        raise ValueError("Dataset TCP pose arrays have incompatible shapes")
    pose = np.concatenate([position, rotation], axis=1)
    if not np.isfinite(pose).all() or not np.isfinite(width).all():
        raise ValueError("Dataset TCP trajectory contains non-finite values")
    return pose, width, replay.n_episodes


def _flange_tcp_from_setup(setup):
    tool = setup.get("flange_to_policy_tcp")
    if not isinstance(tool, dict) or not tool.get("rotation_verified"):
        raise ValueError("flange_to_policy_tcp.rotation_verified must be true")
    translation = np.asarray(tool.get("translation_m"), dtype=float)
    rotation = np.asarray(tool.get("rotation"), dtype=float)
    if (translation.shape != (3,) or rotation.shape != (3,)
            or not np.isfinite(translation).all() or not np.isfinite(rotation).all()):
        raise ValueError("Invalid flange_to_policy_tcp transform")
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_rotvec(rotation).as_matrix()
    matrix[:3, 3] = translation
    return matrix


def _solve_waypoint(ik, target_tcp, previous_q, limits, max_jump, rules, check_collision):
    q = np.asarray(ik.solve(target_tcp), dtype=float)
    lower = np.asarray([limits[f"joint{i}"][0] for i in range(1, 7)], dtype=float)
    upper = np.asarray([limits[f"joint{i}"][1] for i in range(1, 7)], dtype=float)
    if q.shape != (6,) or not np.isfinite(q).all() or np.any(q < lower) or np.any(q > upper):
        raise ValueError("IK returned a joint target outside the checked limits")
    if previous_q is not None and np.max(np.abs(q - previous_q)) > max_jump:
        raise ValueError(
            f"IK joint jump {np.rad2deg(np.max(np.abs(q - previous_q))):.2f} deg "
            f"exceeds {np.rad2deg(max_jump):.2f} deg"
        )
    actual = np.asarray(ik.end_effector_pose(q), dtype=float)
    position_error = float(np.linalg.norm(actual[:3, 3] - target_tcp[:3, 3]))
    rotation_error = float(Rotation.from_matrix(actual[:3, :3].T @ target_tcp[:3, :3]).magnitude())
    if position_error > rules["fk_position_tolerance_m"] or rotation_error > rules["fk_rotation_tolerance_rad"]:
        raise ValueError(
            f"IK residual is too large: {position_error * 1000:.3f} mm / "
            f"{np.rad2deg(rotation_error):.3f} deg"
        )
    if check_collision and ik.check_self_collision(q):
        raise ValueError("IK target is in modeled self-collision")
    return q, position_error, rotation_error


def _read_state_with_retry(session, timeout_s=5.0):
    """Wait briefly for fresh CAN feedback after connecting or reconnecting."""
    deadline = time.monotonic() + float(timeout_s)
    last_error = None
    while time.monotonic() < deadline:
        try:
            return session.read_state()
        except ValueError as exc:
            last_error = exc
            time.sleep(.1)
    raise RuntimeError(f"Timed out waiting for fresh Piper feedback: {last_error}") from last_error


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=_latest_dataset(), help="Zarr dataset zip")
    parser.add_argument("--episode", type=int, default=0)
    parser.add_argument("--source-hz", type=float, default=60000 / 1001,
                        help="Sampling rate of the recorded trajectory")
    parser.add_argument("--replay-hz", type=float, default=None,
                        help="Command rate; defaults to setup latency_compensation.action_period_s")
    parser.add_argument("--rate-scale", type=float, default=1.0,
                        help="Replay duration multiplier; >1 slows the trajectory")
    parser.add_argument("--setup", type=Path,
                        default=UMI_ROOT.parent / "deploy/current/piper_policy_setup.yaml")
    parser.add_argument("--config", type=Path, default=PIPER_ROOT / "config/collection.yaml")
    parser.add_argument("--can", default="can0")
    parser.add_argument("--output", type=Path, default=None, help="JSONL diagnostic output")
    parser.add_argument("--source-dataset", type=Path, default=None,
                        help="Original dataset path when --input is a temporary NPZ")
    parser.add_argument("--start-delay", type=float, default=2.0)
    parser.add_argument("--no-collision-check", action="store_true")
    parser.add_argument("--execute", action="store_true",
                        help="Enable the Piper and send targets; omitted means read-only dry-run")
    args = parser.parse_args()

    if (not np.isfinite([args.source_hz, args.rate_scale, args.start_delay]).all()
            or args.source_hz <= 0 or args.rate_scale <= 0 or args.start_delay < 0):
        parser.error("source-hz and rate-scale must be positive; start-delay must be nonnegative")
    if args.replay_hz is not None and (not np.isfinite(args.replay_hz) or args.replay_hz <= 0):
        parser.error("replay-hz must be positive")
    if not args.can.replace("_", "").replace("-", "").isalnum():
        parser.error("invalid CAN interface name")

    cfg_path = args.config.expanduser().resolve()
    setup_path = args.setup.expanduser().resolve()
    with cfg_path.open(encoding="utf-8") as stream:
        import yaml as _yaml
        cfg_file = _yaml.safe_load(stream)
    with setup_path.open(encoding="utf-8") as stream:
        setup = yaml.safe_load(stream)
    pose, width, episode_count = _load_episode(args.input, args.episode)
    replay_hz = args.replay_hz or 1.0 / float(setup.get("latency_compensation", {}).get("action_period_s", .05005))
    pose, width = resample_pose_trajectory(pose, width, args.source_hz, replay_hz)
    replay_period_s = float(args.rate_scale) / float(replay_hz)
    flange_tcp = _flange_tcp_from_setup(setup)
    gripper_cfg = setup.get("gripper", {})
    rules = dict(cfg_file["motion"])
    max_jump = float(rules["max_ik_jump_rad"])

    if args.output is None:
        output = UMI_ROOT.parent / "deploy/current/replays" / f"episode_{args.episode:03d}_relative.jsonl"
    else:
        output = args.output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)

    from data_collect.common.config import load_config
    from data_collect.cli.umi_piper_sdk import build_guard, connect_arm
    from data_collect.control.piper_sdk_session import PiperSdkSession

    cfg = load_config(str(cfg_path))
    lock_path = Path("/tmp") / f"umi_piper_sdk_{args.can}.lock"
    with lock_path.open("a+") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError(f"Another Piper SDK controller holds {lock_path}") from exc

        arm = session = None
        log_stream = output.open("w", encoding="utf-8", buffering=1)
        try:
            arm, grip, firmware = connect_arm(args.can)
            max_session_width = (
                float(gripper_cfg.get("nominal_training_range_m", [0.0, .076])[1])
                - float(gripper_cfg.get("feedback_to_fingertip_offset_m", 0.0))
            )
            session = PiperSdkSession(
                arm, grip, command_timeout=cfg["command_timeout_s"],
                gripper_max_m=max_session_width,
                gripper_feedback_floor_m=float(
                    gripper_cfg.get("feedback_raw_floor_m", -.010)),
            )
            guard, limits = build_guard(cfg, setup, arm)
            session.configure_joint_limits(limits)
            state = _read_state_with_retry(session)
            time.sleep(float(cfg["motion"]["stationary_duration_s"]))
            fresh = _read_state_with_retry(session)
            if np.max(np.abs(fresh["joints"] - state["joints"])) > cfg["motion"]["stationary_tolerance_rad"]:
                raise RuntimeError("Robot is moving; keep it stationary before replay")
            state = fresh
            seed = np.clip(state["joints"], guard.guard.lower, guard.guard.upper)
            guard.ik.sync_state(seed)
            guard.guard.reset(seed)
            # Piper SDK feedback is xyz + XYZ extrinsic roll/pitch/yaw, not a
            # rotation vector like the recorded UMI trajectory.
            anchor_tcp = sdk_flange_pose_to_matrix(state["flange"]) @ flange_tcp
            # The SDK feedback TCP and the URDF FK TCP can differ by a fixed
            # installation/zero correction.  Align the first target to the
            # model pose at the measured joint state, while preserving every
            # recorded relative motion exactly.
            model_anchor_tcp = np.asarray(guard.ik.end_effector_pose(seed), dtype=float)
            reference_correction = model_anchor_tcp @ np.linalg.inv(anchor_tcp)
            startup_position_error = float(np.linalg.norm(model_anchor_tcp[:3, 3] - anchor_tcp[:3, 3]))
            startup_rotation_error = float(
                Rotation.from_matrix(model_anchor_tcp[:3, :3].T @ anchor_tcp[:3, :3]).magnitude()
            )
            target_tcp = anchor_relative_pose(anchor_tcp, pose)
            relative_delta = np.linalg.inv(_matrix_from_pose(pose[0])) @ _matrix_from_pose(pose[-1])
            print(f"Loaded episode {args.episode}/{episode_count - 1}: {len(pose)} waypoints, "
                  f"{(len(pose) - 1) * replay_period_s:.2f} s, "
                  f"net TCP translation {np.linalg.norm(relative_delta[:3, 3]) * 1000:.1f} mm", flush=True)
            print("Startup SDK/FK TCP discrepancy: "
                  f"{startup_position_error * 1000:.2f} mm / "
                  f"{np.rad2deg(startup_rotation_error):.2f} deg", flush=True)
            print("Replay frame: current robot TCP is the anchor; no tag/T_base_tag is used.", flush=True)
            print("Mode:", "EXECUTE" if args.execute else "DRY-RUN", flush=True)
            log_stream.write(json.dumps({
                "event": "replay_start",
                "dataset": str(Path(args.source_dataset or args.input).expanduser().resolve()),
                "episode": args.episode,
                "episode_count": episode_count,
                "waypoint_count": len(pose),
                "replay_hz": replay_hz,
                "replay_period_s": replay_period_s,
                "source_hz": args.source_hz,
                "rate_scale": args.rate_scale,
                "anchor_tcp_pose_base": matrix_to_pose(anchor_tcp).tolist(),
                "sdk_flange_pose_xyz_rpy": np.asarray(state["flange"], dtype=float).tolist(),
                "model_anchor_tcp_pose_base": matrix_to_pose(model_anchor_tcp).tolist(),
                "reference_correction": reference_correction.tolist(),
                "startup_sdk_fk_position_error_m": startup_position_error,
                "startup_sdk_fk_rotation_error_rad": startup_rotation_error,
                "flange_to_tcp": flange_tcp.tolist(),
                "execute": bool(args.execute),
            }, ensure_ascii=False, allow_nan=False) + "\n")

            if args.execute:
                if args.start_delay:
                    print(f"Starting in {args.start_delay:.1f} s; press Ctrl+C to abort.", flush=True)
                    time.sleep(args.start_delay)
                session.restore_electronic_stop()
                speed = setup.get("policy_motion", {}).get("sdk_speed_percent", 20)
                if isinstance(speed, bool) or not isinstance(speed, int) or not 1 <= speed <= 100:
                    raise ValueError("SDK speed percent must be an integer from 1 to 100")
                arm.set_speed_percent(speed)
                session.activate()

            start = time.monotonic()
            previous_q = seed.copy()
            for index, (tcp_target, recorded_width) in enumerate(zip(target_tcp, width)):
                deadline = start + index * replay_period_s
                now = time.monotonic()
                if deadline > now:
                    time.sleep(deadline - now)
                command_width = float(recorded_width)
                if not np.isfinite(command_width) or command_width < 0:
                    raise ValueError(f"Invalid recorded gripper width {command_width}")
                ik_target_tcp = reference_correction @ tcp_target
                q, pos_error, rot_error = _solve_waypoint(
                    guard.ik, ik_target_tcp, previous_q, limits, max_jump, rules,
                    check_collision=not args.no_collision_check)
                target_flange = tcp_target @ np.linalg.inv(flange_tcp)
                record = {
                    "event": "replay_waypoint",
                    "episode": args.episode,
                    "index": index,
                    "time_since_start_s": time.monotonic() - start,
                    "recorded_tcp_pose": matrix_to_pose(_matrix_from_pose(pose[index])).tolist(),
                    "target_tcp_pose_base": matrix_to_pose(tcp_target).tolist(),
                    "ik_target_tcp_pose_base": matrix_to_pose(ik_target_tcp).tolist(),
                    "target_flange_pose_base": matrix_to_pose(target_flange).tolist(),
                    "target_joint_rad": q.tolist(),
                    "recorded_gripper_m": float(recorded_width),
                    "commanded_gripper_m": float(command_width),
                    "ik_residual_position_m": pos_error,
                    "ik_residual_rotation_rad": rot_error,
                    "execute": bool(args.execute),
                }
                log_stream.write(json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n")
                if args.execute:
                    session.send({"joints": q.tolist(), "gripper_m": command_width, "force_n": 1.0})
                guard.ik.accept(q)
                guard.guard.accept(q)
                previous_q = q
                if index == 0 or index == len(target_tcp) - 1:
                    print(f"waypoint {index + 1}/{len(target_tcp)}: "
                          f"q={np.rad2deg(q).round(2).tolist()} grip={command_width * 1000:.1f} mm",
                          flush=True)
            if args.execute:
                print("Replay completed; the session will send a position hold on exit.", flush=True)
        except KeyboardInterrupt:
            print("Replay interrupted.", flush=True)
            raise
        finally:
            log_stream.close()
            if session is not None:
                session.close()
            elif arm is not None:
                arm.disconnect()


if __name__ == "__main__":
    main()
