#!/usr/bin/env python3
"""Offline diagnostic for the UMI policy TCP to Piper base mapping.

This script is deliberately read-only.  It reads a saved deployment run and,
optionally, the training Zarr dataset.  It never opens CAN, enables the arm,
or sends a command.

The policy action is expressed in the policy TCP frame.  The deployment bridge
maps it through the current flange pose and the configured flange-to-policy-TCP
transform.  This diagnostic reports both parts separately:

* where the policy TCP X/Y/Z axes point in the Piper base frame;
* where the configured 195 mm origin offset lands in the base frame; and
* the local action deltas recovered from recorded target TCP matrices.

The third coordinate in the training Zarr file is not assumed to be Piper's
base Z.  Training and deployment local-frame statistics are compared instead.
"""

import argparse
from collections import Counter
import json
from pathlib import Path
import sys

import numpy as np
import yaml
from scipy.spatial.transform import Rotation


def _read_jsonl(path):
    if not path.is_file():
        return []
    rows = []
    for line in path.read_text().splitlines():
        if line.strip():
            rows.append(json.loads(line))
    return rows


def _pose_matrix(pose):
    pose = np.asarray(pose, dtype=float)
    if pose.shape != (6,) or not np.isfinite(pose).all():
        raise ValueError(f"Expected finite xyz/rpy pose, got {pose!r}")
    out = np.eye(4)
    out[:3, :3] = Rotation.from_euler("xyz", pose[3:]).as_matrix()
    out[:3, 3] = pose[:3]
    return out


def _load_tool(setup_path):
    setup = yaml.safe_load(Path(setup_path).read_text()) or {}
    cfg = setup.get("flange_to_policy_tcp", {})
    translation = np.asarray(cfg.get("translation_m", [0.0, 0.0, 0.195]), dtype=float)
    rotation = np.asarray(cfg.get("rotation", [0.0, 0.0, 0.0]), dtype=float)
    if translation.shape != (3,) or rotation.shape != (3,):
        raise ValueError("flange_to_policy_tcp translation_m and rotation must have 3 values")
    if not np.isfinite(translation).all() or not np.isfinite(rotation).all():
        raise ValueError("flange_to_policy_tcp contains non-finite values")
    tool = np.eye(4)
    tool[:3, :3] = Rotation.from_rotvec(rotation).as_matrix()
    tool[:3, 3] = translation
    return setup, tool, translation, rotation


def _stats(values):
    values = np.asarray(values, dtype=float)
    if values.size == 0:
        return {"count": 0}
    if values.ndim == 1:
        values = values.reshape(-1, 1)
    result = {"count": int(values.shape[0])}
    for j, axis in enumerate("xyz"[:values.shape[1]]):
        column = values[:, j]
        result[axis] = {
            "min": float(np.min(column)),
            "median": float(np.percentile(column, 50)),
            "p95": float(np.percentile(column, 95)),
            "max": float(np.max(column)),
        }
    return result


def _scalar_stats(values):
    values = np.asarray(values, dtype=float)
    if values.size == 0:
        return {"count": 0}
    values = values[np.isfinite(values)]
    if values.size == 0:
        return {"count": 0}
    return {
        "count": int(values.size),
        "median": float(np.percentile(values, 50)),
        "p95": float(np.percentile(values, 95)),
        "min": float(np.min(values)),
        "max": float(np.max(values)),
    }


def _vec_stats(values, unit_scale=1.0):
    values = np.asarray(values, dtype=float) * unit_scale
    return _stats(values)


def _round_vec(value, digits=6):
    return np.round(np.asarray(value, dtype=float), digits).tolist()


def _run_rows(run_dir, tool):
    controls = _read_jsonl(run_dir / "control_events.jsonl")
    rows = []
    event_counts = Counter(row.get("event") for row in controls)
    for row in controls:
        # One row per actually scheduled arm waypoint.  The SDK-return and
        # diagnostic rows are duplicates for this purpose.
        if row.get("event") != "waypoint_scheduled":
            continue
        diagnostic = row.get("ik_diagnostic") or {}
        target_tcp = diagnostic.get("target_tcp_matrix")
        flange_pose = row.get("actual_flange_xyz_rpy") or row.get("feedback_flange_xyz_rpy")
        if target_tcp is None or flange_pose is None:
            continue
        target_tcp = np.asarray(target_tcp, dtype=float)
        if target_tcp.shape != (4, 4) or not np.isfinite(target_tcp).all():
            continue
        current_flange = _pose_matrix(flange_pose)
        current_tcp = current_flange @ tool
        delta_tcp = np.linalg.inv(current_tcp) @ target_tcp
        R_base_tcp = current_tcp[:3, :3]
        local_delta = delta_tcp[:3, 3]
        base_delta = R_base_tcp @ local_delta
        rows.append({
            "prediction_sequence": row.get("prediction_sequence"),
            "action_index": row.get("selected_action_index", diagnostic.get("action_index")),
            "flange_pose": np.asarray(flange_pose, dtype=float),
            "current_tcp": current_tcp,
            "target_tcp": target_tcp,
            "R_base_tcp": R_base_tcp,
            "local_delta": local_delta,
            "base_delta": base_delta,
            "base_delta_axis_contribution": R_base_tcp[2, :] * local_delta,
        })
    return rows, event_counts


def _axis_report(rows, tool, translation):
    if not rows:
        return {"count": 0}
    axes = np.asarray([row["R_base_tcp"] for row in rows])
    # R_base_tcp already includes the configured TCP rotation, while the
    # translation is expressed in the flange frame.  Recover R_base_flange
    # from the recorded pose for the 195 mm origin-offset report.
    offset_base = np.asarray([
        _pose_matrix(row["flange_pose"])[:3, :3] @ translation for row in rows
    ])
    axes_median = np.median(axes, axis=0)
    z_projection = float(axes_median[2, 2])
    if z_projection > 0.8:
        z_verdict = "local +Z maps strongly toward Piper base +Z"
    elif z_projection < -0.2:
        z_verdict = "local +Z does not map toward Piper base +Z; it has a downward base-Z component"
    else:
        z_verdict = "local +Z is mostly lateral in the Piper base frame"
    return {
        "count": len(rows),
        "first": {
            "policy_x_axis_in_base": _round_vec(axes[0, :, 0]),
            "policy_y_axis_in_base": _round_vec(axes[0, :, 1]),
            "policy_z_axis_in_base": _round_vec(axes[0, :, 2]),
            "base_vertical_projection_of_local_axes": _round_vec(axes[0, 2, :]),
            "tcp_origin_offset_in_base_m": _round_vec(offset_base[0]),
        },
        "median_axes": {
            "policy_x_axis_in_base": _round_vec(axes_median[:, 0]),
            "policy_y_axis_in_base": _round_vec(axes_median[:, 1]),
            "policy_z_axis_in_base": _round_vec(axes_median[:, 2]),
            "base_vertical_projection_of_local_axes": _round_vec(axes_median[2, :]),
        },
        "axis_component_ranges": {
            "base_vertical_projection_local_x": _scalar_stats(axes[:, 2, 0]),
            "base_vertical_projection_local_y": _scalar_stats(axes[:, 2, 1]),
            "base_vertical_projection_local_z": _scalar_stats(axes[:, 2, 2]),
        },
        "tcp_origin_offset_in_base_m": _vec_stats(offset_base),
        "tcp_origin_offset_base_z_m": _scalar_stats(offset_base[:, 2]),
        "local_z_mapping_verdict": z_verdict,
        "fixed_offset_is_not_a_cumulative_motion": True,
        "interpretation": (
            "A local +Z action maps to base Z according to the third column "
            "of R_base_tcp. The 195 mm translation changes the TCP origin; "
            "it does not by itself create a cumulative motion."
        ),
    }


def _deployment_report(rows):
    if not rows:
        return {"count": 0}
    local = np.asarray([row["local_delta"] for row in rows])
    base = np.asarray([row["base_delta"] for row in rows])
    contribution = np.asarray([row["base_delta_axis_contribution"] for row in rows])
    return {
        "count": len(rows),
        "action_index_counts": dict(Counter(row["action_index"] for row in rows)),
        "first": {
            "prediction_sequence": rows[0]["prediction_sequence"],
            "local_tcp_delta_mm": _round_vec(local[0] * 1000, 3),
            "base_delta_mm": _round_vec(base[0] * 1000, 3),
            "base_z_contribution_from_local_xyz_mm": _round_vec(contribution[0] * 1000, 3),
        },
        "last": {
            "prediction_sequence": rows[-1]["prediction_sequence"],
            "local_tcp_delta_mm": _round_vec(local[-1] * 1000, 3),
            "base_delta_mm": _round_vec(base[-1] * 1000, 3),
            "base_z_contribution_from_local_xyz_mm": _round_vec(contribution[-1] * 1000, 3),
        },
        "local_tcp_delta_mm": _vec_stats(local, 1000.0),
        "mapped_base_delta_mm": _vec_stats(base, 1000.0),
        "base_z_contribution_mm": _vec_stats(contribution, 1000.0),
        "base_z_from_local_xyz_identity_check": {
            "max_abs_error_mm": float(np.max(np.abs(base[:, 2] - contribution.sum(axis=1)) * 1000)),
        },
        "note": (
            "The recovered local delta is computed from the recorded target TCP "
            "and the recorded current flange pose. It is therefore a mapping "
            "diagnostic, not a claim that the physical arm reached the target."
        ),
    }


def _load_training_stats(dataset_path, episode, action_index, downsample_steps):
    if dataset_path is None:
        return None
    try:
        import zarr
    except ImportError as exc:
        raise RuntimeError("The UMI environment is required for --dataset (zarr is missing)") from exc
    with zarr.ZipStore(str(dataset_path), mode="r") as store:
        root = zarr.open(store, mode="r")
        ends = np.asarray(root["meta/episode_ends"][:], dtype=int)
        pos = np.asarray(root["data/robot0_eef_pos"][:], dtype=float)
        rotvec = np.asarray(root["data/robot0_eef_rot_axis_angle"][:], dtype=float)
    horizon = int(action_index) * int(downsample_steps)
    local = []
    episode_starts = []
    episode_ends = []
    selected_episodes = range(len(ends)) if episode is None else [episode]
    for ep in selected_episodes:
        if ep < 0 or ep >= len(ends):
            raise ValueError(f"episode must be in [0, {len(ends) - 1}]")
        start = 0 if ep == 0 else int(ends[ep - 1])
        end = int(ends[ep])
        poses = np.tile(np.eye(4), (end - start, 1, 1))
        poses[:, :3, :3] = Rotation.from_rotvec(rotvec[start:end]).as_matrix()
        poses[:, :3, 3] = pos[start:end]
        episode_starts.append(start)
        episode_ends.append(end)
        for i in range(0, len(poses) - horizon):
            delta = np.linalg.inv(poses[i]) @ poses[i + horizon]
            local.append(delta[:3, 3])
    local = np.asarray(local, dtype=float)
    result = {
        "dataset": str(Path(dataset_path).resolve()),
        "episode": "all" if episode is None else int(episode),
        "episodes": int(len(episode_starts)),
        "action_index": int(action_index),
        "downsample_steps": int(downsample_steps),
        "frame_offset": int(horizon),
        "relative_tcp_delta_mm": _vec_stats(local, 1000.0),
        "note": (
            "robot0_eef_pos is in the dataset/UMI coordinate frame. The "
            "relative_tcp_delta_mm values are the meaningful comparison to "
            "the recovered policy-local actions; raw z is not Piper base Z."
        ),
    }
    if episode is not None:
        start, end = episode_starts[0], episode_ends[0]
        result.update({
            "frames": int(end - start),
            "raw_tcp_position_start_mm": _round_vec(pos[start] * 1000, 3),
            "raw_tcp_position_end_mm": _round_vec(pos[end - 1] * 1000, 3),
            "raw_tcp_z_start_end_delta_mm": float((pos[end - 1, 2] - pos[start, 2]) * 1000),
            "raw_tcp_z_range_mm": [float(np.min(pos[start:end, 2]) * 1000),
                                    float(np.max(pos[start:end, 2]) * 1000)],
        })
    return result


def _print_summary(report):
    print("Read-only TCP mapping diagnostic; no CAN, enable, or robot command was used.")
    cfg = report["configuration"]
    print(f"Setup: {cfg['setup']}")
    print(f"flange_to_policy_tcp rotation rotvec: {cfg['rotation_rotvec_rad']}")
    print(f"flange_to_policy_tcp translation: {cfg['translation_m']} m")
    axis = report["axis_mapping"]
    if not axis.get("count"):
        print("No recorded flange poses were found; only configuration was loaded.")
    else:
        proj = axis["median_axes"]["base_vertical_projection_of_local_axes"]
        z_axis = axis["median_axes"]["policy_z_axis_in_base"]
        offset = axis["tcp_origin_offset_in_base_m"]["z"]["median"]
        print(f"Median policy axes in Piper base: X={axis['median_axes']['policy_x_axis_in_base']}")
        print(f"                                  Y={axis['median_axes']['policy_y_axis_in_base']}")
        print(f"                                  Z={z_axis}")
        print(f"Base-Z projection of local [X,Y,Z]: {proj}")
        print(f"195 mm offset base-Z component (median): {offset * 1000:.1f} mm")
        print(f"Recovered policy-local action count: {report['deployment']['count']}")
        local = report["deployment"].get("local_tcp_delta_mm", {})
        base = report["deployment"].get("mapped_base_delta_mm", {})
        if local.get("count"):
            print(f"Local action median xyz (mm): "
                  f"{[round(local[a]['median'], 2) for a in 'xyz']}")
            print(f"Mapped base-delta median xyz (mm): "
                  f"{[round(base[a]['median'], 2) for a in 'xyz']}")
    if report.get("training"):
        tr = report["training"]
        rel = tr["relative_tcp_delta_mm"]
        print(f"Training episode {tr['episode']} local delta at action index "
              f"{tr['action_index']} (mm): "
              f"{[round(rel[a]['median'], 2) for a in 'xyz']}")
        print(f"Training local-Z p95: {rel['z']['p95']:.2f} mm")
    print("Interpretation: a fixed 195 mm offset changes the TCP origin; the "
          "direction of local motion is determined by R_base_tcp. Compare the "
          "local action and axis projections before blaming the offset alone.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True,
                        help="Saved deployment run directory; no live robot is opened")
    parser.add_argument("--setup", type=Path,
                        help="Setup YAML; defaults to <run-dir>/setup.yaml")
    parser.add_argument("--dataset", type=Path,
                        help="Optional training dataset.zarr.zip for local-action comparison")
    parser.add_argument("--episode", type=int, default=0)
    parser.add_argument("--all-episodes", action="store_true",
                        help="Aggregate local-action statistics over all training episodes")
    parser.add_argument("--action-index", type=int, default=8,
                        help="Training horizon index to compare; default matches future waypoint")
    parser.add_argument("--downsample-steps", type=int, default=3,
                        help="Training action downsample; UMI default is 3")
    parser.add_argument("--output", type=Path,
                        help="Optional JSON report path; defaults to <run-dir>/tcp_mapping_diagnostic.json")
    args = parser.parse_args()
    run_dir = args.run_dir.expanduser().resolve()
    if not run_dir.is_dir():
        parser.error(f"run directory does not exist: {run_dir}")
    setup_path = (args.setup or (run_dir / "setup.yaml")).expanduser().resolve()
    if not setup_path.is_file():
        parser.error(f"setup YAML does not exist: {setup_path}")
    if args.dataset is not None:
        dataset_path = args.dataset.expanduser().resolve()
        if not dataset_path.is_file():
            parser.error(f"dataset does not exist: {dataset_path}")
    else:
        dataset_path = None
    if args.action_index < 0 or args.downsample_steps <= 0:
        parser.error("action-index must be nonnegative and downsample-steps must be positive")

    setup, tool, translation, rotation = _load_tool(setup_path)
    rows, event_counts = _run_rows(run_dir, tool)
    training_episode = None if args.all_episodes else args.episode
    report = {
        "run_dir": str(run_dir),
        "configuration": {
            "setup": str(setup_path),
            "rotation_rotvec_rad": _round_vec(rotation),
            "translation_m": _round_vec(translation),
            "translation_norm_m": float(np.linalg.norm(translation)),
            "future_waypoint_enabled": setup.get("latency_compensation", {}).get("future_waypoint_enabled"),
        },
        "control_event_counts": dict(event_counts),
        "axis_mapping": _axis_report(rows, tool, translation),
        "deployment": _deployment_report(rows),
        "training": _load_training_stats(dataset_path, training_episode,
                                          args.action_index, args.downsample_steps),
        "safety": {
            "robot_connection_opened": False,
            "can_opened": False,
            "commands_sent": False,
        },
    }
    if report["training"] and report["deployment"].get("local_tcp_delta_mm", {}).get("count"):
        policy_z = report["deployment"]["local_tcp_delta_mm"]["z"]["median"]
        training_z = report["training"]["relative_tcp_delta_mm"]["z"]["median"]
        report["comparison"] = {
            "policy_local_z_median_mm": policy_z,
            "training_local_z_median_mm": training_z,
            "policy_minus_training_local_z_median_mm": policy_z - training_z,
            "interpretation": (
                "A sizeable difference here means the policy is already producing "
                "a different local action before the TCP-to-base mapping."
            ),
        }
    output = (args.output or (run_dir / "tcp_mapping_diagnostic.json")).expanduser().resolve()
    output.write_text(json.dumps(report, indent=2) + "\n")
    _print_summary(report)
    print(f"JSON report: {output}")


if __name__ == "__main__":
    main()
