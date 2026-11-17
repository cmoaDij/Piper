"""Create a tiny simulated recording using the production writer and codec."""

import copy
import numpy as np

from data_collect.common.config import IMAGE_STREAMS, image_streams, local_path
from data_collect.control.kinematics import ForwardKinematics
from data_collect.recording.storage import EpisodeWriter
from data_collect.calibration.tracking import response_fields


def write_episode(directory, config, *, rejected=False, paused=False, moving=False):
    directory.mkdir(parents=True, exist_ok=True)
    cfg = copy.deepcopy(config)
    for camera in cfg["cameras"].values():
        camera.update(width=16, height=16, fps=20)
    cfg["storage"]["video_encoder"] = "libx264"
    cfg["storage"]["batch_ms"] = 1
    cfg["storage"]["batch_max_items"] = 16
    fk = ForwardKinematics(local_path(cfg["urdf"]), tcp_offset=cfg["tcp_offset"])
    metadata = {"start_host_ts_ns": 1_000_000_000, "task_instruction": "synthetic offline test",
                "simulated": True, "config": cfg}
    if paused:
        metadata["recording_mode"] = "grip_paused"
    baselines = {s: np.zeros((8, 8), dtype=np.uint8) for s in IMAGE_STREAMS if s.startswith("tactile/")}
    path = directory / "episode_0001.h5"
    writer = EpisodeWriter(path, metadata, baselines=baselines, fk=fk)
    try:
        for i in range(20):
            ts = np.int64(1_000_000_000 + i * 50_000_000)
            if paused:
                if i == 5:
                    writer.set_recording(False, ts)
                if i >= 10:
                    ts += 2_000_000_000
                if i == 10:
                    writer.set_recording(True, ts)
            for stream in image_streams(cfg):
                shape = (16, 16, 3) if stream.startswith("cameras/") else (8, 8)
                record = {"available_host_ts_ns": ts, "frame_id": np.int64(i),
                          "data": np.full(shape, i, dtype=np.uint8)}
                if stream.startswith("cameras/"):
                    camera = cfg["cameras"][stream.split("/")[-1]]
                    if camera.get("backend") == "gopro":
                        record["capture_host_ts_ns"] = ts - camera["receive_latency_ns"]
                assert writer.submit(stream, record)
            joints = np.full(6, i * .001) if moving else np.zeros(6)
            feedback = {"available_host_ts_ns": ts, "joint_pos": joints,
                        "gripper_pos": np.array([.06]), "tcp_pose": fk(joints),
                        "is_new_packet": np.bool_(True)}
            for part in ("joint", "tcp", "gripper"):
                feedback[part + "_available_host_ts_ns"] = ts
                feedback["sdk_" + part + "_ts"] = np.float64(ts / 1e9)
            assert writer.submit("robot_feedback", feedback)
            command = {"cmd_call_begin_ns": ts, "cmd_call_end_ns": ts + 1,
                "cmd_seq": np.int64(i + 1), "target_joint_pos": joints, "target_gripper_pos": np.array([.06]),
                "following": np.bool_(not 5 <= i < 10)}
            if cfg.get("robot_tracking_latency"):
                command.update(response_fields(ts, ts, cfg["robot_tracking_latency"]))
            assert writer.submit("robot_command", command)
    finally:
        writer.finish(4_000_000_000 if paused else 2_000_000_000,
                      "synthetic rejection" if rejected else None, rejected=rejected)
        writer.join()
    assert writer.error is None, writer.error
    return path
