"""Create a tiny simulated recording using the production writer and codec."""

import copy
import numpy as np

from data_collect.common.config import IMAGE_STREAMS, local_path
from data_collect.control.kinematics import ForwardKinematics
from data_collect.recording.storage import EpisodeWriter


def write_episode(directory, config, *, rejected=False):
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
    baselines = {s: np.zeros((8, 8), dtype=np.uint8) for s in IMAGE_STREAMS if s.startswith("tactile/")}
    path = directory / "episode_0001.h5"
    writer = EpisodeWriter(path, metadata, baselines=baselines, fk=fk)
    try:
        for i in range(20):
            ts = np.int64(1_000_000_000 + i * 50_000_000)
            for stream in IMAGE_STREAMS:
                shape = (16, 16, 3) if stream.startswith("cameras/") else (8, 8)
                assert writer.submit(stream, {"available_host_ts_ns": ts, "frame_id": np.int64(i),
                                               "data": np.full(shape, i, dtype=np.uint8)})
            feedback = {"available_host_ts_ns": ts, "joint_pos": np.zeros(6),
                        "gripper_pos": np.array([.06]), "tcp_pose": fk(np.zeros(6)),
                        "is_new_packet": np.bool_(True)}
            for part in ("joint", "tcp", "gripper"):
                feedback[part + "_available_host_ts_ns"] = ts
                feedback["sdk_" + part + "_ts"] = np.float64(ts / 1e9)
            assert writer.submit("robot_feedback", feedback)
            assert writer.submit("robot_command", {"cmd_call_begin_ns": ts, "cmd_call_end_ns": ts + 1,
                "cmd_seq": np.int64(i + 1), "target_joint_pos": np.zeros(6), "target_gripper_pos": np.array([.06])})
    finally:
        writer.finish(2_000_000_000, "synthetic rejection" if rejected else None, rejected=rejected)
        writer.join()
    assert writer.error is None, writer.error
    return path
