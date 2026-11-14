#!/usr/bin/env python3
"""Stationary, receive-only Piper/GoPro policy diagnostic; never commands hardware."""
import argparse
import json
import os
from pathlib import Path
import socket
import struct
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import cv2
import dill
import hydra
import numpy as np
import torch
import yaml
from omegaconf import OmegaConf
from scipy.spatial.transform import Rotation
from umi.real_world.piper_policy_conversion import (
    GripperPolicyLimits, check_gripper_target, physical_width_from_feedback,
    relative_policy_action_to_flange,
)
from umi.real_world.real_inference_util import get_real_umi_obs_dict
from umi.real_world.piper_camera_geometry import preprocess


class PassiveCan:
    """Collect joint and gripper feedback via a receive-only CAN socket."""
    def __init__(self, interface):
        self.interface = interface
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.samples = []
        self.error = None
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.thread.start()

    def close(self):
        self.stop_event.set()
        self.thread.join(timeout=2)

    def _run(self):
        try:
            sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            sock.bind((self.interface,))
            sock.settimeout(.1)
            data = {}; stamps = {}
            while not self.stop_event.is_set():
                try:
                    frame = sock.recv(16)
                except socket.timeout:
                    continue
                if len(frame) != 16:
                    continue
                cid, length, payload = struct.unpack('=IB3x8s', frame)
                if cid not in range(0x2a1, 0x2a9) or length != 8:
                    continue
                now = time.monotonic()
                data[cid] = payload; stamps[cid] = now
                needed = tuple(range(0x2a1, 0x2a9))
                # Joint feedback is published on 0x2a5..0x2a7.  Use the last
                # joint packet as the sample boundary instead of waiting for
                # the much slower gripper packet.  Waiting for 0x2a8 made the
                # policy observe a mixed, stale state and limited alignment to
                # the gripper report rate.
                if cid != 0x2a7 or not all(k in data and now - stamps[k] < .25 for k in needed):
                    continue
                if data[0x2a8][7] != 0:
                    raise RuntimeError('Gripper feedback is not in width mode')
                if data[0x2a1][1] != 0 or int.from_bytes(data[0x2a1][6:8], 'big') != 0:
                    raise RuntimeError('Robot status indicates an error')
                joints = np.array([v / 1000 for k in (0x2a5, 0x2a6, 0x2a7)
                                   for v in struct.unpack('>ii', data[k])], dtype=float)
                raw_mm = int.from_bytes(data[0x2a8][:4], 'big', signed=True) / 1000
                xy = struct.unpack('>ii', data[0x2a2])
                zrx = struct.unpack('>ii', data[0x2a3])
                ryrz = struct.unpack('>ii', data[0x2a4])
                flange = np.r_[np.array([*xy, zrx[0]]) * 1e-6,
                                np.deg2rad(np.array([zrx[1], *ryrz]) / 1000)]
                joint_time = max(stamps[0x2a5], stamps[0x2a6], stamps[0x2a7])
                with self.lock:
                    self.samples.append((joint_time, joints, raw_mm, flange))
                    self.samples = self.samples[-2000:]
            sock.close()
        except Exception as exc:
            self.error = exc

    def recent(self):
        with self.lock:
            return list(self.samples)


def configure_inference_augmentations(model, disable=False):
    """Remove stochastic image augmentations from the policy encoder.

    The training configuration contains ``RandomCrop`` and ``ColorJitter``.
    They are stored in the encoder's transform modules and torchvision applies
    them in eval mode as well.  Deployment diagnostics can replace only those
    two transforms while retaining any deterministic preprocessing modules.
    """
    if not disable:
        return []
    removed = []
    encoder = getattr(model, 'obs_encoder', None)
    transform_map = getattr(encoder, 'key_transform_map', None)
    if transform_map is None:
        return removed
    for key in list(transform_map.keys()):
        transform = transform_map[key]
        modules = list(transform.children()) if isinstance(transform, torch.nn.Sequential) else [transform]
        kept = []
        for module in modules:
            name = module.__class__.__name__
            if name in {'RandomCrop', 'ColorJitter'}:
                removed.append(f'{key}:{name}')
            else:
                kept.append(module)
        transform_map[key] = torch.nn.Sequential(*kept) if kept else torch.nn.Identity()
    return removed


def load_policy(checkpoint, seed, disable_inference_augmentations=False):
    torch.manual_seed(seed); np.random.seed(seed); torch.set_num_threads(2)
    OmegaConf.register_new_resolver('eval', eval, replace=True)
    payload = torch.load(str(checkpoint), map_location='cpu', pickle_module=dill, mmap=True)
    cfg = payload['cfg']
    cfg.policy.obs_encoder.pretrained = False
    model = hydra.utils.instantiate(cfg.policy)
    model.load_state_dict(payload['state_dicts']['ema_model'], strict=True)
    del payload
    removed_augmentations = []
    if disable_inference_augmentations:
        removed_augmentations = configure_inference_augmentations(
            model, disable=True)
    if not torch.cuda.is_available():
        raise RuntimeError('CUDA unavailable; see deployment GPU runtime settings')
    model.eval().to('cuda')
    model._piper_removed_inference_augmentations = tuple(removed_augmentations)
    return cfg, model


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--checkpoint', type=Path, required=True)
    p.add_argument('--mask', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--setup', type=Path, default=None,
                   help='Optional user-provided flange-to-model-TCP geometry for preview only')
    p.add_argument('--camera', default='/dev/video0')
    p.add_argument('--can', default='can0')
    p.add_argument('--seed', type=int, default=1234)
    p.add_argument('--max-joint-spread-deg', type=float, default=.1)
    p.add_argument('--disable-inference-augmentations', action='store_true',
                   help='Disable RandomCrop and ColorJitter during inference')
    args = p.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    mask = json.loads(args.mask.read_text())
    setup = yaml.safe_load(args.setup.read_text()) if args.setup is not None else {}
    camera_image_geometry = setup.get('camera_image_geometry', 'native_4_3')
    gripper_limits = GripperPolicyLimits()
    print('Loading model; no robot command is sent.', flush=True)
    cfg, model = load_policy(
        args.checkpoint,
        args.seed,
        disable_inference_augmentations=args.disable_inference_augmentations)
    if args.disable_inference_augmentations:
        print('Inference augmentations disabled: '
              f'{list(model._piper_removed_inference_augmentations)}', flush=True)
    bus = PassiveCan(args.can); bus.start()
    cap = None
    try:
        cap = cv2.VideoCapture(args.camera, cv2.CAP_V4L2)
        if not cap.isOpened():
            raise RuntimeError('Cannot open camera; close other capture applications first')
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)
        cap.set(cv2.CAP_PROP_FPS, 60); cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        start = time.monotonic(); selected = []
        while time.monotonic() - start < 1.5:
            ok, frame = cap.read()
            if not ok:
                raise RuntimeError('Camera failed to deliver a frame')
            now = time.monotonic()
            if now - start > .8 and (not selected or now - selected[-1][0] >= 3 * 1001 / 60000):
                selected.append((now, frame.copy()))
                if len(selected) == 2:
                    break
        samples = bus.recent()
        if bus.error:
            raise RuntimeError(f'CAN receiver failed: {bus.error}')
        if len(selected) != 2 or len(samples) < 2:
            raise RuntimeError('Insufficient camera or robot feedback')
        interval = [s for s in samples if selected[0][0] - .25 <= s[0] <= selected[1][0]]
        if len(interval) < 2 or selected[-1][0] - interval[-1][0] > .25:
            raise RuntimeError('Robot feedback is missing or stale')
        spread = np.ptp(np.stack([s[1] for s in interval]), axis=0)
        if spread.max() > args.max_joint_spread_deg:
            raise RuntimeError(f'Robot is moving; max joint spread {spread.max():.3f} degrees')
        raw_mm = np.array([min(interval, key=lambda s: abs(s[0]-t))[2] for t, _ in selected])
        corrected_mm = raw_mm - gripper_limits.feedback_offset_m * 1000
        # Preserve raw feedback, then map it to the configured physical range.
        # No target is sent by this stationary diagnostic.
        width_results = [physical_width_from_feedback(raw / 1000, gripper_limits) for raw in raw_mm]
        physical_mm = np.array([width * 1000 for width, _ in width_results])
        processed, detected = zip(*(preprocess(
            frame, mask, camera_image_geometry=camera_image_geometry)
            for _, frame in selected))
        cv2.imwrite(str(args.output/'camera_raw.jpg'), selected[-1][1])
        cv2.imwrite(str(args.output/'model_input_candidate.png'), cv2.cvtColor(processed[-1], cv2.COLOR_RGB2BGR))
        # With the robot stationary and the diagnostic starting at this pose,
        # both pose histories are the identity irrespective of the fixed TCP
        # transform. No flange-to-TCP calibration is assumed or fabricated.
        env_obs = {'camera0_rgb': np.asarray(processed),
                   'robot0_eef_pos': np.zeros((2, 3)),
                   'robot0_eef_rot_axis_angle': np.zeros((2, 3)),
                   'robot0_gripper_width': physical_mm[:, None] / 1000}
        obs = get_real_umi_obs_dict(env_obs, cfg.task.shape_meta,
                                    obs_pose_repr='relative', episode_start_pose=[np.zeros(6)])
        obs = {k: torch.from_numpy(np.asarray(v, dtype=np.float32)).unsqueeze(0).to('cuda') for k, v in obs.items()}
        with torch.inference_mode():
            action = model.predict_action(obs)['action_pred'][0].cpu().numpy()
        if action.shape != (16, 10) or not np.isfinite(action).all():
            raise RuntimeError('Invalid model prediction')
        target_raw_mm = action[:, 9] * 1000 + gripper_limits.feedback_offset_m * 1000
        rejected_targets = []
        for index, width in enumerate(action[:, 9]):
            try:
                check_gripper_target(width, 1.0)
            except ValueError as exc:
                rejected_targets.append({'step': index, 'reason': str(exc)})
        preview_flange = None
        if args.setup is not None:
            tool = setup['flange_to_policy_tcp']
            if not tool.get('rotation_verified') or tool.get('rotation') is None:
                raise RuntimeError('Flange-to-model-TCP orientation has not been provided')
            tcp = np.eye(4)
            tcp[:3, :3] = Rotation.from_rotvec(tool['rotation']).as_matrix()
            tcp[:3, 3] = tool['translation_m']
            current_flange = interval[-1][3]
            base_flange = np.eye(4)
            base_flange[:3, :3] = Rotation.from_euler('xyz', current_flange[3:]).as_matrix()
            base_flange[:3, 3] = current_flange[:3]
            preview_flange = [relative_policy_action_to_flange(base_flange, tcp, step[:9],
                                                               tcp_verified=True).tolist()
                              for step in action]
        result = {'mode': 'receive-only stationary diagnostic; no robot motion commands',
                  'timestamp': time.strftime('%Y-%m-%d %H:%M:%S %Z'),
                  'camera': args.camera, 'frame_size': list(selected[-1][1].shape[1::-1]),
                  'image_interval_s': selected[1][0] - selected[0][0],
                  'image_timestamp_note': 'host dequeue time, not exposure time; HDMI geometry and latency unverified',
                  'camera_signal_verified_by_operator': False,
                  'source_to_training_image_geometry': camera_image_geometry,
                  'detected_finger_tag_ids': list(detected),
                  'robot_max_joint_spread_deg': float(spread.max()),
                  'robot_joint_angles_deg': interval[-1][1].tolist(),
                  'robot_flange_xyz_m_rpy_rad': interval[-1][3].tolist(),
                  'gripper_can_feedback_mm': raw_mm.tolist(),
                  'gripper_can_minus_offset_mm': corrected_mm.tolist(),
                  'gripper_feedback_offset_mm': gripper_limits.feedback_offset_m * 1000,
                  'gripper_model_input_mm': physical_mm.tolist(),
                  'gripper_model_input_clamped_at_zero': bool(np.any(corrected_mm < 0)),
                  'gripper_training_observed_range_mm': [30.862279, 76],
                  'gripper_input_outside_training_observed_range': bool(
                      np.any(physical_mm < 30.862279) or np.any(physical_mm > 76)),
                  'provisional_gripper_inverse_can_targets_mm': target_raw_mm.tolist(),
                  'targets_above_existing_can_limit_70mm': int((target_raw_mm > 70).sum()),
                  'gripper_targets_rejected_by_preflight': rejected_targets,
                  'first_gripper_width_jump_mm': float(abs(action[0, 9] * 1000 - physical_mm[-1])),
                  'max_predicted_tcp_displacement_mm': float(np.linalg.norm(action[:, :3], axis=1).max() * 1000),
                  'requested_gripper_force_parameter_limit_N_for_future_control': 1.0,
                  'force_cap_enforced_on_hardware': False,
                  'flange_to_model_tcp_source': 'user-provided; not independently validated',
                  'prediction_pose_frame': 'relative model TCP frame; mapping to robot base not independently validated',
                  'preview_flange_targets_xyz_m_rpy_rad': preview_flange,
                  'command_path_implemented': False,
                  'prediction_xyz_m': action[:, :3].tolist(),
                  'prediction_gripper_width_m': action[:, 9].tolist(),
                  'prediction_rotation_6d': action[:, 3:9].tolist()}
        (args.output/'stationary_prediction.json').write_text(json.dumps(result, indent=2)+'\n')
        print('Raw CAN gripper mm:', raw_mm.tolist(), 'model input mm:', physical_mm.tolist(), flush=True)
        print('Predicted physical gripper mm:', np.round(action[:, 9]*1000, 2).tolist(), flush=True)
        print('Candidate raw targets above existing 70 mm limit:', result['targets_above_existing_can_limit_70mm'], flush=True)
        print('First predicted gripper jump (mm):', round(result['first_gripper_width_jump_mm'], 2), flush=True)
        print('Maximum predicted TCP displacement (mm):', round(result['max_predicted_tcp_displacement_mm'], 2), flush=True)
        if result['gripper_input_outside_training_observed_range']:
            print('WARNING: current gripper input is outside the widths present in training.', flush=True)
        print('Saved', args.output, flush=True)
    finally:
        if cap is not None:
            cap.release()
        bus.close()


if __name__ == '__main__':
    main()
