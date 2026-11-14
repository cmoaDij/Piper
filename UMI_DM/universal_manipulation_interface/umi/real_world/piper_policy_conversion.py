"""Pure conversion helpers for a future Piper-X policy bridge.

This module contains no ROS, CAN, robot enable or command-sending code. A
verified flange-to-policy-TCP transform and the existing controller's motion
and emergency-stop guards are still required before any hardware control.
"""
from dataclasses import dataclass
import math

import numpy as np
from scipy.spatial.transform import Rotation

from umi.common.pose_util import rot6d_to_mat


@dataclass(frozen=True)
class GripperPolicyLimits:
    feedback_offset_m: float = 0.0
    training_min_m: float = 0.0
    training_max_m: float = 0.076
    controller_min_m: float = 0.0
    controller_max_m: float = 0.070
    max_force_parameter_n: float = 1.0

    def __post_init__(self):
        fields = (self.feedback_offset_m, self.training_min_m, self.training_max_m,
                  self.controller_min_m, self.controller_max_m, self.max_force_parameter_n)
        if not all(math.isfinite(x) for x in fields):
            raise ValueError('Gripper configuration must be finite')
        if self.training_min_m < 0 or self.training_min_m >= self.training_max_m:
            raise ValueError('Invalid training gripper range')
        if self.controller_min_m < 0 or self.controller_min_m >= self.controller_max_m:
            raise ValueError('Invalid controller gripper range')
        if self.max_force_parameter_n <= 0 or self.max_force_parameter_n > 1.0:
            raise ValueError('Configured force parameter cap must be in (0, 1.0] N')


def physical_width_from_feedback(feedback_m, limits=GripperPolicyLimits()):
    """Convert raw feedback to the model's physical opening.

    Return (width, was_clamped). A configured offset may be used for legacy
    hardware, but the current deployment profile uses zero offset.
    Reject readings above the nominal model domain; do not conceal them.
    """
    value = float(feedback_m)
    if not math.isfinite(value):
        raise ValueError('Invalid gripper feedback')
    # The Piper width sensor can report a small negative raw value at the
    # mechanical closed stop.  Apply the configured offset first, then clamp
    # the physical opening to zero.  Keep rejecting values above the training
    # range because those cannot be represented by the policy.
    width = value - limits.feedback_offset_m
    if width > limits.training_max_m:
        raise ValueError('Gripper feedback implies width beyond model range')
    return max(limits.training_min_m, width), width < limits.training_min_m


def check_gripper_target(physical_width_m, force_n, limits=GripperPolicyLimits()):
    """Convert a proposed width to raw controller units; reject, never clip.

    A returned value is NOT permission to command hardware: the provisional
    mapping, actual clearance, force semantics and motion guards must be
    validated at the robot's sole control outlet.
    """
    width = float(physical_width_m)
    force = float(force_n)
    if not math.isfinite(width) or not limits.training_min_m <= width <= limits.training_max_m:
        raise ValueError('Model gripper width outside nominal physical range')
    if not math.isfinite(force) or not 0 < force <= limits.max_force_parameter_n:
        raise ValueError('Requested gripper force parameter exceeds allowed range')
    target = width + limits.feedback_offset_m
    if not limits.controller_min_m <= target <= limits.controller_max_m:
        raise ValueError('Converted gripper target outside current controller limits')
    return target, force


def _rigid_transform(matrix, name):
    x = np.asarray(matrix, dtype=np.float64)
    if x.shape != (4, 4) or not np.isfinite(x).all():
        raise ValueError(f'{name} must be a finite 4x4 matrix')
    r = x[:3, :3]
    if (not np.allclose(x[3], [0, 0, 0, 1], atol=1e-7)
            or not np.allclose(r.T @ r, np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(r), 1, atol=1e-6)):
        raise ValueError(f'{name} must be a rigid transform')
    return x


def relative_policy_action_to_flange(base_flange, flange_policy_tcp,
                                     relative_action, *, tcp_verified=False):
    """Return a base-frame flange target as XYZ (m) and extrinsic XYZ RPY (rad).

    Policy action is relative to the current model TCP, represented as
    xyz + row-based rotation-6D. The 19.5 cm measured translation alone does
    not establish the TCP orientation, so ``tcp_verified`` must be explicit.
    This only computes geometry and does not check collision, IK or timing.
    """
    if not tcp_verified:
        raise ValueError('Full flange-to-model-TCP pose has not been verified')
    base_flange = _rigid_transform(base_flange, 'base_flange')
    flange_tcp = _rigid_transform(flange_policy_tcp, 'flange_policy_tcp')
    action = np.asarray(relative_action, dtype=float)
    if action.shape != (9,) or not np.isfinite(action).all():
        raise ValueError('Expected finite xyz and rotation-6D')
    a1, a2 = action[3:6], action[6:9]
    if np.linalg.norm(a1) < 1e-6 or np.linalg.norm(np.cross(a1, a2)) < 1e-6:
        raise ValueError('Degenerate rotation-6D')
    delta = np.eye(4)
    delta[:3, :3] = rot6d_to_mat(action[3:9])
    delta[:3, 3] = action[:3]
    target_flange = base_flange @ flange_tcp @ delta @ np.linalg.inv(flange_tcp)
    return np.r_[target_flange[:3, 3], Rotation.from_matrix(target_flange[:3, :3]).as_euler('xyz')]
