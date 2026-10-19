#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Modified Denavit–Hartenberg forward kinematics.

Convention for each link ``i`` (parameters ``d, a, alpha, theta_offset``)::

    ^{i-1}T_i = R_x(alpha) * T_x(a) * R_z(theta) * T_z(d)

with ``theta = q_i + theta_offset`` and joint variable ``q_i`` in radians.

Internal FK uses **row-major 16-float** matrices (one list per 4×4) to avoid
nested lists and cut allocations in the chain multiply.
"""
import math
from typing import List, Tuple

from ..api.constants import ROBOT_MDH_PRESET
from .tf import T16_to_pose6, matmul16_to

MDH = Tuple[float, float, float, float]


def _link_mdh_write_16(
    out: List[float], d: float, a: float, alpha: float, theta: float
) -> None:
    """Fused modified-DH link into *out* (16 floats, row-major). In-place."""
    ca, sa = math.cos(alpha), math.sin(alpha)
    ct, st = math.cos(theta), math.sin(theta)
    out[0] = ct
    out[1] = -st
    out[2] = 0.0
    out[3] = a
    out[4] = ca * st
    out[5] = ca * ct
    out[6] = -sa
    out[7] = -sa * d
    out[8] = sa * st
    out[9] = sa * ct
    out[10] = ca
    out[11] = ca * d
    out[12] = 0.0
    out[13] = 0.0
    out[14] = 0.0
    out[15] = 1.0


def get_mdh(robot: str) -> Tuple[MDH, ...]:
    """Return MDH ``(d, a, alpha, theta_offset)`` for *robot*.

    Parameters are loaded from ``pyAgxArm.api.constants.ROBOT_MDH_PRESET``.
    """
    if robot not in ROBOT_MDH_PRESET:
        raise KeyError(
            f"No MDH table for robot={robot!r}. "
            f"Expected one of: {', '.join(sorted(ROBOT_MDH_PRESET.keys()))}."
        )
    # 返回新的 tuple，避免外部错误修改全局常量
    return tuple(tuple(link) for link in ROBOT_MDH_PRESET[robot])


def fk_from_mdh(
    mdh: List[MDH],
    joint_radians: List[float],
) -> List[float]:
    """FK"""
    n = len(mdh)
    if len(joint_radians) != n:
        raise ValueError(
            f"joint_radians length {len(joint_radians)} != {n} (MDH links)"
        )

    t_acc = [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0]
    t_tmp = [0.0] * 16
    link = [0.0] * 16

    for i in range(n):
        d_i, a_i, alpha, theta_off = mdh[i]
        theta = joint_radians[i] + theta_off
        _link_mdh_write_16(link, d_i, a_i, alpha, theta)
        matmul16_to(t_tmp, t_acc, link)
        t_acc, t_tmp = t_tmp, t_acc

    return T16_to_pose6(t_acc)
