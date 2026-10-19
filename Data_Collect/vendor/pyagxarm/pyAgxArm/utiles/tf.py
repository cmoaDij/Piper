import math
from typing import Tuple, List

# Define Euler angle order encoding table
_AXES2TUPLE = {
    'sxyz': (0, 0, 0, 0),
    'rzyx': (0, 0, 0, 1),
}
axes = 'sxyz'
# For axis index calculation
_NEXT_AXIS = [1, 2, 0, 1]
# Set floating-point comparison error threshold
_EPS = 1e-10


def normalize_quat(qx, qy, qz, qw):
    s = qx * qx + qy * qy + qz * qz + qw * qw
    inv = 1.0 / math.sqrt(s)
    return qx * inv, qy * inv, qz * inv, qw * inv


def _rot9_from_quat(qx: float, qy: float, qz: float, qw: float) -> List[float]:
    """Rotation matrix R (row-major 9 floats) from unit quaternion."""
    xx = qx * qx
    yy = qy * qy
    zz = qz * qz
    xy = qx * qy
    xz = qx * qz
    yz = qy * qz
    xw = qx * qw
    yw = qy * qw
    zw = qz * qw
    return [
        1.0 - 2.0 * (yy + zz),
        2.0 * (xy - zw),
        2.0 * (xz + yw),
        2.0 * (xy + zw),
        1.0 - 2.0 * (xx + zz),
        2.0 * (yz - xw),
        2.0 * (xz - yw),
        2.0 * (yz + xw),
        1.0 - 2.0 * (xx + yy),
    ]


def _euler_from_rot9_generic(
    m: List[float],
    i: int,
    j: int,
    k: int,
    repetition: int,
    frame: int,
    parity: int,
) -> Tuple[float, float, float]:
    """Euler (ax,ay,az) from row-major 3×3 *m* and axis config (original logic)."""

    def M(row: int, col: int) -> float:
        return m[row * 3 + col]

    if repetition:
        sy = math.sqrt(M(i, j) ** 2 + M(i, k) ** 2)
        if sy > _EPS:
            ax = math.atan2(M(i, j), M(i, k))
            ay = math.atan2(sy, M(i, i))
            az = math.atan2(M(j, i), -M(k, i))
        else:
            ax = math.atan2(-M(j, k), M(j, j))
            ay = math.atan2(sy, M(i, i))
            az = 0.0
    else:
        cy = math.sqrt(M(i, i) ** 2 + M(j, i) ** 2)
        if cy > _EPS:
            ax = math.atan2(M(k, j), M(k, k))
            ay = math.atan2(-M(k, i), cy)
            az = math.atan2(M(j, i), M(i, i))
        else:
            ax = math.atan2(-M(j, k), M(j, j))
            ay = math.atan2(-M(k, i), cy)
            az = 0.0

    if parity:
        ax, ay, az = -ax, -ay, -az
    if frame:
        ax, az = az, ax
    return ax, ay, az


def quat_convert_euler(qx: float, qy: float, qz: float, qw: float) -> Tuple[float, float, float]:
    """
    Convert quaternion [x, y, z, w] to Euler angles (roll, pitch, yaw).

    Parameters:
        x, y, z, w - Quaternion components
        axes - Euler angle axis order, supports 'sxyz' or 'rzyx'

    Returns:
        tuple(float, float, float): Euler angles (roll, pitch, yaw) in radians
    """
    try:
        firstaxis, parity, repetition, frame = _AXES2TUPLE[axes.lower()]
    except (KeyError, AttributeError):
        raise ValueError(
            f"Unsupported axes specification: '{axes}'. "
            f"Only 'sxyz' and 'rzyx' are currently supported."
        )
    i = firstaxis
    j = _NEXT_AXIS[i + parity]
    k = _NEXT_AXIS[i - parity + 1]

    qx, qy, qz, qw = normalize_quat(qx, qy, qz, qw)
    m = _rot9_from_quat(qx, qy, qz, qw)

    # Default ``axes='sxyz'``: fixed indices i=0,j=1,k=2, no repetition — unrolled hot path
    if repetition == 0 and frame == 0 and parity == 0 and i == 0 and j == 1 and k == 2:
        m00 = m[0]
        m10, m11, m12 = m[3], m[4], m[5]
        m20, m21, m22 = m[6], m[7], m[8]
        cy = math.sqrt(m00 * m00 + m10 * m10)
        if cy > _EPS:
            ax = math.atan2(m21, m22)
            ay = math.atan2(-m20, cy)
            az = math.atan2(m10, m00)
        else:
            ax = math.atan2(-m12, m11)
            ay = math.atan2(-m20, cy)
            az = 0.0
        return ax, ay, az

    return _euler_from_rot9_generic(m, i, j, k, repetition, frame, parity)


def euler_convert_quat(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """
    Convert Euler angles (roll, pitch, yaw) to quaternion.

    Parameters:
        roll  - Rotation angle around X-axis (in radians)
        pitch - Rotation angle around Y-axis (in radians)
        yaw   - Rotation angle around Z-axis (in radians)

    Returns:
        Quaternion [x, y, z, w]
    """
    try:
        firstaxis, parity, repetition, frame = _AXES2TUPLE[axes.lower()]
    except (KeyError, AttributeError):
        raise ValueError(
            f"Unsupported axes specification: '{axes}'. "
            f"Only 'sxyz' and 'rzyx' are currently supported."
        )
    i = firstaxis
    j = _NEXT_AXIS[i + parity]
    k = _NEXT_AXIS[i - parity + 1]

    if frame:
        roll, yaw = yaw, roll
    if parity:
        pitch = -pitch

    roll *= 0.5
    pitch *= 0.5
    yaw *= 0.5

    c_roll = math.cos(roll)
    s_roll = math.sin(roll)
    c_pitch = math.cos(pitch)
    s_pitch = math.sin(pitch)
    c_yaw = math.cos(yaw)
    s_yaw = math.sin(yaw)

    cc = c_roll * c_yaw
    cs = c_roll * s_yaw
    sc = s_roll * c_yaw
    ss = s_roll * s_yaw

    # ``sxyz`` only: i=0,j=1,k=2, repetition=0, frame=0 (``rzyx`` uses frame swap)
    if repetition == 0 and frame == 0 and i == 0 and j == 1 and k == 2:
        qx = c_pitch * sc - s_pitch * cs
        qy = c_pitch * ss + s_pitch * cc
        qz = c_pitch * cs - s_pitch * sc
        qw = c_pitch * cc + s_pitch * ss
        if parity:
            qy = -qy
        return qx, qy, qz, qw

    qi = 0.0
    qj = 0.0
    qk = 0.0
    qw = 0.0
    if repetition:
        qi = c_pitch * (cs + sc)
        qj = s_pitch * (cc + ss)
        qk = s_pitch * (cs - sc)
        qw = c_pitch * (cc - ss)
    else:
        qi = c_pitch * sc - s_pitch * cs
        qj = c_pitch * ss + s_pitch * cc
        qk = c_pitch * cs - s_pitch * sc
        qw = c_pitch * cc + s_pitch * ss

    out = [0.0, 0.0, 0.0, 0.0]
    out[i] = qi
    out[j] = qj
    out[k] = qk
    out[3] = qw
    if parity:
        out[j] *= -1.0
    return out[0], out[1], out[2], out[3]


def _wrap_angle_pi(angle: float) -> float:
    """Map angle to (-π, π] (replaces per-call nested while loops)."""
    return math.atan2(math.sin(angle), math.cos(angle))


def _clamp_scalar(val: float, lo: float, hi: float) -> float:
    if val > hi:
        return hi
    if val < lo:
        return lo
    return val


def quat_to_euler(q: list, epsilon_deg: float = 0.01) -> Tuple[float, float, float]:
    """
    Convert quaternion to Euler angles using external rotation XYZ order.

    This function converts a quaternion [x, y, z, w] to Euler angles (roll, pitch, yaw)
    while avoiding singularities at the critical points (gimbal lock).

    Parameters
    ----------
    q : tuple or list
        Quaternion in format [x, y, z, w]
    epsilon_deg : float, optional
        Distance from critical points in degrees (default: 0.01)

    Returns
    -------
    tuple
        Euler angles (roll, pitch, yaw) in radians with range limits:
        - roll: (-π + epsilon, π - epsilon)
        - pitch: (-π/2 + epsilon, π/2 - epsilon)
        - yaw: (-π + epsilon, π - epsilon)

    Notes
    -----
    The function handles singularity conditions by limiting angles near the
    critical points (±π/2 for pitch) to avoid numerical instability.

    Examples
    --------
    >>> quat_to_euler([0, 0, 0, 1])
    (0.0, 0.0, 0.0)

    >>> quat_to_euler([0, 0, 0.7071, 0.7071], epsilon_deg=0.5)
    (0.0, 0.0, 1.57075)
    """
    x, y, z, w = q
    epsilon = math.radians(epsilon_deg)

    xx = x * x
    yy = y * y
    zz = z * z
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (xx + yy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (yy + zz)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    pitch_limit = math.pi / 2 - epsilon
    pitch = _clamp_scalar(pitch, -pitch_limit, pitch_limit)

    roll = _wrap_angle_pi(roll)
    yaw = _wrap_angle_pi(yaw)

    roll_yaw_limit = math.pi - epsilon
    roll = _clamp_scalar(roll, -roll_yaw_limit, roll_yaw_limit)
    yaw = _clamp_scalar(yaw, -roll_yaw_limit, roll_yaw_limit)

    return roll, pitch, yaw


# -------------------------- pose6 / rigid transforms (row-major 4×4, 16 floats) ------
# Homogeneous transforms use **T16** only (no nested 4×4 lists).

def _zyx_rpy_to_rotation_elems(
    roll: float, pitch: float, yaw: float
) -> Tuple[float, float, float, float, float, float, float, float, float]:
    """R = Rz(yaw) Ry(pitch) Rx(roll); return 9 matrix entries row-major."""
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)
    r00 = cy * cp
    r01 = cy * sp * sr - sy * cr
    r02 = cy * sp * cr + sy * sr
    r10 = sy * cp
    r11 = sy * sp * sr + cy * cr
    r12 = sy * sp * cr - cy * sr
    r20 = -sp
    r21 = cp * sr
    r22 = cp * cr
    return r00, r01, r02, r10, r11, r12, r20, r21, r22


def _rot_to_rpy_9(
    r00: float,
    r01: float,
    r02: float,
    r10: float,
    r11: float,
    r12: float,
    r20: float,
    r21: float,
    r22: float,
) -> List[float]:
    """Inverse of ZYX rotation; same as ``rot_to_rpy`` without nested list."""
    pitch = math.asin(max(-1.0, min(1.0, -r20)))
    cp = math.cos(pitch)
    eps = 1e-9
    if abs(cp) < eps:
        roll = 0.0
        yaw = math.atan2(-r01, r11)
    else:
        roll = math.atan2(r21, r22)
        yaw = math.atan2(r10, r00)
    return [roll, pitch, yaw]


def rpy_to_rot(roll: float, pitch: float, yaw: float) -> List[List[float]]:
    """Rotation matrix for roll-pitch-yaw (ZYX order): R = Rz(yaw) * Ry(pitch) * Rx(roll)."""
    r00, r01, r02, r10, r11, r12, r20, r21, r22 = _zyx_rpy_to_rotation_elems(
        roll, pitch, yaw
    )
    return [
        [r00, r01, r02],
        [r10, r11, r12],
        [r20, r21, r22],
    ]


def rot_to_rpy(R: List[List[float]]) -> List[float]:
    """Inverse of rpy_to_rot for ZYX order. Returns [roll, pitch, yaw]."""
    return _rot_to_rpy_9(
        R[0][0],
        R[0][1],
        R[0][2],
        R[1][0],
        R[1][1],
        R[1][2],
        R[2][0],
        R[2][1],
        R[2][2],
    )


def matmul16_to(dst: List[float], a: List[float], b: List[float]) -> None:
    """``dst = a @ b`` (4×4 row-major). *dst* must not alias *a* or *b*."""
    for i in range(4):
        o = i * 4
        ai0 = a[o]
        ai1 = a[o + 1]
        ai2 = a[o + 2]
        ai3 = a[o + 3]
        dst[o] = ai0 * b[0] + ai1 * b[4] + ai2 * b[8] + ai3 * b[12]
        dst[o + 1] = ai0 * b[1] + ai1 * b[5] + ai2 * b[9] + ai3 * b[13]
        dst[o + 2] = ai0 * b[2] + ai1 * b[6] + ai2 * b[10] + ai3 * b[14]
        dst[o + 3] = ai0 * b[3] + ai1 * b[7] + ai2 * b[11] + ai3 * b[15]


def pose6_to_T16_into(out: List[float], pose: List[float]) -> None:
    """Write pose6 as 4×4 homogeneous transform into *out* (length ≥ 16, row-major)."""
    x, y, z, roll, pitch, yaw = pose
    r00, r01, r02, r10, r11, r12, r20, r21, r22 = _zyx_rpy_to_rotation_elems(
        roll, pitch, yaw
    )
    out[0] = r00
    out[1] = r01
    out[2] = r02
    out[3] = x
    out[4] = r10
    out[5] = r11
    out[6] = r12
    out[7] = y
    out[8] = r20
    out[9] = r21
    out[10] = r22
    out[11] = z
    out[12] = 0.0
    out[13] = 0.0
    out[14] = 0.0
    out[15] = 1.0


def pose6_to_T16(pose: List[float]) -> List[float]:
    """pose6 → new row-major 4×4 (16 floats)."""
    out = [0.0] * 16
    pose6_to_T16_into(out, pose)
    return out


def inv_T16(T: List[float]) -> List[float]:
    """Inverse of rigid 4×4 (row-major); returns a new 16-float list."""
    r00, r01, r02 = T[0], T[1], T[2]
    r10, r11, r12 = T[4], T[5], T[6]
    r20, r21, r22 = T[8], T[9], T[10]
    tx, ty, tz = T[3], T[7], T[11]
    invx = -(r00 * tx + r10 * ty + r20 * tz)
    invy = -(r01 * tx + r11 * ty + r21 * tz)
    invz = -(r02 * tx + r12 * ty + r22 * tz)
    return [
        r00,
        r10,
        r20,
        invx,
        r01,
        r11,
        r21,
        invy,
        r02,
        r12,
        r22,
        invz,
        0.0,
        0.0,
        0.0,
        1.0,
    ]


def T16_to_pose6(t: List[float]) -> List[float]:
    """Row-major homogeneous 4×4 (16 floats) → pose6 [x,y,z,roll,pitch,yaw]."""
    x, y, z = t[3], t[7], t[11]
    rpy = _rot_to_rpy_9(
        t[0], t[1], t[2], t[4], t[5], t[6], t[8], t[9], t[10]
    )
    return [x, y, z, rpy[0], rpy[1], rpy[2]]
