"""Joint-space admission checks and a bounded velocity/acceleration trajectory."""

import numpy as np


class MotionLimiter:
    def __init__(self, limits, rules):
        self.lower = np.array([limits[f"joint{i}"][0] for i in range(1, 7)], dtype=float)
        self.upper = np.array([limits[f"joint{i}"][1] for i in range(1, 7)], dtype=float)
        self.rules = rules
        self.position = self.velocity = self.request = None
        self.time_ns = None

    def reset(self, actual, now_ns):
        actual = self._vector(actual)
        if np.any(actual < self.lower - 1e-6) or np.any(actual > self.upper + 1e-6):
            raise ValueError("实测关节角超出模型限位，请核对零位并在限位内准备起始姿态；不会自动移动到边界")
        self.position = actual.copy()
        self.request = actual.copy()
        self.velocity = np.zeros(6)
        self.time_ns = int(now_ns)

    @staticmethod
    def _vector(value):
        value = np.asarray(value, dtype=float)
        if value.shape != (6,) or not np.isfinite(value).all():
            raise ValueError("运动目标必须为有限六轴关节角")
        return value

    def check_first(self, target, actual):
        delta = np.abs(self._vector(target) - self._vector(actual))
        if np.max(delta) > self.rules["first_target_tolerance_rad"]:
            raise ValueError(f"首条目标跳变 {np.rad2deg(delta.max()):.2f}°，超过启动门限，拒绝开始")

    def step(self, target, actual, now_ns):
        target, actual = self._vector(target), self._vector(actual)
        if self.position is None:
            raise ValueError("尚未完成 A 键参考对齐")
        dt = (int(now_ns) - self.time_ns) / 1e9
        if not 0 < dt <= self.rules["max_control_gap_s"]:
            raise ValueError("控制周期中断，请 B 停止后重新对齐")
        if np.any(target < self.lower) or np.any(target > self.upper):
            raise ValueError("关节目标超出 URDF 限位")
        if np.max(np.abs(target - self.request)) > self.rules["max_target_step_rad"]:
            raise ValueError("连续 IK 目标发生大幅跳变，拒绝跟踪")
        if np.max(np.abs(self.position - actual)) > self.rules["max_tracking_error_rad"]:
            raise ValueError("实际关节未跟上已发送轨迹，停止下发")
        acceleration = self.rules["max_acceleration_rad_s2"]
        speed = min(self.rules["max_velocity_rad_s"], self.rules["max_command_step_rad"] / dt)
        # Keep the queued goal close to the measured arm, even when the hand
        # keeps moving faster than the robot can follow.
        lower = np.maximum(self.lower, actual - self.rules["max_target_lead_rad"])
        upper = np.minimum(self.upper, actual + self.rules["max_target_lead_rad"])
        bounded_target = np.clip(target, lower, upper)
        error = bounded_target - self.position
        # Reserve one discrete step of braking distance; the continuous
        # sqrt(2*a*d) bound alone overshoots small targets at finite sample rates.
        def braking_speed(distance):
            return np.sqrt((acceleration * dt) ** 2 + 2 * acceleration * np.maximum(distance, 0)) - acceleration * dt

        desired = np.sign(error) * np.minimum(speed, braking_speed(np.abs(error)))
        velocity = np.clip(desired, self.velocity - acceleration * dt, self.velocity + acceleration * dt)
        velocity = np.clip(velocity, -speed, speed)
        velocity = np.clip(velocity, -braking_speed(self.position - lower), braking_speed(upper - self.position))
        position = self.position + velocity * dt
        if np.any(position < lower - 1e-6) or np.any(position > upper + 1e-6):
            raise ValueError("轨迹即将越过关节限位或实测位置附近的跟随范围，拒绝下发")
        if np.max(np.abs(velocity - self.velocity)) > acceleration * dt + 1e-9:
            raise ValueError("无法同时满足速度和加速度限制，拒绝下发")
        self.position, self.velocity = position, velocity
        self.request, self.time_ns = target.copy(), int(now_ns)
        return position.tolist()


def check_feedback_motion(history, rules):
    """Check distinct joint packets; cached reads do not imply zero velocity.

    Use absolute travel per axis so fast back-and-forth movement cannot cancel
    itself out. Include the fraction of the segment crossing the window edge.
    """
    packets = []
    for record in history:
        stamp = int(record["joint_available_host_ts_ns"])
        if packets and stamp == packets[-1][0]:
            continue
        if packets and stamp < packets[-1][0]:
            raise ValueError("关节反馈时间回退，停止运动")
        packets.append((stamp, MotionLimiter._vector(record["joint_pos"])))
    if len(packets) < 2:
        return
    window_ns = rules["feedback_motion_window_s"] * 1e9
    cutoff = packets[-1][0] - window_ns
    travel = np.zeros(6)
    start = packets[-1][0]
    for (t0, q0), (t1, q1) in zip(packets, packets[1:]):
        if t1 <= cutoff:
            continue
        delta = np.abs(q1 - q0)
        if delta.max() > rules["max_feedback_step_rad"] + 1e-9:
            joint = int(np.argmax(delta)) + 1
            raise ValueError(f"关节{joint}实测位置单包跳变 {np.rad2deg(delta.max()):.2f}°，停止运动")
        if (t1 - t0) / 1e9 > rules["max_control_gap_s"]:
            raise ValueError("关节反馈采样中断，无法确认运动速度，停止运动")
        start = min(start, max(t0, cutoff))
        travel += delta * (t1 - max(t0, cutoff)) / (t1 - t0)
    duration = (packets[-1][0] - start) / 1e9
    if duration >= rules["feedback_motion_window_s"] * 0.8:
        limit = rules["max_feedback_window_travel_rad"] * duration / rules["feedback_motion_window_s"]
        if travel.max() > limit + 1e-9:
            joint = int(np.argmax(travel)) + 1
            raise ValueError(f"关节{joint}在 {duration * 1000:.0f} ms 内实测累计变化 {np.rad2deg(travel.max()):.2f}°，超过速度/短时位移限制，停止运动")
