"""Pure safety and arbitration logic, independent of ROS and hardware."""

from __future__ import annotations

import numpy as np

from .contract import (
    ACTION_DIM,
    JOINT_LOWER,
    JOINT_UPPER,
    MAX_DELTA_PER_TICK,
    MAX_TRACKING_ERROR,
)


class TrackingError(RuntimeError):
    """Report that feedback no longer follows the absolute command cursor."""

    def __init__(self, errors: np.ndarray, limits: np.ndarray) -> None:
        self.errors = errors.copy()
        self.limits = limits.copy()
        super().__init__(
            "tracking error exceeded safety limit: "
            f"errors={self.errors.tolist()}, limits={self.limits.tolist()}"
        )


class SafeTargetController:
    """Interpolate absolute targets from the last command, not live feedback."""

    def __init__(self, max_delta_scale: float = 1.0) -> None:
        if not 0.0 < max_delta_scale <= 1.0:
            raise ValueError("max_delta_scale must be in (0, 1]")
        self.max_delta = MAX_DELTA_PER_TICK * float(max_delta_scale)
        self.max_tracking_error = MAX_TRACKING_ERROR.copy()
        self.target: np.ndarray | None = None
        self.command_cursor: np.ndarray | None = None

    def clear(self) -> None:
        self.target = None
        self.command_cursor = None

    def set_target(self, raw_target, feedback) -> tuple[np.ndarray, bool]:
        raw_target = np.asarray(raw_target, dtype=np.float32)
        feedback = np.asarray(feedback, dtype=np.float32)
        if raw_target.shape != (ACTION_DIM,) or feedback.shape != (ACTION_DIM,):
            raise ValueError("target and feedback must be seven-dimensional")
        if not np.isfinite(raw_target).all() or not np.isfinite(feedback).all():
            raise ValueError("target and feedback must be finite")
        safe = np.clip(raw_target, JOINT_LOWER, JOINT_UPPER)
        clipped = not np.array_equal(safe, raw_target)
        self.target = safe
        if self.command_cursor is None:
            self.command_cursor = feedback.copy()
        return safe.copy(), clipped

    def tick(
        self,
        feedback,
        *,
        advance: bool = True,
        enforce_tracking: bool = True,
    ) -> np.ndarray | None:
        if self.target is None:
            return None
        feedback = np.asarray(feedback, dtype=np.float32)
        if feedback.shape != (ACTION_DIM,) or not np.isfinite(feedback).all():
            raise ValueError("feedback must be a finite seven-dimensional vector")
        if self.command_cursor is None:
            self.command_cursor = feedback.copy()
        errors = np.abs(self.command_cursor - feedback)
        if enforce_tracking and np.any(errors > self.max_tracking_error):
            raise TrackingError(errors, self.max_tracking_error)
        if advance:
            self.command_cursor += np.clip(
                self.target - self.command_cursor,
                -self.max_delta,
                self.max_delta,
            )
        return self.command_cursor.copy()


class Deadman:
    """Require a repeatedly refreshed positive enable heartbeat."""

    def __init__(self, timeout_s: float = 0.25) -> None:
        if timeout_s <= 0.0:
            raise ValueError("timeout must be positive")
        self.timeout_s = timeout_s
        self.last_positive: float | None = None

    def update(self, enabled: bool, now: float) -> None:
        self.last_positive = float(now) if enabled else None

    def active(self, now: float) -> bool:
        if self.last_positive is None:
            return False
        age = float(now) - self.last_positive
        return 0.0 <= age <= self.timeout_s
