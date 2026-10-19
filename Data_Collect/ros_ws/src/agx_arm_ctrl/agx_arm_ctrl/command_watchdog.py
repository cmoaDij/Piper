"""Monotonic external-command watchdog for the controller software gate."""

from __future__ import annotations

import math


def complete_command_values(names, positions, required_names):
    """Return finite named positions only for one complete unambiguous command."""
    names = tuple(str(name) for name in names)
    positions = tuple(positions)
    required = tuple(str(name) for name in required_names)
    if (
        not required
        or len(set(required)) != len(required)
        or len(names) != len(positions)
        or len(set(names)) != len(names)
        or set(names) != set(required)
    ):
        return None
    try:
        values = {name: float(value) for name, value in zip(names, positions)}
        result = {name: values[name] for name in required}
    except (KeyError, TypeError, ValueError):
        return None
    return result if all(math.isfinite(value) for value in result.values()) else None


class ExternalCommandWatchdog:
    """Expire an opened control session unless commands keep arriving."""

    def __init__(self, timeout_s: float = 0.25) -> None:
        if not math.isfinite(timeout_s) or timeout_s <= 0.0:
            raise ValueError("external command timeout must be positive")
        self.timeout_s = float(timeout_s)
        self.last_command_time: float | None = None
        self.armed = False

    def open(self, now: float) -> None:
        """Arm the watchdog and grant one timeout interval of startup grace."""
        if not math.isfinite(now):
            raise ValueError("watchdog time must be finite")
        self.armed = True
        self.last_command_time = float(now)

    def close(self) -> None:
        """Disarm after the external command gate has closed."""
        self.armed = False
        self.last_command_time = None

    def command(self, now: float) -> None:
        """Refresh an armed session from one accepted complete command."""
        if not self.armed:
            return
        if not math.isfinite(now):
            raise ValueError("watchdog time must be finite")
        self.last_command_time = float(now)

    def expired(self, now: float) -> bool:
        """Return true for stale or future-dated armed session state."""
        if not self.armed or self.last_command_time is None:
            return False
        age = float(now) - self.last_command_time
        return not math.isfinite(age) or age < 0.0 or age > self.timeout_s
