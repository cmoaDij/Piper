"""Buffered mutually-nearest synchronization for policy camera observations."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Any

import numpy as np


CAMERA_ROLES = ("wrist", "global")


@dataclass(frozen=True)
class TimestampedImage:
    """One decoded image with the robot snapshot present at its callback."""

    role: str
    image: Any
    timestamp: float
    arrival: float
    state: Any | None = None
    state_timestamp: float | None = None


@dataclass(frozen=True)
class PairState:
    """Robot snapshot selected against a paired camera timestamp midpoint."""

    state: Any
    timestamp: float
    image_skew: float


def select_pair_state(
    wrist: TimestampedImage,
    global_: TimestampedImage,
    max_skew_s: float,
) -> PairState | None:
    """Match collection's nearest camera-snapshot state selection rule."""
    if not np.isfinite(max_skew_s) or max_skew_s <= 0.0:
        raise ValueError("max state/image skew must be finite and positive")
    image_timestamp = (wrist.timestamp + global_.timestamp) * 0.5
    candidates = [
        frame
        for frame in (wrist, global_)
        if frame.state is not None
        and frame.state_timestamp is not None
        and np.isfinite(frame.state_timestamp)
    ]
    if not candidates:
        return None
    reference = min(
        candidates,
        key=lambda frame: abs(float(frame.state_timestamp) - image_timestamp),
    )
    state_timestamp = float(reference.state_timestamp)
    image_skew = state_timestamp - image_timestamp
    if abs(image_skew) > max_skew_s:
        return None
    return PairState(reference.state, state_timestamp, image_skew)


class NearestCameraSynchronizer:
    """Pair mutually nearest buffered frames after a bounded look-ahead."""

    def __init__(self, max_skew_s: float) -> None:
        if not np.isfinite(max_skew_s) or max_skew_s <= 0.0:
            raise ValueError("max camera skew must be finite and positive")
        self.max_skew_s = float(max_skew_s)
        self.buffers = {role: deque() for role in CAMERA_ROLES}
        self.unmatched = {role: 0 for role in CAMERA_ROLES}

    @staticmethod
    def _nearest_index(buffer: deque[TimestampedImage], timestamp: float) -> int:
        return min(
            range(len(buffer)),
            key=lambda index: abs(buffer[index].timestamp - timestamp),
        )

    def push(
        self, frame: TimestampedImage
    ) -> list[tuple[TimestampedImage, TimestampedImage]]:
        """Buffer one frame and return mature mutually-nearest pairs."""
        if frame.role not in self.buffers:
            raise ValueError(f"unknown camera role: {frame.role}")
        buffer = self.buffers[frame.role]
        if buffer and frame.timestamp <= buffer[-1].timestamp:
            raise ValueError(f"{frame.role} timestamps must be strictly increasing")
        buffer.append(frame)
        return self._drain()

    def _drain(self) -> list[tuple[TimestampedImage, TimestampedImage]]:
        pairs = []
        wrist = self.buffers["wrist"]
        global_ = self.buffers["global"]
        while wrist and global_:
            earliest_role = "wrist" if wrist[0].timestamp <= global_[0].timestamp else "global"
            primary = self.buffers[earliest_role]
            other_role = "global" if earliest_role == "wrist" else "wrist"
            other = self.buffers[other_role]
            candidate = primary[0]
            other_index = self._nearest_index(other, candidate.timestamp)
            counterpart = other[other_index]
            mutual = self._nearest_index(primary, counterpart.timestamp) == 0
            if primary[-1].timestamp < counterpart.timestamp:
                break
            if mutual and abs(candidate.timestamp - counterpart.timestamp) <= self.max_skew_s:
                primary.popleft()
                del other[other_index]
                pairs.append(
                    (candidate, counterpart)
                    if earliest_role == "wrist"
                    else (counterpart, candidate)
                )
            else:
                primary.popleft()
                self.unmatched[earliest_role] += 1
        pairs.sort(key=lambda pair: (pair[0].timestamp + pair[1].timestamp) * 0.5)
        return pairs
