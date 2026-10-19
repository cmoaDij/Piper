"""Timestamp-indexed 10 Hz action scheduling and ACT temporal aggregation."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from .contract import ACTION_DIM


@dataclass(frozen=True)
class ChunkScheduleResult:
    """Summary of one chunk insertion."""

    accepted_actions: int
    expired_actions: int
    committed_overlap_actions: int
    first_slot: int | None
    last_slot: int | None


class AbsoluteActionScheduler:
    """Bind predictions to clock slots and merge overlapping ACT chunks."""

    def __init__(self, period_s: float = 0.1) -> None:
        if not math.isfinite(period_s) or period_s <= 0.0:
            raise ValueError("period_s must be positive and finite")
        self.period_s = float(period_s)
        self.clear()

    def clear(self) -> None:
        self.policy_type: str | None = None
        self.execution_steps: int | None = None
        self.temporal_ensemble_coeff: float | None = None
        self.predictions: dict[int, list[tuple[int, np.ndarray]]] = {}
        self.last_requested_observation_slot: int | None = None
        self.last_scheduled_slot: int | None = None
        self.epoch_timestamp: float | None = None

    def slot(self, timestamp: float) -> int:
        """Return the execution slot containing a host-clock timestamp."""
        if not math.isfinite(timestamp):
            raise ValueError("timestamp must be finite")
        if self.epoch_timestamp is None:
            return 0
        elapsed = (timestamp - self.epoch_timestamp) / self.period_s
        return math.floor(elapsed + 1e-9)

    def _observation_slot(self, timestamp: float) -> int:
        if not math.isfinite(timestamp):
            raise ValueError("timestamp must be finite")
        if self.epoch_timestamp is None:
            self.epoch_timestamp = float(timestamp)
        return round((timestamp - self.epoch_timestamp) / self.period_s)

    @property
    def temporal_ensemble_enabled(self) -> bool:
        return (
            self.policy_type == "act"
            and self.temporal_ensemble_coeff is not None
        )

    def should_request(self, observation_timestamp: float, now: float) -> bool:
        observation_slot = self._observation_slot(observation_timestamp)
        if (
            self.last_requested_observation_slot is not None
            and observation_slot <= self.last_requested_observation_slot
        ):
            return False
        if self.policy_type is None or self.temporal_ensemble_enabled:
            return True
        current_slot = self.slot(now)
        # Start the next inference one slot before depletion so the old target
        # remains executable while the new chunk is being computed.
        return (
            self.last_scheduled_slot is None
            or current_slot >= self.last_scheduled_slot - 1
        )

    def note_request(self, observation_timestamp: float) -> None:
        self.last_requested_observation_slot = self._observation_slot(
            observation_timestamp
        )

    def _configure(
        self,
        policy_type: str,
        execution_steps: int,
        temporal_ensemble_coeff: float | None,
        period_s: float,
    ) -> None:
        if policy_type not in {"diffusion", "act"}:
            raise ValueError("unsupported policy type")
        if execution_steps < 1:
            raise ValueError("execution_steps must be positive")
        if temporal_ensemble_coeff is not None and (
            not math.isfinite(temporal_ensemble_coeff)
            or temporal_ensemble_coeff < 0.0
        ):
            raise ValueError("invalid temporal ensemble coefficient")
        if not math.isclose(
            period_s, self.period_s, rel_tol=0.0, abs_tol=1e-9
        ):
            raise ValueError("policy and bridge action periods disagree")
        configuration = (policy_type, execution_steps, temporal_ensemble_coeff)
        existing = (
            self.policy_type,
            self.execution_steps,
            self.temporal_ensemble_coeff,
        )
        if self.policy_type is not None and configuration != existing:
            raise ValueError(
                "policy scheduling metadata changed without a reset"
            )
        (
            self.policy_type,
            self.execution_steps,
            self.temporal_ensemble_coeff,
        ) = configuration

    def add_chunk(
        self,
        actions,
        *,
        observation_timestamp: float,
        received_timestamp: float,
        policy_type: str,
        execution_steps: int,
        temporal_ensemble_coeff: float | None,
        period_s: float,
    ) -> ChunkScheduleResult:
        actions = np.asarray(actions, dtype=np.float32)
        if (
            actions.ndim != 2
            or actions.shape[0] < 1
            or actions.shape[1] != ACTION_DIM
            or not np.isfinite(actions).all()
        ):
            raise ValueError(f"actions must be finite [steps,{ACTION_DIM}]")
        if execution_steps > actions.shape[0]:
            raise ValueError("execution_steps exceeds action chunk length")
        self._configure(
            policy_type,
            execution_steps,
            temporal_ensemble_coeff,
            period_s,
        )
        observation_slot = self._observation_slot(observation_timestamp)
        current_slot = self.slot(received_timestamp)
        response_age = max(0.0, received_timestamp - observation_timestamp)
        first_index = max(
            0,
            math.floor(response_age / self.period_s + 1e-9),
            current_slot - observation_slot,
        )
        final_index = (
            actions.shape[0]
            if self.temporal_ensemble_enabled
            else min(actions.shape[0], execution_steps)
        )
        expired = min(first_index, final_index)
        committed_overlap = 0
        accepted_slots: list[int] = []
        for index in range(first_index, final_index):
            target_slot = observation_slot + index
            if target_slot < current_slot:
                expired += 1
                continue
            prediction = (observation_slot, actions[index].copy())
            if self.temporal_ensemble_enabled:
                self.predictions.setdefault(target_slot, []).append(prediction)
            else:
                if target_slot in self.predictions:
                    # Already-issued targets stay committed. The new chunk is
                    # aligned by absolute time and starts after the overlap,
                    # rather than restarting from its zero prefix.
                    committed_overlap += 1
                    continue
                self.predictions[target_slot] = [prediction]
            accepted_slots.append(target_slot)
        self._prune(current_slot)
        if accepted_slots:
            if self.last_scheduled_slot is None:
                self.last_scheduled_slot = accepted_slots[-1]
            else:
                self.last_scheduled_slot = max(
                    self.last_scheduled_slot, accepted_slots[-1]
                )
        elif not self.temporal_ensemble_enabled:
            self.last_scheduled_slot = current_slot - 1
        return ChunkScheduleResult(
            len(accepted_slots),
            expired,
            committed_overlap,
            accepted_slots[0] if accepted_slots else None,
            accepted_slots[-1] if accepted_slots else None,
        )

    def _prune(self, current_slot: int) -> None:
        for slot in [slot for slot in self.predictions if slot < current_slot]:
            del self.predictions[slot]

    def action_for_time(self, timestamp: float) -> np.ndarray | None:
        current_slot = self.slot(timestamp)
        self._prune(current_slot)
        entries = self.predictions.get(current_slot)
        if not entries:
            return None
        if not self.temporal_ensemble_enabled or len(entries) == 1:
            return entries[-1][1].copy()
        entries = sorted(entries, key=lambda item: item[0])
        oldest_query_slot = entries[0][0]
        coefficient = float(self.temporal_ensemble_coeff)
        weights = np.asarray(
            [
                math.exp(-coefficient * (query_slot - oldest_query_slot))
                for query_slot, _ in entries
            ],
            dtype=np.float64,
        )
        values = np.stack([action for _, action in entries]).astype(np.float64)
        return np.average(values, axis=0, weights=weights).astype(np.float32)
