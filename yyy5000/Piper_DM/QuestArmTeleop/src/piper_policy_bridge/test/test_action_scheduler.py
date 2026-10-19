import math

import numpy as np

from piper_policy_bridge.action_scheduler import AbsoluteActionScheduler


def chunk(values):
    return np.repeat(np.asarray(values, dtype=np.float32)[:, None], 7, axis=1)


def test_late_chunk_drops_only_expired_slots():
    scheduler = AbsoluteActionScheduler(0.1)
    result = scheduler.add_chunk(
        chunk([0.0, 1.0, 2.0, 3.0]),
        observation_timestamp=10.01,
        received_timestamp=10.15,
        policy_type="diffusion",
        execution_steps=4,
        temporal_ensemble_coeff=None,
        period_s=0.1,
    )

    assert result.expired_actions == 1
    assert result.accepted_actions == 3
    np.testing.assert_array_equal(scheduler.action_for_time(10.15), np.ones(7))


def test_subperiod_response_keeps_action_zero_across_wall_clock_boundary():
    scheduler = AbsoluteActionScheduler(0.1)
    result = scheduler.add_chunk(
        chunk([4.0, 5.0]),
        observation_timestamp=10.099,
        received_timestamp=10.105,
        policy_type="diffusion",
        execution_steps=2,
        temporal_ensemble_coeff=None,
        period_s=0.1,
    )

    assert result.expired_actions == 0
    np.testing.assert_array_equal(
        scheduler.action_for_time(10.105), np.full(7, 4.0)
    )


def test_non_ensemble_prefetches_before_queue_depletion():
    scheduler = AbsoluteActionScheduler(0.1)
    scheduler.note_request(10.01)
    scheduler.add_chunk(
        chunk([0.0, 1.0, 2.0, 3.0]),
        observation_timestamp=10.01,
        received_timestamp=10.02,
        policy_type="act",
        execution_steps=4,
        temporal_ensemble_coeff=None,
        period_s=0.1,
    )

    assert not scheduler.should_request(10.11, 10.11)
    assert scheduler.should_request(10.21, 10.21)


def test_non_ensemble_prefetch_keeps_committed_overlap_and_skips_new_prefix():
    scheduler = AbsoluteActionScheduler(0.1)
    scheduler.add_chunk(
        chunk([0.0, 1.0, 2.0, 3.0]),
        observation_timestamp=10.01,
        received_timestamp=10.02,
        policy_type="diffusion",
        execution_steps=4,
        temporal_ensemble_coeff=None,
        period_s=0.1,
    )
    result = scheduler.add_chunk(
        chunk([100.0, 101.0, 102.0, 103.0]),
        observation_timestamp=10.21,
        received_timestamp=10.22,
        policy_type="diffusion",
        execution_steps=4,
        temporal_ensemble_coeff=None,
        period_s=0.1,
    )

    assert result.committed_overlap_actions == 2
    np.testing.assert_array_equal(
        scheduler.action_for_time(10.25), np.full(7, 2.0)
    )
    np.testing.assert_array_equal(
        scheduler.action_for_time(10.35), np.full(7, 3.0)
    )
    np.testing.assert_array_equal(
        scheduler.action_for_time(10.45), np.full(7, 102.0)
    )


def test_act_temporal_ensemble_matches_absolute_execution_slot():
    scheduler = AbsoluteActionScheduler(0.1)
    scheduler.add_chunk(
        chunk([0.0, 10.0, 20.0]),
        observation_timestamp=10.01,
        received_timestamp=10.02,
        policy_type="act",
        execution_steps=1,
        temporal_ensemble_coeff=0.01,
        period_s=0.1,
    )
    scheduler.add_chunk(
        chunk([100.0, 110.0, 120.0]),
        observation_timestamp=10.11,
        received_timestamp=10.12,
        policy_type="act",
        execution_steps=1,
        temporal_ensemble_coeff=0.01,
        period_s=0.1,
    )

    old_weight = 1.0
    new_weight = math.exp(-0.01)
    expected = (
        10.0 * old_weight + 100.0 * new_weight
    ) / (old_weight + new_weight)
    np.testing.assert_allclose(
        scheduler.action_for_time(10.15), np.full(7, expected), rtol=1e-6
    )


def test_missed_query_uses_absolute_slot_distance_for_weight():
    scheduler = AbsoluteActionScheduler(0.1)
    scheduler.add_chunk(
        chunk([0.0, 1.0, 2.0, 3.0]),
        observation_timestamp=10.01,
        received_timestamp=10.02,
        policy_type="act",
        execution_steps=1,
        temporal_ensemble_coeff=0.1,
        period_s=0.1,
    )
    scheduler.add_chunk(
        chunk([30.0, 31.0]),
        observation_timestamp=10.21,
        received_timestamp=10.22,
        policy_type="act",
        execution_steps=1,
        temporal_ensemble_coeff=0.1,
        period_s=0.1,
    )

    expected = (2.0 + math.exp(-0.2) * 30.0) / (1.0 + math.exp(-0.2))
    np.testing.assert_allclose(
        scheduler.action_for_time(10.25), np.full(7, expected), rtol=1e-6
    )
