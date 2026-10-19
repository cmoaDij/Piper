import numpy as np
import pytest

from data_collect.dataset.alignment import action_indices, causal_indices
from data_collect.calibration.latency import image_indices


def test_observations_use_only_past_valid_frames():
    idx, age = causal_indices([10, 20, 30], np.array([15, 25, 35]), [True, False, True])
    np.testing.assert_array_equal(idx, [0, 0, 2])
    np.testing.assert_array_equal(age, [5, 15, 5])
    with pytest.raises(ValueError, match="起点"):
        causal_indices([20], np.array([10]))


def test_action_ties_prefer_past_then_hold_last_command():
    indices, held = action_indices([100, 120], np.array([110, 170]), half_window_ns=15)
    np.testing.assert_array_equal(indices, [0, 1])
    np.testing.assert_array_equal(held, [False, True])


def test_negative_offset_never_makes_a_frame_available_early():
    raw = np.array([100, 200], dtype=np.int64)
    idx, age, corrected = image_indices(raw, np.array([150, 250]), None,
                                        "cameras/global_rgb", {"relative_offsets_ns": {"global_rgb": -50}})
    np.testing.assert_array_equal(idx, [0, 1])
    assert (age >= 0).all()
    np.testing.assert_array_equal(corrected, [50, 150])
