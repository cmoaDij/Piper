from pathlib import Path
import sys

import pytest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from ik_result_contract import (  # noqa: E402
    COLLISION_FIELD,
    decode_correlated_result,
    encode_correlated_result,
)


def test_atomic_ik_result_round_trip():
    names, positions = encode_correlated_result(
        [f"joint{index}" for index in range(1, 7)],
        [0.1 * index for index in range(6)],
        [-0.1 * index for index in range(6)],
        True,
    )
    assert names[-1] == COLLISION_FIELD
    decoded_names, decoded_positions, path_start, collision = (
        decode_correlated_result(names, positions)
    )
    assert decoded_names == tuple(f"joint{index}" for index in range(1, 7))
    assert decoded_positions == tuple(0.1 * index for index in range(6))
    assert path_start == tuple(-0.1 * index for index in range(6))
    assert collision is True


@pytest.mark.parametrize(
    "names,positions",
    [
        (("joint1", COLLISION_FIELD), (0.0, 0.5)),
        (
            ("joint1", "__path_start_joint2__", COLLISION_FIELD),
            (0.0, 0.0, 0.0),
        ),
        (
            ("joint1", "__path_start_joint1__", COLLISION_FIELD),
            (float("nan"), 0.0, 0.0),
        ),
    ],
)
def test_atomic_ik_result_rejects_ambiguous_or_nonfinite_data(names, positions):
    with pytest.raises(ValueError, match="malformed"):
        decode_correlated_result(names, positions)
