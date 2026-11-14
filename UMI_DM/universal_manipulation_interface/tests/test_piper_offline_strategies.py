"""Check time-aligned replay behavior using synthetic model predictions."""
import numpy as np
from scripts.analyze_piper_offline_strategies import compare


def test():
    rows = []
    for i in range(8):
        t = 10 + i * .05005
        poses = [[t + k * .05005, 0, 0, 0, 0, 0] for k in range(16)]
        rows.append({'camera_times_monotonic_s': [t - .05, t],
                     'first_future_action_index': 2,
                     'predicted_flange_xyz_rpy': poses,
                     'predicted_physical_width_m': [.02 if i % 2 else .06] * 16})
    t, strategies, metrics, used = compare(rows)
    future = strategies['first_future'][0]
    ensemble = strategies['future_ensemble'][0]
    # Every synthetic chunk predicts the same world position at a given time.
    np.testing.assert_allclose(future, ensemble, atol=1e-10)
    assert used[-1] == 3
    assert metrics['ema_action_1']['gripper_step_mm']['max'] < metrics['direct_action_1']['gripper_step_mm']['max']
    assert t[-1] > t[0]
    print('Offline strategy timing and smoothing checks passed')


if __name__ == '__main__':
    test()
