"""Focused checks for read-only Piper policy conversion and force bounds."""
import numpy as np
from scipy.spatial.transform import Rotation
from umi.real_world.piper_policy_conversion import (
    GripperPolicyLimits, check_gripper_target, physical_width_from_feedback,
    relative_policy_action_to_flange,
)


def reject(fn):
    try:
        fn()
    except ValueError:
        return
    raise AssertionError('Invalid conversion was accepted')


def test():
    limits = GripperPolicyLimits()
    assert np.allclose(physical_width_from_feedback(.0606, limits)[0], .0606)
    assert physical_width_from_feedback(.0023, limits) == (.0023, False)
    assert physical_width_from_feedback(-.0023, limits) == (0.0, True)
    target, force = check_gripper_target(.050, 1.0, limits)
    assert np.allclose([target, force], [.050, 1.0])
    reject(lambda: check_gripper_target(.071, 1.0, limits))
    reject(lambda: check_gripper_target(.050, 1.01, limits))
    reject(lambda: check_gripper_target(float('nan'), 1.0, limits))
    reject(lambda: GripperPolicyLimits(max_force_parameter_n=1.1))
    base = np.eye(4); flange_tcp = np.eye(4)
    flange_tcp[:3, :3] = Rotation.from_euler('z', 90, degrees=True).as_matrix()
    flange_tcp[2, 3] = .195
    action = np.array([.01, 0, 0, 1, 0, 0, 0, 1, 0])
    reject(lambda: relative_policy_action_to_flange(base, flange_tcp, action))
    pose = relative_policy_action_to_flange(base, flange_tcp, action, tcp_verified=True)
    assert np.allclose(pose, [0, .01, 0, 0, 0, 0], atol=1e-8)
    reject(lambda: relative_policy_action_to_flange(base, flange_tcp, np.zeros(9), tcp_verified=True))
    bad = flange_tcp.copy(); bad[0, 0] = 2
    reject(lambda: relative_policy_action_to_flange(base, bad, action, tcp_verified=True))
    print('Piper policy conversions passed')


if __name__ == '__main__':
    test()
