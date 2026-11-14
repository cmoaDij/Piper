from umi.real_world.piper_policy_timing import latency_compensated_action_index, next_action_index


def test():
    assert next_action_index(10.0, 10.07, 16) == 2
    assert next_action_index(10.0, 10.001, 16) == 1
    for obs, now, count in ((10, 9, 16), (10, 10.251, 16), (10, 10.15, 2)):
        try:
            next_action_index(obs, now, count)
        except ValueError:
            pass
        else:
            raise AssertionError('Invalid or stale prediction was accepted')
    assert latency_compensated_action_index(
        10.0, 10.05, 16, camera_latency_s=.210716, piper_latency_s=.340) == 13
    assert latency_compensated_action_index(
        10.0, 10.05, 16, camera_latency_s=.210716, piper_latency_s=.340,
        lead_s=.05) == 14
    assert latency_compensated_action_index(
        10., 10.05, 16, camera_latency_s=.146, robot_action_latency_s=.215,
        gripper_action_latency_s=.070) == 9
    assert latency_compensated_action_index(
        10., 10.05, 16, camera_latency_s=.146, robot_action_latency_s=.070,
        gripper_action_latency_s=.215) == 9


if __name__ == '__main__':
    test()
    print('Piper policy timing checks passed')
