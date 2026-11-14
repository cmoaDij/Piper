"""Monotonic action timing for Piper policy integration; no hardware access."""
import math


def next_action_index(observation_time, now, count, *, action_period=0.05005,
                      min_lead=0.01, max_observation_age=0.25):
    """Select the first future target, or reject an expired observation.

    The default period is three frames of a 60000/1001 fps demonstration; the
    10 ms lead follows the upstream UMI evaluator, and 250 ms is the current
    Piper input timeout. Time values must use the same monotonic clock. Host
    image-dequeue time still does not account for HDMI and exposure latency.
    """
    values = (observation_time, now, action_period, min_lead, max_observation_age)
    if not all(math.isfinite(x) for x in values):
        raise ValueError('Action timing values must be finite')
    if not isinstance(count, int) or count <= 0 or action_period <= 0 or min_lead < 0 or max_observation_age <= 0:
        raise ValueError('Invalid action timing configuration')
    age = now - observation_time
    if age < 0 or age > max_observation_age:
        raise ValueError('Observation timestamp is in the future or stale')
    for index in range(count):
        if observation_time + index * action_period > now + min_lead:
            return index
    raise ValueError('All predicted actions have expired')


def latency_compensated_action_index(observation_receive_time, now, count, *,
                                     camera_latency_s=0.0,
                                     piper_latency_s=None,
                                     robot_action_latency_s=None,
                                     gripper_action_latency_s=None,
                                     lead_s=0.0,
                                     action_period=0.05005):
    """Select a future point using measured camera and Piper delays.

    The policy horizon is referenced to the scene time represented by the
    image, which is the host receive time minus camera latency.  The selected
    point must be at or after the current time plus effective Piper latency and
    any explicitly configured lead.
    """
    if robot_action_latency_s is None:
        robot_action_latency_s = 0.0 if piper_latency_s is None else piper_latency_s
    if gripper_action_latency_s is None:
        gripper_action_latency_s = robot_action_latency_s
    values = (observation_receive_time, now, camera_latency_s,
              robot_action_latency_s, gripper_action_latency_s, lead_s, action_period)
    if not all(math.isfinite(x) for x in values):
        raise ValueError('Latency compensation values must be finite')
    if (not isinstance(count, int) or count <= 0 or action_period <= 0
            or camera_latency_s < 0 or robot_action_latency_s < 0 or gripper_action_latency_s < 0 or lead_s < 0):
        raise ValueError('Invalid latency compensation configuration')
    reference_time = observation_receive_time - camera_latency_s
    required_time = now + max(robot_action_latency_s, gripper_action_latency_s) + lead_s
    index = int(math.ceil((required_time - reference_time) / action_period))
    if index < 0:
        index = 0
    if index >= count:
        raise ValueError('All latency-compensated actions have expired')
    return index
