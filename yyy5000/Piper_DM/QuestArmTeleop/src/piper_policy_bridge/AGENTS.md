# Repository Guidelines

This ROS 2 Python package is the only bridge between policy IPC and Piper command topics. Keep it compatible with ROS Humble Python 3.10 and independent of LeRobot/Python 3.12. The bridge publishes only `/policy/joint_states`; the mux alone publishes `/control/joint_states`.

Never weaken dead-man timeout, stale-feedback rejection, finite-value validation, hard joint limits, or per-control-tick motion limits. Tests must not access CAN or actuators.

The implemented `PIPER_LEROBOT_V3` interface requires distinct wrist/global ROS image
subscriptions with timestamps. Pair them with the same buffered mutually-nearest rule
used during collection and a default 25 ms skew limit. At every policy tick, require
the latest completed pair to be fresh; otherwise clear the target and publish nothing.
Never synthesize, duplicate, independently combine two latest callbacks, or indefinitely
reuse a missing view. Preserve all existing safety gates.
Snapshot the latest timestamped feedback state in each image callback. After a camera
pair completes, use the image timestamp midpoint and the same nearest-snapshot rule as
collection, with a default 50 ms state/image skew limit. Policy inference must receive
that frozen aligned state; 50 Hz command interpolation must continue to use the newest
live feedback state.

IPC responses carry the full action chunk and echo the observation midpoint timestamp.
Schedule every action on the absolute host-clock 10 Hz grid, drop chunk prefixes whose
slots expired before response arrival, and prefetch non-ensemble chunks one slot before
depletion. ACT temporal aggregation must combine only predictions that name the same
execution slot and retain the original exponential preference for older predictions.
Reset this scheduler together with every policy episode reset.

Actuated deployment must enforce this startup order: close the hardware command gate,
move the six arm joints home, verify near-zero position and velocity continuously,
explicitly reset the policy action queue, start the dead-man heartbeat, and wait for
operator `RUN`. The bridge startup hold must prevent inference and command publication
before `RUN`; after release, require one fresh policy command and recheck home feedback
before opening `/control_enable`. Any failure leaves the hardware gate closed. The
driver's Empty `/move_home` response is not proof of success, and arm home does not
imply a gripper position.
Piper v158 velocity feedback may remain quantized above the configured threshold at a
stationary zero. The live-policy state machine therefore uses a continuous position-only
home check rather than broadening the velocity limit. It must also command an AGX
gripper to `0.07 m` during `/move_home` and verify that opening from joint feedback
before `RUN`. Keep command and feedback units distinct: the current demonstrations
command `0.07 m` but observed fully-open feedback tops out near `0.06692 m`; deployment
therefore verifies a calibrated minimum feedback opening of `0.060 m` rather than
requiring feedback to equal the command target.

`piper_dual_realsense.launch.py` defaults to headless publishing for deployment. Its
explicit `preview:=true` mode may start two `rqt_image_view` processes for local camera
alignment; documentation must not imply that the default launch opens GUI windows.
Both deployment cameras publish native `320x240 RGB8 @ 30 Hz` so raw DDS traffic is
already at the policy input resolution. Keep the bridge's conditional area resize for
compatibility with other publishers, but do not restore `640x480` raw deployment topics.
