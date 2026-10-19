# Repository Guidelines

## Scope

`oculus_reader` turns Quest input into Piper targets. Launch files are in `launch/`, parameters in `config/`, and executable ROS Python nodes in `scripts/`. Preserve the existing controller topic and parameter interfaces unless dependent launch files are updated together.

## Learning Integration

The learning stack observes `/feedback/joint_states`, `/feedback/tcp_pose`, and final command targets. Do not add another direct publisher to `/control/joint_states`: route teleoperation and policy targets through the planned command mux. Full target vectors use `joint1` through `joint6`, then `gripper`.

## Safety and Checks

Use four-space PEP 8 Python and run `colcon build --packages-select oculus_reader`. Test launch changes in RViz before hardware. Preserve manual control, emergency-stop access, and source-selection behavior when changing command flow.

## Quest Controls

Single-arm teleoperation uses `A` to start, `B` to stop, right joystick press `RJ` to recenter the active hand/TCP mapping, and the right index-trigger axis for proportional gripper control. Recenter must be rising-edge triggered, require active teleoperation and valid TCP feedback, and preserve the arm's current TCP pose; it must never command a home pose. The released trigger must command the configured fully open position (`gripper_max_range`, currently `0.07 m`); a fully pressed trigger must command `0.0 m`. Button parameters may name any non-empty field emitted by `buttons_parser.py`; do not add a hard-coded field-name whitelist. Normal teleoperation launches use arm auto-enable and an open control gate, so operators should not need manual `SetBool` payloads such as `"{data: false}"`. Keep the standard services available for diagnostics and explicit shutdown tooling.

On a rising edge of `B`, publish `/teleop/enabled=false` before scheduling the optional
`/move_home` request. Standard single-arm USB teleoperation enables auto-home with a
0.5 s delay. Only schedule it when teleoperation was active, cancel an unsent request
if `A` is pressed during the delay, and reject a new start while the service future is
in progress. Home means the driver's fixed all-zero joint target, not reference recenter.
On every accepted `A` rising edge, require TCP feedback and atomically rebase both the
current controller pose and current TCP pose before enabling teleoperation. Never reuse
the pre-home TCP reference for the first post-home target.

## D435 Video Recording

`teleop_single_piper_usb_realsense_record.launch.py` starts the unchanged USB teleoperation stack, a D435 colour stream, and `realsense_video_recorder.py`. The recorder uses system Python and OpenCV, subscribes to `/camera/wrist_camera/color/image_raw`, and writes H.264 MP4 while `/teleop/enabled` is true. `pub_delta_pose.py` sets this passive state when Quest `A` starts teleoperation and clears it when `B` stops. RGB `Image.header.stamp` selects the 30 FPS output slot; missing slots repeat the prior frame so playback preserves real duration. The recorder never publishes arm commands.
