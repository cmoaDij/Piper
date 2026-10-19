# Repository Guidelines

## Scope

This nested ROS 2 workspace contains Piper/Nero drivers, messages, descriptions, and MoveIt packages. Keep controller changes in `src/agx_arm_ctrl/`, messages in `src/agx_arm_msgs/`, and robot assets in `src/agx_arm_description/`. Test fixtures and command examples are under `test/`.

## Control Contract

Piper absolute joint targets are named `joint1`–`joint6` plus `gripper`. The learning policy publishes to `/policy/joint_states`; a mux must validate and select it before forwarding one complete message to the driver's `/control/joint_states`. Do not weaken CAN, limit, or error checks for policy integration.

`/move_home` must not trust a previously latched `motion_status`. Stream the fixed
joint-zero target at the configured low home speed and declare completion only after
measured joint positions remain within tolerance for the configured duration. For an
AGX gripper, command and later verify the `0.07 m` open position before policy startup.

## Checks

Build from this workspace root with `colcon build --symlink-install`, then run `colcon test --packages-select agx_arm_ctrl`. Test hardware changes with RViz or disabled actuators first and document CAN interface, arm type, and emergency-stop verification.

The `/move_home` service may apply `home_speed_percent` only for its fixed all-zero
joint move and must restore the configured normal `speed_percent` afterward. A value
of zero disables the temporary override. Standard USB teleoperation configures normal
speed 50% and home speed 10%; keep both launch-configurable.
