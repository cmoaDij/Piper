# Repository Guidelines

## Project Structure & Module Organization

This ROS 2 workspace teleoperates Agilex arms with Meta Quest headsets. Keep headset input, pose processing, inverse kinematics, launch files, and configuration in `src/oculus_reader/`. Robot drivers, messages, MoveIt configuration, descriptions, and CAN utilities live under `src/agx_arm_ros/`. Hardware test commands are in `src/agx_arm_ros/test/`; documentation images belong in `img/`. Do not commit generated `build/`, `install/`, or `log/` directories.

## Build, Test, and Development Commands

Run commands from the repository root unless noted:

- `colcon build --symlink-install` builds every ROS 2 package and keeps Python edits visible without a full reinstall.
- `source install/setup.bash` overlays built packages; repeat in each new terminal.
- `colcon test` runs registered ament tests, including the controller package's lint checks.
- `colcon test-result --verbose` displays test failures and diagnostics.
- `ros2 launch oculus_reader teleop_single_piper_usb.launch.py` starts the Piper V1.5.8 teleoperation stack. Other single- and dual-arm launch files are under `src/oculus_reader/launch/`.

Install dependencies documented in `README.md` before building.

## Coding Style & Naming Conventions

Use four spaces for Python indentation and follow PEP 8. Name modules, functions, ROS nodes, topics, and parameters with `snake_case`; use `PascalCase` for classes and uppercase only for constants. Name launch files `<purpose>.launch.py` and robot-specific YAML descriptively, such as `arm_ik_pose_node.nero.yaml`. Declare new dependencies in the relevant `package.xml` and build file. `agx_arm_ctrl` enforces `ament_flake8`, `ament_pep257`, and copyright checks.

## Testing Guidelines

Add Python tests as `test_*.py` in the package's `test/` directory. Run `colcon test --packages-select <package>` while iterating, then test the complete workspace. No coverage threshold is currently configured. For launch or hardware changes, document the tested robot model, CAN interface, single/dual-arm mode, and observed behavior; never rely on hardware validation as a substitute for lint and build checks.

## Commit & Pull Request Guidelines

History favors short, imperative summaries such as `update README`, `add teleop single&double piper`, and `fix ...`. Keep each commit focused and state the affected behavior. Pull requests should explain the change, list build/test commands and hardware validation, link relevant issues, and call out configuration or dependency changes. Include screenshots or RViz captures for visualization, frame-alignment, or model changes.

## Hardware Safety

Before running teleoperation, verify CAN port mappings, arm type, workspace clearance, controller frames, and access to the emergency stop. Test new motion logic in RViz or with actuators disabled before commanding a physical robot.

## Wrist Camera Video

Use `teleop_single_piper_usb_realsense_record.launch.py` to add D435 colour-video recording without changing the existing USB teleoperation launch. The recorder observes only `/camera/wrist_camera/color/image_raw` and saves H.264 MP4 files outside Git. It never publishes an arm command.

In the combined launch, Quest `A` begins both teleoperation and video capture; `B` stops teleoperation and finalizes the MP4. `pub_delta_pose.py` publishes this passive state as `/teleop/enabled`, so future data loggers should subscribe to that topic rather than send arm commands.
