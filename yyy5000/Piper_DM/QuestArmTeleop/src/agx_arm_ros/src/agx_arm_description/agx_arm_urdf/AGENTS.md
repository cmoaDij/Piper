# Repository Guidelines

## Scope

This nested robot-description repository contains Piper and Nero URDF/Xacro models, meshes, and README files. Keep each robot variant in its existing directory and avoid changing binary meshes unless the physical model changed.

## Conventions and Validation

Preserve joint names, parent/child links, units, and the `joint1`–`joint6` plus `gripper` ordering expected by drivers and learned-policy data. Update Xacro and generated URDF inputs together when applicable. Verify description changes with `ros2 launch agx_arm_description display.launch.py arm_type:=piper` before use on hardware.
