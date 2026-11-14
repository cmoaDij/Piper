# Latency calibration

## Camera receive latency

Use the same GoPro/HDMI/capture-card path as deployment. Point the camera at
the QR window on this computer and run:

```bash
cd ~/Piper_All/UMI_DM/universal_manipulation_interface
conda activate umi
python scripts/calibrate_camera_receive_latency.py \
  --camera /dev/video0 \
  --samples 60 \
  --output ../deploy/current/latency/camera_latency.json
```

The encoded QR timestamp and frame receive timestamp use the same monotonic
clock. Use the median as the nominal time-axis correction. Keep P95 and
maximum as jitter and stale-frame diagnostics. The value includes display
refresh, GoPro, HDMI, capture, and dequeue latency. Do not substitute a phone
display or an SD-card-only recording.

## Piper feedback observation latency

This read-only test listens directly to SocketCAN. It does not require SDK
firmware identification, enable the arm, or send commands:

```bash
cd ~/Piper_All/Piper/Data_Collect
source ./env.sh capture
python ~/Piper_All/UMI_DM/universal_manipulation_interface/scripts/calibrate_piper_feedback_latency.py \
  --can can0 \
  --seconds 30 \
  --output ~/Piper_All/UMI_DM/deploy/current/latency/piper_feedback
```

Use the joint-group age, packet periods, and joint-group skew as the feedback
freshness diagnostic. These are host receive measurements, so they do not
claim to expose motor-sensor delay.

## Piper effective tracking latency

This test moves the robot, so inspect the start pose and physical workspace
before running it. The script requires the explicit `--execute` flag and uses a
small one-joint-at-a-time alternating step motion around `[0, 15, -15, 30, 0, 0]`
degrees:

```bash
cd ~/Piper_All/Piper/Data_Collect
source ./env.sh capture
python ~/Piper_All/UMI_DM/universal_manipulation_interface/scripts/calibrate_piper_tracking_latency.py \
  --execute --joints 1 2 3 4 5 6 \
  --amplitude-deg 3 --cycles 5 --dwell-s 0.5 --speed-percent 20 \
  --can can0 \
  --output ~/Piper_All/UMI_DM/deploy/current/latency/piper_action
```

The output contains `targets.jsonl`, raw `feedback.csv`, and
`piper_latency.json`. The estimate is the positive-lag cross-correlation of
the SDK target and host-received CAN feedback for each excited joint. By
default all six joints are measured sequentially. Use `--joints 1 4` for a
shorter run. Treat `robot_action_latency_ms` as an effective command-to-
feedback action delay. It must not be used as feedback observation latency.

## Piper gripper response (run this before tuning grasp/release timing)

The new Piper-specific script uses the same SDK `move_gripper_m` path as
deployment. The older `calibrate_gripper_latency.py` is for WSG, not Piper.
Remove objects from the gripper, stop the policy/other motion programs, and
keep the arm physically stationary. The script first calls the SDK's
`set_speed_percent` using the deployment speed (20%). This sends a CAN-control
mode message, with the SDK's partial-update field `move_mode=0xff`, and waits
for fresh controller status confirming CAN control. S-V1.9-0 can report arm
motion mode `UNKNOWN=255` after this message. Gripper-only calibration records
that value and does not require an arm trajectory mode; it does not send an
extra J-mode change. Actual arm faults, CAN control loss, joint displacement
and arm-enable changes still stop the test. This initialization
is also present in the SDK's gripper examples; width commands alone may be
ignored while the controller is in standby. No arm motors are enabled, no
joint/pose targets or payload settings are sent, and zero points are retained.
The script monitors joint displacement, CAN control mode and changes in the
initial arm enable state. The controller speed setting is set to the deployment
value as part of CAN initialization.
Existing enabled holding can be retained; a disabled arm must be securely
supported. It does not home, change joint limits, or set encoder zeros.
Keep the physical stop available before running this motion test.

```bash
cd ~/Piper_All/Piper/Data_Collect
source ./env.sh capture

python ~/Piper_All/UMI_DM/universal_manipulation_interface/scripts/calibrate_piper_gripper_latency.py \
  --execute --can can0 \
  --closed-mm 20 --open-mm 60 \
  --cycles 6 --dwell-s 1.5 \
  --output ~/Piper_All/UMI_DM/deploy/current/latency/gripper_action
```

The initial open-to-60-mm settling phase is excluded. The test then alternates
20 and 60 mm with unequal dwell times to reduce periodic fit ambiguity. It
uses the existing 1 N SDK force parameter (not an independently verified
contact-force measurement). Targets are raw widths with zero added offset.
On exit it requests gripper disable and records whether fresh feedback
confirmed disable. Its watchdog also disables only the gripper. Interruptions
and errors preserve partial data, including monitored joint/enable states in
`arm_states.jsonl`. Position-change errors name the affected joint and its
displacement; enable-state changes are reported separately.

Output in that directory:

- `gripper_latency.json`: effective command-to-feedback lag, opening/closing
  `t10_ms`, `t50_ms`, `t90_ms`, feedback periods and SDK call durations.
- `targets.jsonl`: actual SDK gripper-call start/return times for target changes.
- `sdk_calls.jsonl`: initial settle, target changes and unchanged-target heartbeats.
- `feedback.csv`: raw, signed width feedback with per-packet host receive time.
- `can_commands.jsonl`: observed 0x151 mode and 0x159 gripper command frames,
  including raw payload bytes. SocketCAN loopback observation is not a
  controller acknowledgement; the fresh status/motion response is checked
  separately.
- `controller_status.jsonl`: raw controller status frames including the raw
  motion-mode byte, avoiding ambiguity from SDK enum conversion.

The calibration enables the SDK connection's local CAN loopback so the
separate passive listener can observe outgoing commands. The SDK default
disables loopback; a zero observed-command count in an older run does not
prove the SDK sent nothing. Policy deployment keeps its existing connection
defaults.

`control_setup` records pre/post controller modes and the fresh status time.
`initial_gripper_settle` records the initial, final and observed min/max widths.
A stationary 47.4 mm trace after requests for 60 mm is not a latency result.
The script reports the actual and requested widths on an initial-settle
failure, and a completed run requires feedback confirming gripper disable.

`t50_ms` means half of the displacement from the pre-command width to the
requested width; `t90_ms` means 90%. These are response times, not pure dead
time. The correlation fit is an initial effective tracking reference for that
width range and control path. A search-boundary peak or missing 90% crossings
needs inspection; the fit search step is not the measurement accuracy.
The sampler timestamps immediately after `recv()` in an independent filtered
CAN thread. Printing, file writes and fitting are outside the receive loop.
Host scheduling and feedback transport remain part of the measured endpoint.
The script does not change deployment parameters or independently calibrate
the gripper feedback path.

## Absolute robot observation delay (independent reference required)

This is not a prerequisite for an initial short deployment check, provided
the provisional zero robot-observation correction is clearly treated as
unknown. Accurate physical-time alignment requires a separately calibrated
reference. CAN update intervals, SDK receive timestamps, and commanded joint
targets alone cannot identify physical-sample-to-host-receive delay.

`calibrate_piper_observation_latency.py` provides a passive recorder and an
offline comparison. It does not enable or move the arm. Use an independent
encoder or optical measurement of the **same physical joint angle**, with
known sample timestamps mapped to the **same host monotonic clock**. A clock
offset is indistinguishable from feedback delay; this script never fits away
such an offset. The GoPro/QR median includes display and decode-sampling
uncertainty, so it is not a precise reference for resolving a few milliseconds
of CAN feedback delay.

Record feedback while the independent reference records the same nonuniform,
small joint motion (for example, the existing arm tracking calibration):

```bash
cd ~/Piper_All/Piper/Data_Collect
source ./env.sh capture

python ~/Piper_All/UMI_DM/universal_manipulation_interface/scripts/calibrate_piper_observation_latency.py record \
  --can can0 --seconds 50 \
  --output ~/Piper_All/UMI_DM/deploy/current/latency/robot_observation
```

This records `joint_feedback.csv` with one row per joint in each actual CAN
pair packet. It never assigns one timestamp to a mixture of six cached angles.
The external reference must be exported in this long CSV format; angles are
in radians with the same joint signs as Piper:

```csv
host_sample_monotonic_s,joint_index,position_rad
12345.000000,1,0.0100
12345.005000,1,0.0102
```

For **real, synchronized** physical sample timestamps, analyze with the command
below. `REFERENCE_CSV` and `REFERENCE_CLOCK_UNCERTAINTY_MS` must be set to the
actual reference file and its independently established synchronization
uncertainty in milliseconds. Do not invent that number to run the command.

```bash
python ~/Piper_All/UMI_DM/universal_manipulation_interface/scripts/calibrate_piper_observation_latency.py analyze \
  --reference-csv "${REFERENCE_CSV:?Set the independent reference CSV path}" \
  --reference-timestamp sample \
  --clock-uncertainty-ms "${REFERENCE_CLOCK_UNCERTAINTY_MS:?Set measured clock uncertainty in ms}" \
  --feedback-csv ~/Piper_All/UMI_DM/deploy/current/latency/robot_observation/joint_feedback.csv \
  --output ~/Piper_All/UMI_DM/deploy/current/latency/robot_observation
```

If the reference has receive timestamps instead of sample timestamps, use CSV
column `host_recv_monotonic_s`, `--reference-timestamp receive`, and also supply
`--reference-latency-ms` from its separate calibration. Its uncertainty must be
considered together with clock uncertainty and sample periods. With only CAN
and no independent time reference, do not run an alleged absolute-latency
calibration or populate the robot observation correction from packet age.

`robot_observation_latency.json` keeps per-joint fit curves, signed delays,
sample periods and declared clock uncertainty. Negative or search-boundary
results are excluded from the nominal summary; unexcited joints receive no
estimate. No deployment config is changed automatically.

The existing 215 ms arm result is command-to-**received**-feedback phase lag.
If a physical observation delay is later identified under the same conditions,
that part is already inside the 215 ms estimate. Separating a physical action
delay requires removing that observation component (or refitting against the
physical reference); do not count it again as actuator delay.
