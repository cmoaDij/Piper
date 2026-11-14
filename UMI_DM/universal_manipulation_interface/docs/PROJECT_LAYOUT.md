# UMI workspace layout

The repository keeps code, acquisition sessions, training runs, and hardware
runs separate. Existing `my_session*` directories remain valid compatibility
paths; new work should use the shorter aliases below.

```text
sessions/YYYYMMDD_task_vNN/       raw videos, calibration, processed runs
train/YYYYMMDD_task_vNN_mNN/      checkpoints, logs, and training config
deploy/YYYYMMDD_task_vNN_mNN_dNN/ one hardware run and its diagnostics
```

For the current data:

- `sessions/20261107_pick_place_v01` points to `my_session2`.
- `train/20261107_pick_place_v01_m01` points to the completed 120-epoch run.
- `train/current` is the checkpoint selected by the deployment launcher.
- `deploy/20261107_pick_place_v01_m01_d01` contains the deployment manifest.

Do not edit files through the aliases while a process is writing the target.
The configured SLAM pipeline still expects `raw_videos/`, `mapping.MP4`,
`gripper_calibration/`, and the existing calibration filenames.

## Naming

Use `YYYYMMDD_task_vNN` for sessions, `_mNN` for training runs, and `_dNN`
for hardware runs. Keep raw videos immutable. Each processed run and training
run should have its own manifest and checkpoint directory.
