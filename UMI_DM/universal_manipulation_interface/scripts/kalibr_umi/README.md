# GoPro ChArUco to Kalibr

This adapter is specific to the recorded 10 x 8 ChArUco board with 50 mm squares. It creates a ROS1 bag containing every original-size frame as mono8 and every extracted IMU sample, then injects the existing OpenICC full-resolution ChArUco corners into Kalibr. It does not modify the MP4. Kalibr's checkerboard target is used **only as a geometry container**; its checkerboard detector is bypassed. The adapter checks all 63 target coordinates and timestamps before accepting observations. Frames with fewer than eight detected corners or an unsuccessful pose initialization are reported and excluded.

The generated IMU noise YAML is provisional, copied from the upstream UMI GoPro SLAM YAML; it is not an Allan variance measurement of this camera. Kalibr estimates the camera-to-IMU time offset. Compare the resulting residuals and transforms with OpenICC, and verify timestamps and physical geometry before use. Kalibr and OpenICC use different transform/time-shift conventions.

Dependencies in the active Python 3.9 `umi` environment: `av`, `numpy`, `PyYAML`, `rosbags==0.9.23`, and `py-ubjson==0.16.1`. The uncompressed ROS1 bag needs approximately 15 GB. It stores decoded 2704 x 2028 grayscale frames, not a transcoded video. Uncompressed chunks are deliberate: ROS Noetic `roslz4` rejected the LZ4 framing from `rosbags` in testing.

```
python scripts/kalibr_umi/prepare_dataset.py \
  --video /home/l1/Piper_All/UMI_DM/hero11_calibration_fullres/cam_imu/GX010545.MP4 \
  --telemetry /home/l1/Piper_All/UMI_DM/hero11_calibration_fullres/cam_imu/GX010545_gen.json \
  --corners /home/l1/Piper_All/UMI_DM/hero11_calibration_fullres/cam_imu/cam_imu_corners_GX010545.uson \
  --intrinsics /home/l1/Piper_All/UMI_DM/hero11_calibration_fullres/cam/cam_calib_GX010543_fi_1.0.json \
  --output /home/l1/Piper_All/UMI_DM/kalibr_comparison

scripts/kalibr_umi/run_kalibr.sh /home/l1/Piper_All/UMI_DM/kalibr_comparison
```

The second command requires the official upstream ROS1 Noetic Kalibr Docker image built locally as `umi-kalibr:local`. Its image expects the upstream source tree at `/catkin_ws/src/kalibr` and the built environment at `/catkin_ws/devel/setup.bash`. No `--bag-freq`, truncation, or timestamp correction is used.

The image used for this comparison was built from the official Kalibr repository
at commit `1f60227442d25e36365ef5f72cd80b9666d73467`, using
`docker build -t umi-kalibr:local -f Dockerfile_ros1_20_04 .`.

`img_timestamps_ns` from the OpenICC telemetry file are used directly for ROS
image timestamps; `timestamps_ns` are used for IMU messages. The adapter checks
these image timestamps against the full-resolution corner records and checks the
board coordinate permutation before fitting. OpenICC's gyro and accelerometer
arrays are used without changing their units or axis order.

`diagnose_video.py` compares the stored corner detections to independent OpenCV
ChArUco detections on every original video frame. It writes visual overlays,
per-frame PnP errors, and a rotation-speed correlation diagnostic. The latter
uses only adjacent pairs with independent PnP RMSE below 2 px; it is a screening
tool rather than a camera–IMU time-offset calibration. Neither the diagnostic
gating nor the independently detected corners change a calibration input.

`check_corner_outliers.py` screens the independently detected image points
against a per-frame RANSAC pose using the full-resolution fisheye model. It
writes an auditable CSV and a separate observation JSON, keeping every input
frame and timestamp. No video, IMU data, or original observation file is
modified. A screened point is a candidate outlier, not proof of detector
failure: large residuals can also arise from model or recording errors. Compare
different thresholds and/or robust loss settings before interpreting results.

```
python scripts/kalibr_umi/check_corner_outliers.py \
  --observations /path/to/charuco_observations.json \
  --intrinsics /path/to/cam_calib.json \
  --output /path/to/new_diagnostics --threshold-px 3
```

For a Kalibr sensitivity run, copy the generated `filtered_observations.json`
to a new experiment directory as `charuco_observations.json`, alongside the
unchanged bag and YAML inputs, then run `run_kalibr.sh` on that directory. The
3 px and 20 px runs in this session are diagnostic comparisons; neither
threshold establishes ground-truth calibration accuracy. Test the screening
and adapter with `python tests/test_kalibr_corner_outliers.py` and
`python tests/test_kalibr_charuco_adapter.py`.
