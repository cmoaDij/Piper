# Deployment workflow

Use the canonical training alias and run a receive-only check before enabling
Piper:

```bash
cd ~/Piper_All/UMI_DM/universal_manipulation_interface
bash scripts/run_piper_policy.sh \
  --observe 30 \
  --checkpoint ../train/current/checkpoints/latest.ckpt \
  --output-root ../deploy/current/runs
```

The browser dashboard is available at `http://127.0.0.1:8768/`. It shows the
latest camera/model input, prediction timing, controller feedback, and TCP
prediction-versus-feedback history. Dashboard updates and diagnostic events
are best effort and use bounded/non-blocking paths; the control socket remains
separate.

Only after the observe run is clean should execution be attempted:

```bash
bash scripts/run_piper_policy.sh \
  --execute 30 \
  --checkpoint ../train/current/checkpoints/latest.ckpt \
  --output-root ../deploy/current/runs
```

Each run writes a manifest, `live_observation.jsonl`, `control_events.jsonl`,
`observation_skips.jsonl`, and model/controller logs under the selected output
root. The launcher accepts `--session`, `--setup`, `--mask`, and
`--dashboard-port` so a new session or model never has to be hard-coded into
the script.

The SDK controller now uses the analytical EAIK branch solver for Piper.  The
capture environment must contain the Python 3.11 EAIK extension and `urchin`;
install them once with:

```bash
cd ~/Piper_All/UMI_DM/universal_manipulation_interface
bash scripts/install_eaik_capture.sh
```

The solver enumerates all analytical branches, filters the controller joint
intersection, and selects the branch nearest to the latest feedback.  It does
not turn an unreachable target into a boundary command; the existing FK
residual and collision checks still reject it.

The active deployment setup uses the camera latency median `0.146283 s` as the
nominal camera time correction, keeps robot observation latency separate from
robot action latency, and uses a 50.05 ms action period. The current Piper
effective action-delay reference is `0.215 s`, measured at SDK speed 20% with
3-degree steps on all six joints (210--220 ms). It depends on the motion and
controller settings; it is not an absolute sensor or CAN transmission delay.

Camera frames are paired with CAN states at the corrected scene time:
`CAN receive time = camera receive time - camera delay + robot observation delay`.
The robot observation delay remains unmeasured and is provisionally zero;
packet age and the 215 ms tracking estimate must not be substituted for it.
The controller selects a feasible future waypoint and dispatches it at
`scene time + waypoint index * 0.05005 - robot action delay`, rather than
immediately after IK. This is one timed waypoint per accepted prediction,
using the SDK's existing `move_j` planner, not the upstream RTDE interpolator.
Logs include the corrected observation times, planned waypoint time,
scheduled dispatch time, SDK call timestamps, and dispatch lateness.
This arm calibration does not measure the gripper's own response delay.

After a run, summarize its timing and tracking records with:

```bash
python scripts/analyze_deployment_run.py \
  --input-dir ../deploy/current/runs/sdk_YYYYMMDD_HHMMSS_xxxxxx
```

Create a complete offline task-space plot, including all prediction horizons,
the selected expected TCP waypoints, SDK feedback TCP, joint trajectories, and
gripper traces:

```bash
python scripts/plot_deployment_trajectory.py \
  --input-dir ../deploy/current/runs/sdk_YYYYMMDD_HHMMSS_xxxxxx \
  --setup ../my_session/deployment_check/piper_policy_setup.yaml
```

The output is written to `trajectory_analysis/`, with the main 3-D figure in
`trajectory_3d.png` and the machine-readable reconstruction in
`trajectory_data.json`.  Actual feedback points are the SDK read timestamps;
the plot does not claim that a command had physically arrived at that exact
instant.
