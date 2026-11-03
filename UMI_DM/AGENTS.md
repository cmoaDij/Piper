# Repository Guidelines

## Project Structure & Module Organization

The Git repository lives in `universal_manipulation_interface/`. Run the commands below from that directory (`cd universal_manipulation_interface`).

- `umi/`: robotics interfaces, shared memory, pose utilities, SLAM helpers, and trajectory evaluation.
- `diffusion_policy/`: policy models, datasets, training workspaces, and Hydra configuration under `config/`.
- `scripts_slam_pipeline/`: numbered video-processing, calibration, and dataset-generation stages, orchestrated by `run_slam_pipeline.py`.
- `scripts/` and `scripts_real/`: calibration, recording, replay, and robot-control tools.
- `tests/`: pose and camera checks; `assets/`: documentation images; `example/`: sample configurations and calibration assets.

## Build, Test, and Development Commands

The documented platform is Ubuntu 22.04. Install system dependencies and Docker as described in `README.md`; no separate application build is configured.

- `mamba env create -f conda_environment.yaml`: create the pinned Python 3.9 environment.
- `conda activate umi`: activate runtime dependencies.
- `python run_slam_pipeline.py example_demo_session`: process a captured demonstration session.
- `python scripts_slam_pipeline/07_generate_replay_buffer.py -o example_demo_session/dataset.zarr.zip example_demo_session`: generate training data.
- `python train.py --config-name=train_diffusion_unet_timm_umi_workspace task.dataset_path=example_demo_session/dataset.zarr.zip`: launch single-GPU training.
- `python tests/test_pose_util.py`: run numerical pose-conversion assertions.

## Coding Style & Naming Conventions

Use four-space indentation, `snake_case` modules/functions/variables, and `PascalCase` classes. Follow neighboring Python code and preserve numbered pipeline filenames. Keep training settings in Hydra YAML configurations. No repository-wide formatter or linter configuration is present; avoid unrelated formatting changes.

## Testing Guidelines

Existing tests use plain Python assertions and executable `test()` functions. Name new files `test_<feature>.py` and add focused regression checks for changed behavior. No coverage threshold is configured. Camera tests require capture hardware and may reset USB devices or record video; run them explicitly only in a prepared hardware environment. Report commands, results, and unavailable hardware checks.

## Commit & Pull Request Guidelines

History uses short descriptive subjects such as `Fix lr scheduler typing imports`; no strict prefix convention is evident. Use specific, action-oriented subjects. Pull requests should describe behavior changes, link relevant issues, list validation, and identify hardware or dataset requirements. Include images for visual changes. Keep datasets, checkpoints, recordings, and generated outputs out of commits; use ignored data/output directories.
