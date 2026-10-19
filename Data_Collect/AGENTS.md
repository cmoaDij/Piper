# Repository Guidelines

## Project Structure & Module Organization

All collection code and resources live in `Data_Collect/` at the repository root. Inside it, `data_collect/` contains `application/` for orchestration, `control/` for robot targets and motion guards, `sensors/` for cameras/tactile acquisition, `recording/` for HDF5/video, `calibration/` for delay tools, `dataset/` for audit/export, `web/` for local services/assets, `common/` for shared infrastructure, and `cli/` for commands.

Keep ROS packages in `ros_ws/src/`, device settings and dependency constraints in `config/`, third-party SDKs in `vendor/`, tests in `tests/`, and operational documentation in `docs/`. Preserve both standard Piper and Piper-X models; current collection uses Piper-X.

## Build, Test, and Development Commands

Run from `Data_Collect/` after rebuilding the documented environment:

```bash
source env.sh
python3 -m data_collect.cli.gui --demo --no-browser --port 8767
python3 -m data_collect.cli.gui
ros2 launch ./launch/collect.launch.py can_port:=can0
python3 -m data_collect.cli.review
source env.sh export
python3 -m data_collect.cli.export --help
```

Build internal ROS packages from `ros_ws/` with `colcon --log-base ../data/logs/build build --symlink-install --cmake-args -DBUILD_TESTING=OFF -DPython3_EXECUTABLE="$CONDA_PREFIX/bin/python"`. Do not source system ROS after `env.sh`.

## Coding Style & Naming Conventions

Use four-space Python indentation, snake_case modules/functions, PascalCase classes, and UPPER_CASE constants. Match surrounding code. Keep CLI modules thin, resolve resources through `common/paths.py`, and store device/motion settings in YAML. No repository-wide formatter is configured.

## Testing Guidelines

Install offline development dependencies with `python -m pip install -e '.[dev]'`, then run `python -m pytest`. Name tests `test_*.py`; use `tests/unit/` for guards and timing logic, `tests/integration/` for synthetic recording/audit/export and HTTP checks, and `tests/fixtures/` for small generators. Never silently substitute synthetic checks for hardware validation. No numerical coverage threshold is configured.

## Commit & Pull Request Guidelines

Use imperative commit summaries; history also includes chore: prefixes. Work on feature branches and submit PRs to main for review. Explain behavior changes and validation results; include screenshots for UI changes. Never force-push.

## Configuration & Hardware Safety

Preserve the single SDK control outlet and existing motion guards. Keep raw datasets, runtime environments, build output, real calibration and credentials out of Git. Verify CAN/USB mappings, calibration and emergency-stop availability before hardware runs. Do not trim SDK drivers or effectors without resolving their imports and testing the result.
