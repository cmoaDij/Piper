"""Check environment activation and argument forwarding without starting ROS."""

import json
import os
import shutil
import subprocess

import pytest

from data_collect.common.paths import ROOT


@pytest.mark.parametrize("arguments,executable,expected", [
    (["--can-port", "can7", "--no-browser"], "python",
     ["-m", "data_collect.cli.gui", "--start", "--can-port", "can7", "--no-browser"]),
    (["--terminal", "can_port:=can7"], "ros2", None),
    (["--terminal", "can_port:=can7", "teleop_only:=true"], "ros2", None),
])
def test_start_loads_environment_and_forwards_arguments(tmp_path, arguments, executable, expected):
    project = tmp_path / "project with spaces"
    project.mkdir()
    shutil.copy2(ROOT / "start.sh", project / "start.sh")
    install = project / "ros_ws/install"
    install.mkdir(parents=True)
    (install / "local_setup.bash").touch()
    (project / "env.sh").write_text(
        'if [[ "${1:-capture}" != capture ]]; then\n'
        ' echo "用法：source env.sh [capture|export]" >&2\n'
        ' return 1\n'
        'fi\n'
        'export PIPER_TEST_ENV=loaded\nexport PATH="$PWD/bin:$PATH"\n'
    )
    binaries = project / "bin"
    binaries.mkdir()
    output = project / "invocation.json"
    for name in ("python", "ros2", "adb"):
        script = binaries / name
        script.write_text(
            "#!/usr/bin/python3\nimport json, os, sys\n"
            "from pathlib import Path\n"
            "if Path(sys.argv[0]).name != 'adb':\n"
            " Path('invocation.json').write_text(json.dumps([sys.argv, os.getcwd(), os.environ['PIPER_TEST_ENV']]))\n"
        )
        script.chmod(0o755)
    result = subprocess.run([str(project / "start.sh"), *arguments], cwd=tmp_path,
                            capture_output=True, text=True, timeout=5)
    assert result.returncode == 0, result.stderr
    command, cwd, activated = json.loads(output.read_text())
    assert command[0].endswith("/" + executable)
    if executable == "ros2":
        expected = ["launch", str(project / "launch/collect.launch.py"), *arguments[1:]]
    assert command[1:] == expected
    assert cwd == str(project) and activated == "loaded"


def test_start_help_needs_no_runtime_environment():
    result = subprocess.run([str(ROOT / "start.sh"), "--help"], env={**os.environ},
                            capture_output=True, text=True, timeout=5)
    assert result.returncode == 0 and "--terminal" in result.stdout


def test_missing_capture_environment_points_to_installer(tmp_path):
    project = tmp_path / "project with spaces"
    project.mkdir()
    shutil.copy2(ROOT / "env.sh", project / "env.sh")
    result = subprocess.run(
        ["bash", "-c", 'source "$1" capture', "bash", str(project / "env.sh")],
        capture_output=True, text=True, timeout=5,
    )
    assert result.returncode != 0
    assert str(project / ".runtime/capture") in result.stderr
    assert str(project / "setup_teleop.sh") in result.stderr
