"""Exercise the shell entry point with fake SDK/model processes, without hardware."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time


ROOT = Path(__file__).resolve().parents[1]


def test():
    with tempfile.TemporaryDirectory(prefix='piper_launcher_test_') as temp:
        root = Path(temp)
        umi = root / 'UMI_DM/universal_manipulation_interface'
        session = root / 'UMI_DM/my_session'
        piper = root / 'Piper/Data_Collect'
        (umi / 'scripts').mkdir(parents=True)
        (piper / 'data_collect/cli').mkdir(parents=True)
        (piper / 'bin').mkdir()
        (piper / 'bin/python').symlink_to(sys.executable)
        (piper / 'env.sh').write_text('export PATH="$PWD/bin:$PATH"\n')
        for relative in ('deployment_check/piper_policy_setup.yaml',
                         'training_49/model/checkpoints/latest.ckpt',
                         'calibration/training_mask.json'):
            target = session / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            target.touch()
        launcher = umi / 'scripts/run_piper_policy.sh'
        # Select the test interpreter; the production launcher uses UMI's env.
        launcher.write_text((ROOT / 'scripts/run_piper_policy.sh').read_text().replace(
            'model_python=/home/l1/miniconda3/envs/umi/bin/python',
            'model_python="$PIPER_TEST_PYTHON"'))
        (piper / 'data_collect/cli/umi_piper_sdk.py').write_text('''
import os, pathlib, signal, socket, sys, time
root = pathlib.Path(os.environ['PIPER_TEST_ROOT'])
with (root / 'sdk_invocations').open('a') as file:
    file.write('started\\n')
if '--home' in sys.argv or '--check' in sys.argv:
    (root / 'home_or_check').write_text(sys.argv[-1])
    sys.exit(0)
if (root / 'fail_controller').exists():
    sys.exit(7)
sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
path = sys.argv[sys.argv.index('--socket') + 1]
sock.bind(path)
def stop(*args):
    raise KeyboardInterrupt
signal.signal(signal.SIGTERM, stop)
signal.signal(signal.SIGINT, stop)
try:
    while True:
        time.sleep(.01)
except KeyboardInterrupt:
    pass
finally:
    sock.close()
    pathlib.Path(path).unlink()
    (root / 'controller_stopped').touch()
''')
        (umi / 'scripts/piper_live_observation.py').write_text('''
import json, os, pathlib, signal, sys, time
root = pathlib.Path(os.environ['PIPER_TEST_ROOT'])
(root / 'model_args.json').write_text(json.dumps(sys.argv))
duration = float(sys.argv[sys.argv.index('--duration') + 1])
assert sys.argv[sys.argv.index('--preview-port') + 1] == '8768'
def stop(*args):
    raise KeyboardInterrupt
signal.signal(signal.SIGTERM, stop)
signal.signal(signal.SIGINT, stop)
(root / 'model_started').touch()
try:
    if duration:
        time.sleep(duration)
    else:
        while True:
            time.sleep(.01)
except KeyboardInterrupt:
    pass
finally:
    (root / 'model_stopped').touch()
''')
        (umi / 'scripts/piper_camera_alignment.py').write_text('''
import json, os, pathlib, sys
root = pathlib.Path(os.environ['PIPER_TEST_ROOT'])
(root / 'alignment_args.json').write_text(json.dumps(sys.argv))
''')
        env = {**os.environ, 'PIPER_TEST_PYTHON': sys.executable, 'PIPER_TEST_ROOT': str(root)}
        def run(*args):
            return subprocess.run(['bash', str(launcher), *args], env=env,
                                  capture_output=True, text=True, timeout=10)

        result = run('--home')
        assert result.returncode == 0, result.stdout + result.stderr
        assert (root / 'home_or_check').read_text() == '--home'
        assert not (root / 'model_started').exists()
        result = run('--execute', '.1')
        assert result.returncode == 2, 'Invalid durations must fail before starting SDK'
        result = run('--execute', '0.1')
        assert result.returncode == 0, result.stdout + result.stderr
        assert (root / 'model_stopped').exists() and (root / 'controller_stopped').exists()
        dataset = root / 'reference dataset.zarr.zip'
        dataset.touch()
        result = run('--observe', '0.1', '--match-dataset', str(dataset), '--match-episode', '1',
                     '--match-alpha', '0.35')
        assert result.returncode == 0, result.stdout + result.stderr
        import json
        arguments = json.loads((root / 'model_args.json').read_text())
        assert arguments[arguments.index('--match-dataset') + 1] == str(dataset)
        assert arguments[arguments.index('--match-episode') + 1] == '1'
        assert arguments[arguments.index('--match-alpha') + 1] == '0.35'
        for name in ('model_started', 'model_stopped', 'controller_stopped'):
            (root / name).unlink()
        proc = subprocess.Popen(['bash', str(launcher), '--execute'], env=env,
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                                start_new_session=True)
        try:
            deadline = time.monotonic() + 10
            while not (root / 'model_started').exists() and proc.poll() is None and time.monotonic() < deadline:
                time.sleep(.02)
            assert (root / 'model_started').exists(), 'Continuous model did not start'
            # Terminating the launcher must also stop both children.
            proc.terminate()
            output, _ = proc.communicate(timeout=10)
            assert proc.returncode == 143, output
            assert (root / 'model_stopped').exists() and (root / 'controller_stopped').exists()
        finally:
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.communicate(timeout=5)
        (root / 'model_started').unlink()
        (root / 'fail_controller').touch()
        result = run('--execute', '0.1')
        assert result.returncode == 7, result.stdout + result.stderr
        assert not (root / 'model_started').exists()
        before = (root / 'sdk_invocations').read_bytes()
        # Camera-only alignment must work without a checkpoint or SDK startup.
        (session / 'training_49/model/checkpoints/latest.ckpt').unlink()
        result = run('--align', '0.1', '--match-dataset', str(dataset), '--match-episode', '1')
        assert result.returncode == 0, result.stdout + result.stderr
        assert (root / 'sdk_invocations').read_bytes() == before
        assert not (root / 'model_started').exists()
        arguments = json.loads((root / 'alignment_args.json').read_text())
        assert arguments[arguments.index('--match-episode') + 1] == '1'
        assert '--checkpoint' not in arguments and '--can' not in arguments
        result = run('--align')
        assert result.returncode == 2 and (root / 'sdk_invocations').read_bytes() == before


if __name__ == '__main__':
    test()
    print('Piper launcher: home, timed/continuous execution and child cleanup passed')
