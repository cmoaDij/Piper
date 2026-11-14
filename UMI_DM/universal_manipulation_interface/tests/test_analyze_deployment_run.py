import json
import tempfile
from pathlib import Path

from scripts.analyze_deployment_run import main


def test():
    with tempfile.TemporaryDirectory(prefix='umi_analysis_') as directory:
        root = Path(directory)
        (root / 'live_observation.jsonl').write_text(json.dumps({
            'inference_s': .05, 'frame_age_at_prediction_s': .08
        }) + '\n')
        (root / 'control_events.jsonl').write_text(json.dumps({
            'event': 'sdk_calls_returned',
            'actual_flange_xyz_rpy': [0, 0, 0, 0, 0, 0],
            'predicted_flange_xyz_rpy': [.001, 0, 0, 0, 0, 0],
            'sdk_call_timing': {'arm_call_start_monotonic_s': 1.,
                                'arm_call_return_monotonic_s': 1.002}
        }) + '\n')
        output = root / 'analysis'
        import sys
        old = sys.argv
        sys.argv = ['analyze_deployment_run', '--input-dir', str(root), '--output-dir', str(output)]
        try:
            main()
        finally:
            sys.argv = old
        result = json.loads((output / 'deployment_summary.json').read_text())
        assert result['prediction_packets'] == 1
        assert result['successful_sdk_events'] == 1
        # A pre-send snapshot cannot establish tracking error at a future
        # waypoint. No extrapolated "error" should be reported from one sample.
        assert result['target_time_aligned_tcp_error_mm']['count'] == 0


if __name__ == '__main__':
    test()
    print('Deployment analyzer checks passed')
