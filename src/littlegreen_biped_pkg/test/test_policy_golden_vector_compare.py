from __future__ import annotations

import importlib.util
import subprocess
import sys
from pathlib import Path

PACKAGE = Path(__file__).parents[1]
SCRIPT = PACKAGE / 'scripts' / 'policy_golden_vector_compare.py'
FIXTURE = PACKAGE / 'test' / 'golden' / 'v102_locomotion_observation_vectors.yaml'
POLICY = PACKAGE / 'src' / 'configs' / 'policy.yaml'


def test_packaged_observation_vectors_match_track2_builder() -> None:
    completed = subprocess.run(
        [
            sys.executable,
            str(SCRIPT),
            '--fixture', str(FIXTURE),
            '--policy-yaml', str(POLICY),
            '--skip-onnx',
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    assert completed.returncode == 0, completed.stdout + completed.stderr
    assert 'GOLDEN VECTOR COMPARISON: PASS' in completed.stdout


def test_neutral_static_builder_forces_zero_one_phase_tail() -> None:
    spec = importlib.util.spec_from_file_location('policy_golden_vector_compare', SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    vector = {
        'command_velocity': [0.4, 0.0, 0.2],
        'base_angular_velocity': [0.0, 0.0, 0.0],
        'projected_gravity': [0.0, 0.0, -1.0],
        'joint_position_relative_default': [0.0] * 12,
        'joint_velocity': [0.0] * 12,
        'previous_bounded_action': [0.0] * 12,
    }
    observation = module.build_observation(vector, {'phase_mode': 'neutral_static'})
    assert len(observation) == 47
    assert observation[45:] == [0.0, 1.0]
