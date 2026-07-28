from __future__ import annotations

import subprocess
import sys
from pathlib import Path

PACKAGE = Path(__file__).parents[1]
SCRIPT = PACKAGE / 'scripts' / 'policy_golden_vector_compare.py'
FIXTURE = PACKAGE / 'test' / 'golden' / 'v231_stand_observation_vectors.yaml'
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
