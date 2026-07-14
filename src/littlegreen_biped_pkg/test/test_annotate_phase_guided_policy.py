from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest

MODULE_PATH = Path(__file__).parents[1] / 'scripts' / 'annotate_phase_guided_policy.py'
SPEC = importlib.util.spec_from_file_location('annotate_phase_guided_policy', MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
ANNOTATE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ANNOTATE)


def genuine_export_stub() -> dict:
    return {
        'metadata': {'task': ANNOTATE.EXPECTED_TASK},
        'num_observations': 47,
        'num_actions': 12,
        'action_contract_version': 4,
        'policy_dt': 0.02,
        'policy_sha256': '0' * 64,
    }


def test_annotates_only_contract_metadata() -> None:
    policy = genuine_export_stub()
    result = ANNOTATE.annotate(policy)
    assert result['num_observations'] == 47
    assert result['policy_sha256'] == policy['policy_sha256']
    assert result['observation_contract_version'] == 2
    assert result['gait_phase_period_s'] == 0.72
    assert result['observation_layout'][-1] == 'gait_phase_sin_cos_2'
    assert 'observation_contract_version' not in policy


def test_refuses_45_dimensional_bundle() -> None:
    policy = genuine_export_stub()
    policy['num_observations'] = 45
    with pytest.raises(ValueError, match='must already be 47'):
        ANNOTATE.annotate(policy)


def test_refuses_unexpected_task() -> None:
    policy = genuine_export_stub()
    policy['metadata']['task'] = 'Velocity-Lilgreen-Stand-ST3215-Loaded-v5s3'
    with pytest.raises(ValueError, match='expected'):
        ANNOTATE.annotate(policy)
