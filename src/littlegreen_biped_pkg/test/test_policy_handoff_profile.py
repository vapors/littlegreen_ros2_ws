from __future__ import annotations

import importlib.util
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / 'scripts'
CONFIGS = ROOT / 'src' / 'configs'


def load_module(name: str):
    path = SCRIPTS / f'{name}.py'
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_packaged_handoff_matches_v102_contract():
    handoff = yaml.safe_load((CONFIGS / 'policy_handoff.yaml').read_text())
    assert handoff['schema_version'] == 1
    assert handoff['mode'] == 'learned_zero_command'
    assert handoff['task'] == 'Velocity-Lilgreen-Locomotion-ST3215-Loaded-v102'
    assert handoff['policy_sha256'] == 'da7adcaf996809bfb7a413bfec712dd2f3bd2ed984f8f661af2f9cc9c4d7872e'
    assert len(handoff['joint_position_rad']) == 12
    assert len(handoff['previous_action_bounded']) == 12
    assert handoff['command'] == [0.0, 0.0, 0.0]
    assert handoff['phase'] == [0.0, 1.0]
    assert handoff['max_abs_previous_action_obs_vs_action_term'] == 0.0


def test_packaged_handoff_is_inside_joint_limits():
    handoff = yaml.safe_load((CONFIGS / 'policy_handoff.yaml').read_text())
    joint_map = yaml.safe_load((CONFIGS / 'joint_map.yaml').read_text())
    entries = sorted(joint_map['joints'], key=lambda item: int(item['policy_action_index']))
    for q, entry in zip(handoff['joint_position_rad'], entries):
        assert float(entry['limit_lower_rad']) <= float(q) <= float(entry['limit_upper_rad'])



def test_auditor_accepts_profile_when_policy_identity_matches(tmp_path: Path):
    audit = load_module('policy_bundle_audit')
    policy = {
        'metadata': {'task': 'Velocity-Lilgreen-Locomotion-ST3215-Loaded-v102', 'task_role': 'locomotion'},
        'policy_sha256': 'da7adcaf996809bfb7a413bfec712dd2f3bd2ed984f8f661af2f9cc9c4d7872e',
        'phase_mode': 'neutral_static',
    }
    joint_map = yaml.safe_load((CONFIGS / 'joint_map.yaml').read_text())
    handoff_path = CONFIGS / 'policy_handoff.yaml'
    errors, warnings = [], []
    result = audit.validate_policy_handoff(policy, joint_map, handoff_path, errors, warnings)
    assert result is not None
    assert errors == []


def test_packaged_handoff_separates_track1_source_from_hardware_trim():
    handoff = yaml.safe_load((CONFIGS / 'policy_handoff.yaml').read_text())
    source = [float(x) for x in handoff['source_median_joint_position_rad']]
    trim = [float(x) for x in handoff['hardware_trim_rad']]
    effective = [float(x) for x in handoff['joint_position_rad']]
    assert len(source) == len(trim) == len(effective) == 12
    assert trim == [
        0.0, 0.0, -0.035, 0.0, -0.100, 0.0,
        0.0, 0.0, -0.035, 0.0, -0.100, 0.0,
    ]
    clamps = {int(item['index']): item for item in handoff['physical_limit_clamps']}
    for i, (src, delta, actual) in enumerate(zip(source, trim, effective)):
        expected = src + delta
        if i in clamps:
            clamp = clamps[i]
            assert abs(float(clamp['pre_clamp_rad']) - expected) <= 1.0e-9
            expected = float(clamp['effective_rad'])
        assert abs(actual - expected) <= 1.0e-9


def test_packaged_handoff_records_physical_orientation_calibration():
    handoff = yaml.safe_load((CONFIGS / 'policy_handoff.yaml').read_text())
    provenance = handoff['hardware_trim_provenance']
    assert provenance['calibration_scope'] == 'policy_handoff_only'
    assert provenance['calibrated_on_robot'] is True
    assert provenance['orientation_expectation_result'] == 'PASS'
    gravity = [float(x) for x in provenance['median_projected_gravity_base']]
    assert len(gravity) == 3
    assert gravity[2] < -0.99
