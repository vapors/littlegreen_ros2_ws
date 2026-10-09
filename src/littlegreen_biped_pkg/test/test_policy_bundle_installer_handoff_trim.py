from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / 'scripts'
CONFIGS = ROOT / 'src' / 'configs'


def load_installer():
    path = SCRIPTS / 'install_exported_policy_bundle.py'
    spec = importlib.util.spec_from_file_location('installer', path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def analyzer_json_from_packaged_handoff(tmp_path: Path) -> Path:
    handoff = yaml.safe_load((CONFIGS / 'policy_handoff.yaml').read_text())
    policy = yaml.safe_load((CONFIGS / 'policy_latest.yaml').read_text())
    data = {
        'schema_version': 2,
        'task': handoff['task'],
        'checkpoint': handoff['source']['checkpoint'],
        'joint_order': handoff['joint_order'],
        'median_joint_position_rad': handoff['source_median_joint_position_rad'],
        'median_previous_bounded_action': handoff['previous_action_bounded'],
        'q_default_rad': policy['action_default_rad'],
        'command': [0.0, 0.0, 0.0],
        'median_projected_gravity_b': handoff['source_median_projected_gravity_b'],
        'previous_action_observation_indices': [33, 44],
        'previous_action_observation_semantics': handoff['previous_action_observation_semantics'],
        'max_abs_previous_action_obs_vs_action_term': 0.0,
        'selection': handoff['source']['selection'],
    }
    path = tmp_path / 'zero_command_handoff_pose.json'
    path.write_text(json.dumps(data), encoding='utf-8')
    return path


def test_reinstall_same_policy_preserves_calibrated_hardware_trim(tmp_path: Path):
    installer = load_installer()
    source = analyzer_json_from_packaged_handoff(tmp_path)
    profile = installer.normalize_handoff_profile(
        source,
        CONFIGS / 'policy_latest.yaml',
        CONFIGS / 'joint_map.yaml',
        CONFIGS / 'policy_handoff.yaml',
    )
    assert profile['hardware_trim_rad'] == [
        0.0, 0.0, -0.035, 0.0, -0.100, 0.0,
        0.0, 0.0, -0.035, 0.0, -0.100, 0.0,
    ]
    assert profile['hardware_trim_provenance']['calibrated_on_robot'] is True
    expected = yaml.safe_load((CONFIGS / 'policy_handoff.yaml').read_text())['joint_position_rad']
    assert all(abs(float(a) - float(b)) <= 1.0e-9 for a, b in zip(profile['joint_position_rad'], expected))


def test_new_policy_identity_does_not_inherit_old_hardware_trim(tmp_path: Path):
    installer = load_installer()
    source = analyzer_json_from_packaged_handoff(tmp_path)
    policy = yaml.safe_load((CONFIGS / 'policy_latest.yaml').read_text())
    policy['policy_sha256'] = '0' * 64
    if isinstance(policy.get('metadata'), dict):
        policy['metadata']['policy_sha256'] = '0' * 64
    new_policy = tmp_path / 'policy.yaml'
    new_policy.write_text(yaml.safe_dump(policy, sort_keys=False), encoding='utf-8')
    profile = installer.normalize_handoff_profile(
        source,
        new_policy,
        CONFIGS / 'joint_map.yaml',
        CONFIGS / 'policy_handoff.yaml',
    )
    assert profile['hardware_trim_rad'] == [0.0] * 12
    assert profile['hardware_trim_provenance']['calibrated_on_robot'] is False
