from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path

import yaml

MODULE_PATH = Path(__file__).parents[1] / 'scripts' / 'policy_bundle_audit.py'
SPEC = importlib.util.spec_from_file_location('policy_bundle_audit', MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
AUDIT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(AUDIT)

CONFIG_DIR = Path(__file__).parents[1] / 'src' / 'configs'


def make_probe(
    path: Path,
    input_dim: int,
    output_dim: int = 12,
    input_element_type: int = 1,
    output_element_type: int = 1,
) -> Path:
    payload = {
        'input_name': 'obs',
        'output_name': 'actions',
        'input_shape': [1, input_dim],
        'output_shape': [1, output_dim],
        'input_element_type': input_element_type,
        'output_element_type': output_element_type,
    }
    path.write_text(
        '#!/usr/bin/env python3\n'
        f'import json\nprint(json.dumps({payload!r}))\n',
        encoding='utf-8',
    )
    path.chmod(0o755)
    return path


def make_bundle(tmp_path: Path, observations: int) -> tuple[Path, Path, Path]:
    policy = yaml.safe_load((CONFIG_DIR / 'policy_latest.yaml').read_text(encoding='utf-8'))
    joint_map_path = CONFIG_DIR / 'joint_map.yaml'
    onnx_path = tmp_path / 'policy.onnx'
    onnx_path.write_bytes(b'synthetic-test-fixture-not-a-deployment-model')
    policy['policy_checkpoint_relative_path'] = 'policy.onnx'
    policy['policy_checkpoint_filename'] = 'policy.onnx'
    policy['policy_sha256'] = hashlib.sha256(onnx_path.read_bytes()).hexdigest()
    policy['num_observations'] = observations

    if observations == 47:
        policy.update({
            'policy_dt': 0.02,
            'observation_contract_version': 2,
            'observation_contract_name': 'littlegreen_hardware_phase_guided_47_v1',
            'observation_layout': AUDIT.PHASE_LAYOUT,
            'gait_phase_enabled': True,
            'gait_phase_period_s': 0.72,
            'gait_phase_encoding': 'sin_cos_2pi',
            'gait_phase_append_order': 'after_previous_action',
            'gait_phase_training_timebase': 'episode_step_time',
            'gait_phase_training_reset_semantics': 'environment_episode_reset',
        })

    policy_path = tmp_path / 'policy.yaml'
    policy_path.write_text(yaml.safe_dump(policy, sort_keys=False), encoding='utf-8')
    return policy_path, joint_map_path, onnx_path


def test_legacy_45_bundle_remains_compatible(tmp_path: Path) -> None:
    policy_path, joint_map_path, onnx_path = make_bundle(tmp_path, 45)
    probe = make_probe(tmp_path / 'probe', 45)
    errors, warnings, shape = AUDIT.audit(
        policy_path, joint_map_path, onnx_path, probe, False
    )
    assert errors == []
    assert warnings
    assert shape['input_shape'] == [1, 45]


def test_phase_guided_47_bundle_passes_with_explicit_metadata(tmp_path: Path) -> None:
    policy_path, joint_map_path, onnx_path = make_bundle(tmp_path, 47)
    probe = make_probe(tmp_path / 'probe', 47)
    errors, warnings, shape = AUDIT.audit(
        policy_path, joint_map_path, onnx_path, probe, False
    )
    assert errors == []
    assert warnings == []
    assert shape['input_shape'] == [1, 47]


def test_yaml_47_with_45_dimensional_onnx_is_rejected(tmp_path: Path) -> None:
    policy_path, joint_map_path, onnx_path = make_bundle(tmp_path, 47)
    probe = make_probe(tmp_path / 'probe', 45)
    errors, _, _ = AUDIT.audit(policy_path, joint_map_path, onnx_path, probe, False)
    assert any('ONNX input shape' in error for error in errors)


def test_47_bundle_missing_phase_metadata_is_rejected(tmp_path: Path) -> None:
    policy_path, joint_map_path, onnx_path = make_bundle(tmp_path, 47)
    policy = yaml.safe_load(policy_path.read_text(encoding='utf-8'))
    del policy['gait_phase_period_s']
    policy_path.write_text(yaml.safe_dump(policy, sort_keys=False), encoding='utf-8')
    probe = make_probe(tmp_path / 'probe', 47)
    errors, _, _ = AUDIT.audit(policy_path, joint_map_path, onnx_path, probe, False)
    assert any('gait_phase_period_s' in error for error in errors)


def test_unsupported_observation_count_is_rejected(tmp_path: Path) -> None:
    policy_path, joint_map_path, onnx_path = make_bundle(tmp_path, 46)
    probe = make_probe(tmp_path / 'probe', 46)
    errors, _, _ = AUDIT.audit(policy_path, joint_map_path, onnx_path, probe, False)
    assert any('supported contracts' in error for error in errors)


def test_legacy_45_bundle_cannot_enable_phase(tmp_path: Path) -> None:
    policy_path, joint_map_path, onnx_path = make_bundle(tmp_path, 45)
    policy = yaml.safe_load(policy_path.read_text(encoding='utf-8'))
    policy['gait_phase_enabled'] = True
    policy_path.write_text(yaml.safe_dump(policy, sort_keys=False), encoding='utf-8')
    probe = make_probe(tmp_path / 'probe', 45)
    errors, _, _ = AUDIT.audit(policy_path, joint_map_path, onnx_path, probe, False)
    assert any('cannot set gait_phase_enabled' in error for error in errors)


def test_phase_guided_bundle_requires_action_contract_v4(tmp_path: Path) -> None:
    policy_path, joint_map_path, onnx_path = make_bundle(tmp_path, 47)
    policy = yaml.safe_load(policy_path.read_text(encoding='utf-8'))
    policy['action_contract_version'] = 3
    policy_path.write_text(yaml.safe_dump(policy, sort_keys=False), encoding='utf-8')
    probe = make_probe(tmp_path / 'probe', 47)
    errors, _, _ = AUDIT.audit(policy_path, joint_map_path, onnx_path, probe, False)
    assert any('requires action_contract_version: 4' in error for error in errors)


def test_wrong_onnx_output_dimension_is_rejected(tmp_path: Path) -> None:
    policy_path, joint_map_path, onnx_path = make_bundle(tmp_path, 47)
    probe = make_probe(tmp_path / 'probe', 47, output_dim=14)
    errors, _, _ = AUDIT.audit(policy_path, joint_map_path, onnx_path, probe, False)
    assert any('ONNX output shape' in error for error in errors)


def test_non_float32_onnx_tensor_is_rejected(tmp_path: Path) -> None:
    policy_path, joint_map_path, onnx_path = make_bundle(tmp_path, 47)
    probe = make_probe(tmp_path / 'probe', 47, input_element_type=11)
    errors, _, _ = AUDIT.audit(policy_path, joint_map_path, onnx_path, probe, False)
    assert any('input tensor must be float32' in error for error in errors)
