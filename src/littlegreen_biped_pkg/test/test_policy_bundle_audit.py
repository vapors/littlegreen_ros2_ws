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
JOINT_MAP = CONFIG_DIR / 'joint_map.yaml'


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


def copy_v231_bundle(tmp_path: Path) -> tuple[Path, Path]:
    for name in (
        'policy.yaml', 'policy.onnx', 'deployment_contract.yaml',
        'policy.sha256', 'bundle_manifest.yaml',
    ):
        (tmp_path / name).write_bytes((CONFIG_DIR / name).read_bytes())
    return tmp_path / 'policy.yaml', tmp_path / 'policy.onnx'


def rewrite_policy(path: Path, mutate) -> None:
    policy = yaml.safe_load(path.read_text(encoding='utf-8'))
    mutate(policy)
    path.write_text(yaml.safe_dump(policy, sort_keys=False), encoding='utf-8')


def test_unmodified_v231_bundle_passes_with_builtin_onnx_inspector(tmp_path: Path) -> None:
    policy_path, onnx_path = copy_v231_bundle(tmp_path)
    errors, warnings, shape = AUDIT.audit(policy_path, JOINT_MAP, onnx_path)
    assert errors == []
    assert warnings == []
    assert shape['input_name'] == 'obs'
    assert shape['input_shape'] == [1, 47]
    assert shape['output_name'] == 'actions'
    assert shape['output_shape'] == [1, 12]


def test_legacy_45_bundle_remains_compatible(tmp_path: Path) -> None:
    legacy = CONFIG_DIR / 'legacy_v280_45d'
    policy_path = tmp_path / 'policy.yaml'
    onnx_path = tmp_path / 'policy.onnx'
    policy_path.write_bytes((legacy / 'policy.yaml').read_bytes())
    onnx_path.write_bytes((legacy / 'policy.onnx').read_bytes())
    probe = make_probe(tmp_path / 'probe', 45)
    errors, warnings, shape = AUDIT.audit(policy_path, JOINT_MAP, onnx_path, probe)
    assert errors == []
    assert warnings
    assert shape['input_shape'] == [1, 45]


def test_old_v280_47_contract_remains_a_separate_compatibility_path(tmp_path: Path) -> None:
    policy_path, onnx_path = copy_v231_bundle(tmp_path)
    def mutate(policy):
        policy.update({
            'observation_contract_version': 2,
            'observation_contract_name': AUDIT.V280_CONTRACT_NAME,
            'observation_layout': AUDIT.V280_LAYOUT,
            'gait_phase_enabled': True,
            'gait_phase_period_s': 0.72,
            'gait_phase_encoding': 'sin_cos_2pi',
            'gait_phase_append_order': 'after_previous_action',
            'gait_phase_training_timebase': 'episode_step_time',
            'gait_phase_training_reset_semantics': 'environment_episode_reset',
        })
    rewrite_policy(policy_path, mutate)
    probe = make_probe(tmp_path / 'probe', 47)
    errors, warnings, _ = AUDIT.audit(policy_path, JOINT_MAP, onnx_path, probe)
    # Companion hashes intentionally no longer match after mutation, but the parser reaches the
    # isolated legacy path and reports its warning.
    assert any('legacy v2.8.0' in warning for warning in warnings)
    assert not any('unsupported 47-D observation_contract_name' in error for error in errors)


def test_yaml_47_with_45_dimensional_onnx_is_rejected(tmp_path: Path) -> None:
    policy_path, onnx_path = copy_v231_bundle(tmp_path)
    probe = make_probe(tmp_path / 'probe', 45)
    errors, _, _ = AUDIT.audit(policy_path, JOINT_MAP, onnx_path, probe)
    assert any('ONNX input shape' in error for error in errors)


def test_v231_compact_layout_mismatch_is_rejected(tmp_path: Path) -> None:
    policy_path, onnx_path = copy_v231_bundle(tmp_path)
    rewrite_policy(policy_path, lambda policy: policy.__setitem__('observation_layout', 'wrong'))
    probe = make_probe(tmp_path / 'probe', 47)
    errors, _, _ = AUDIT.audit(policy_path, JOINT_MAP, onnx_path, probe)
    assert any('compact observation_layout' in error for error in errors)


def test_v231_layout_ranges_mismatch_is_rejected(tmp_path: Path) -> None:
    policy_path, onnx_path = copy_v231_bundle(tmp_path)
    def mutate(policy):
        policy['observation_layout_ranges']['phase_sin_cos'] = [44, 46]
    rewrite_policy(policy_path, mutate)
    probe = make_probe(tmp_path / 'probe', 47)
    errors, _, _ = AUDIT.audit(policy_path, JOINT_MAP, onnx_path, probe)
    assert any('observation_layout_ranges' in error for error in errors)


def test_stand_phase_mode_mismatch_is_rejected(tmp_path: Path) -> None:
    policy_path, onnx_path = copy_v231_bundle(tmp_path)
    rewrite_policy(
        policy_path,
        lambda policy: policy.__setitem__('phase_mode', 'command_synchronized_continuous_nonblocking'),
    )
    probe = make_probe(tmp_path / 'probe', 47)
    errors, _, _ = AUDIT.audit(policy_path, JOINT_MAP, onnx_path, probe)
    assert any('Stand bundle requires phase_mode' in error for error in errors)


def test_walk_without_explicit_stage_is_rejected(tmp_path: Path) -> None:
    policy_path, onnx_path = copy_v231_bundle(tmp_path)
    def mutate(policy):
        policy['metadata']['task_role'] = 'walk'
        policy['phase_mode'] = 'command_synchronized_continuous_nonblocking'
        policy['deployment_requires_random_static_phase_for_stand'] = False
        policy['deployment_requires_command_synchronized_phase_for_walk'] = True
    rewrite_policy(policy_path, mutate)
    probe = make_probe(tmp_path / 'probe', 47)
    errors, _, _ = AUDIT.audit(policy_path, JOINT_MAP, onnx_path, probe)
    assert any('explicitly pin checkpoint stage' in error for error in errors)


def test_missing_complete_bundle_file_is_rejected(tmp_path: Path) -> None:
    policy_path, onnx_path = copy_v231_bundle(tmp_path)
    (tmp_path / 'deployment_contract.yaml').unlink()
    probe = make_probe(tmp_path / 'probe', 47)
    errors, _, _ = AUDIT.audit(policy_path, JOINT_MAP, onnx_path, probe)
    assert any('complete bundle missing' in error for error in errors)


def test_policy_sha256_file_mismatch_is_rejected(tmp_path: Path) -> None:
    policy_path, onnx_path = copy_v231_bundle(tmp_path)
    (tmp_path / 'policy.sha256').write_text('0' * 64 + '  policy.onnx\n', encoding='utf-8')
    probe = make_probe(tmp_path / 'probe', 47)
    errors, _, _ = AUDIT.audit(policy_path, JOINT_MAP, onnx_path, probe)
    assert any('policy.sha256' in error for error in errors)


def test_wrong_onnx_output_dimension_is_rejected(tmp_path: Path) -> None:
    policy_path, onnx_path = copy_v231_bundle(tmp_path)
    probe = make_probe(tmp_path / 'probe', 47, output_dim=14)
    errors, _, _ = AUDIT.audit(policy_path, JOINT_MAP, onnx_path, probe)
    assert any('ONNX output shape' in error for error in errors)


def test_non_float32_onnx_tensor_is_rejected(tmp_path: Path) -> None:
    policy_path, onnx_path = copy_v231_bundle(tmp_path)
    probe = make_probe(tmp_path / 'probe', 47, input_element_type=11)
    errors, _, _ = AUDIT.audit(policy_path, JOINT_MAP, onnx_path, probe)
    assert any('input tensor must be float32' in error for error in errors)
