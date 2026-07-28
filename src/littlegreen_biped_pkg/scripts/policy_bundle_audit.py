#!/usr/bin/env python3
"""Offline audit of a complete LittleGreen policy deployment bundle.

The v2.9.0 audit consumes the exported Track 1 schema unchanged.  It validates the
policy YAML, ONNX hash and tensor interface, deployment contract, checksum file,
bundle manifest, canonical joint order, and action-contract-v4 mapping.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import subprocess
import sys
from pathlib import Path
from typing import Any, Iterator

import yaml

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    get_package_share_directory = None

PASS = 0
TEST_FAIL = 2
CONFIG_ERROR = 5
INTERNAL_ERROR = 70
TOLERANCE_RAD = 1.0e-5
LEGACY_OBSERVATIONS = 45
PHASE_OBSERVATIONS = 47
NUM_ACTIONS = 12
V231_CONTRACT_NAME = 'littlegreen_velocity_47d_phase_v1'
V280_CONTRACT_NAME = 'littlegreen_hardware_phase_guided_47_v1'
V231_LAYOUT = (
    'command3,base_ang_vel3,projected_gravity3,joint_pos_rel12,joint_vel12,'
    'previous_bounded_action12,phase_sin1,phase_cos1'
)
V231_RANGES = {
    'command_velocity': [0, 2],
    'base_angular_velocity': [3, 5],
    'projected_gravity': [6, 8],
    'joint_position_relative_default': [9, 20],
    'joint_velocity': [21, 32],
    'previous_bounded_action': [33, 44],
    'phase_sin_cos': [45, 46],
}
V280_LAYOUT = [
    'command_velocity_3',
    'base_angular_velocity_3',
    'projected_gravity_3',
    'joint_position_relative_to_default_12',
    'joint_velocity_12',
    'previous_bounded_normalized_action_12',
    'gait_phase_sin_cos_2',
]


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def default_paths() -> tuple[Path, Path]:
    if get_package_share_directory is not None:
        try:
            share = Path(get_package_share_directory('littlegreen_biped_pkg'))
            return share / 'configs/policy_latest.yaml', share / 'configs/joint_map.yaml'
        except Exception:
            pass
    root = Path.home() / 'littlegreen_ros2_ws' / 'src' / 'littlegreen_biped_pkg' / 'src' / 'configs'
    return root / 'policy_latest.yaml', root / 'joint_map.yaml'


def require_sequence(mapping: dict[str, Any], key: str, length: int) -> list[Any]:
    value = mapping.get(key)
    if not isinstance(value, list) or len(value) != length:
        raise ValueError(f'{key} must contain exactly {length} values')
    return value


def require_scalar_or_sequence(mapping: dict[str, Any], key: str, length: int) -> list[float]:
    value = mapping.get(key)
    if isinstance(value, (int, float)):
        return [float(value)] * length
    if isinstance(value, list) and len(value) == length:
        return [float(item) for item in value]
    raise ValueError(f'{key} must be a scalar or contain exactly {length} values')


def close(a: float, b: float) -> bool:
    return math.isfinite(a) and math.isfinite(b) and abs(a - b) <= TOLERANCE_RAD


def task_role(policy: dict[str, Any]) -> str:
    metadata = policy.get('metadata')
    if isinstance(metadata, dict) and metadata.get('task_role'):
        return str(metadata['task_role'])
    return str(policy.get('task_role', ''))


def task_name(policy: dict[str, Any]) -> str:
    metadata = policy.get('metadata')
    if isinstance(metadata, dict) and metadata.get('task'):
        return str(metadata['task'])
    return str(policy.get('task', ''))


def validate_v231_observation_contract(
    policy: dict[str, Any], errors: list[str], warnings: list[str]
) -> None:
    if int(policy.get('schema_version', -1)) != 2:
        errors.append('v2.3.1 shared contract requires schema_version: 2')
    if int(policy.get('observation_contract_version', -1)) != 1:
        errors.append('v2.3.1 shared contract requires observation_contract_version: 1')
    if policy.get('observation_layout') != V231_LAYOUT:
        errors.append('v2.3.1 compact observation_layout does not match the shared 47-D layout')
    if policy.get('observation_layout_ranges') != V231_RANGES:
        errors.append('observation_layout_ranges do not match the exact inclusive 47-D ranges')
    if policy.get('phase_indices') != [45, 46]:
        errors.append('phase_indices must be [45, 46]')
    if policy.get('phase_encoding') != 'sin_cos_2pi':
        errors.append('phase_encoding must be sin_cos_2pi')
    if policy.get('shared_47d_stand_walk_contract') is not True:
        errors.append('shared_47d_stand_walk_contract must be true')
    if policy.get('legacy_45d_checkpoint_support') is not False:
        errors.append('legacy_45d_checkpoint_support must be false for the v2.3.1 contract')
    if int(policy.get('observation_count', -1)) != PHASE_OBSERVATIONS:
        errors.append('observation_count must be 47')
    if int(policy.get('critic_num_observations', policy.get('critic_observation_count', -1))) != 50:
        errors.append('critic_num_observations must be 50')
    if int(policy.get('critic_observation_count', 50)) != 50:
        errors.append('critic_observation_count must be 50 when present')

    role = task_role(policy)
    mode = str(policy.get('phase_mode', ''))
    try:
        period = float(policy.get('phase_period_s'))
        if not math.isfinite(period) or period <= 0.0:
            raise ValueError
    except (TypeError, ValueError):
        errors.append('phase_period_s must be finite and positive')
    for key in ('phase_transition_fraction', 'phase_linear_command_threshold', 'phase_yaw_command_threshold'):
        try:
            value = float(policy.get(key))
            if not math.isfinite(value) or value < 0.0:
                raise ValueError
        except (TypeError, ValueError):
            errors.append(f'{key} must be finite and non-negative')

    if role == 'stand':
        if mode != 'randomized_static_per_episode':
            errors.append('Stand bundle requires phase_mode: randomized_static_per_episode')
        if policy.get('phase_reset_semantics') != 'sample_uniform_once_for_each_reset_environment_and_hold':
            errors.append('Stand phase_reset_semantics does not match training')
        if policy.get('deployment_requires_random_static_phase_for_stand') is not True:
            errors.append('Stand bundle must require random-static phase deployment')
        if policy.get('deployment_requires_command_synchronized_phase_for_walk') is not False:
            errors.append('Stand bundle must not require the Walk phase generator')
    elif role == 'walk':
        if mode != 'command_synchronized_continuous_nonblocking':
            errors.append('Walk bundle requires phase_mode: command_synchronized_continuous_nonblocking')
        stage_explicit = any(
            key in policy for key in ('phase_deployment_stage', 'deployment_stage')
        ) or policy.get('phase_period_pinned_for_deployment') is True
        if not stage_explicit:
            errors.append(
                'Walk live deployment is blocked: export must explicitly pin checkpoint stage or period'
            )
        if policy.get('deployment_requires_command_synchronized_phase_for_walk') is not True:
            errors.append('Walk bundle must require command-synchronized phase deployment')
    else:
        errors.append('v2.3.1 task_role must be stand or walk')

    if int(policy.get('action_contract_version', 0)) != 4:
        errors.append('v2.3.1 shared 47-D bundle requires action_contract_version: 4')


def validate_observation_contract(
    policy: dict[str, Any], errors: list[str], warnings: list[str]
) -> int:
    try:
        count = int(policy.get('num_observations', -1))
    except (TypeError, ValueError):
        errors.append('num_observations must be an integer')
        return -1
    if count not in (LEGACY_OBSERVATIONS, PHASE_OBSERVATIONS):
        errors.append(
            f'num_observations is {count}; supported contracts are 45-D legacy and 47-D shared phase'
        )
        return count
    if count == LEGACY_OBSERVATIONS:
        version = policy.get('observation_contract_version')
        name = policy.get('observation_contract_name')
        if version is None or name is None:
            warnings.append('legacy 45-D bundle lacks explicit observation metadata')
        else:
            if int(version) != 1:
                errors.append('45-D bundle requires observation_contract_version: 1')
            if str(name) not in {'littlegreen_hardware_45_v1', 'littlegreen_hardware_45_legacy'}:
                errors.append('unsupported 45-D observation_contract_name')
        if policy.get('gait_phase_enabled') is True:
            errors.append('45-D bundle cannot enable phase observations')
        return count

    name = str(policy.get('observation_contract_name', ''))
    if name == V231_CONTRACT_NAME:
        validate_v231_observation_contract(policy, errors, warnings)
    elif name == V280_CONTRACT_NAME:
        warnings.append('using legacy v2.8.0 successful-tick phase compatibility path')
        required = {
            'observation_contract_version': 2,
            'gait_phase_enabled': True,
            'gait_phase_period_s': 0.72,
            'gait_phase_encoding': 'sin_cos_2pi',
            'gait_phase_append_order': 'after_previous_action',
            'gait_phase_training_timebase': 'episode_step_time',
            'gait_phase_training_reset_semantics': 'environment_episode_reset',
        }
        for key, expected in required.items():
            actual = policy.get(key)
            if isinstance(expected, float):
                try:
                    matches = abs(float(actual) - expected) <= 1.0e-9
                except (TypeError, ValueError):
                    matches = False
            else:
                matches = actual == expected
            if not matches:
                errors.append(f'{key} is {actual!r}, expected {expected!r}')
        if policy.get('observation_layout') != V280_LAYOUT:
            errors.append('legacy v2.8.0 observation_layout mismatch')
        if int(policy.get('action_contract_version', 0)) != 4:
            errors.append('legacy 47-D phase bundle requires action_contract_version: 4')
    else:
        errors.append(f'unsupported 47-D observation_contract_name: {name!r}')
    return count


# Minimal ONNX protobuf reader. It deliberately reads only ModelProto.graph and
# GraphProto input/output ValueInfo fields, avoiding a runtime Python ONNX dependency.
def _read_varint(buffer: bytes, index: int) -> tuple[int, int]:
    value = 0
    shift = 0
    while True:
        if index >= len(buffer) or shift > 70:
            raise ValueError('invalid protobuf varint')
        byte = buffer[index]
        index += 1
        value |= (byte & 0x7F) << shift
        if not byte & 0x80:
            return value, index
        shift += 7


def _protobuf_fields(buffer: bytes) -> Iterator[tuple[int, int, Any]]:
    index = 0
    while index < len(buffer):
        key, index = _read_varint(buffer, index)
        number, wire = key >> 3, key & 7
        if wire == 0:
            value, index = _read_varint(buffer, index)
        elif wire == 1:
            value, index = buffer[index:index + 8], index + 8
        elif wire == 2:
            length, index = _read_varint(buffer, index)
            value, index = buffer[index:index + length], index + length
        elif wire == 5:
            value, index = buffer[index:index + 4], index + 4
        else:
            raise ValueError(f'unsupported protobuf wire type {wire}')
        yield number, wire, value


def _length_fields(buffer: bytes, number: int) -> list[bytes]:
    return [value for field, wire, value in _protobuf_fields(buffer) if field == number and wire == 2]


def _value_info(value_info: bytes) -> dict[str, Any]:
    name = ''
    type_proto: bytes | None = None
    for field, wire, value in _protobuf_fields(value_info):
        if field == 1 and wire == 2:
            name = value.decode('utf-8')
        elif field == 2 and wire == 2:
            type_proto = value
    if type_proto is None:
        raise ValueError(f'ONNX ValueInfo {name!r} has no type')
    tensor_fields = _length_fields(type_proto, 1)
    if len(tensor_fields) != 1:
        raise ValueError(f'ONNX ValueInfo {name!r} is not a tensor')
    element_type = -1
    shape_proto: bytes | None = None
    for field, wire, value in _protobuf_fields(tensor_fields[0]):
        if field == 1 and wire == 0:
            element_type = int(value)
        elif field == 2 and wire == 2:
            shape_proto = value
    dimensions: list[Any] = []
    if shape_proto is not None:
        for dimension in _length_fields(shape_proto, 1):
            dim_value: Any = None
            for field, wire, value in _protobuf_fields(dimension):
                if field == 1 and wire == 0:
                    dim_value = int(value)
                elif field == 2 and wire == 2:
                    dim_value = value.decode('utf-8')
            dimensions.append(dim_value)
    return {'name': name, 'shape': dimensions, 'element_type': element_type}


def inspect_onnx_contract(onnx_path: Path) -> dict[str, Any]:
    graphs = _length_fields(onnx_path.read_bytes(), 7)
    if len(graphs) != 1:
        raise ValueError(f'ONNX model contains {len(graphs)} graphs, expected one')
    inputs = [_value_info(value) for value in _length_fields(graphs[0], 11)]
    outputs = [_value_info(value) for value in _length_fields(graphs[0], 12)]
    if len(inputs) != 1 or len(outputs) != 1:
        raise ValueError('ONNX policy must expose exactly one graph input and one graph output')
    return {
        'input_name': inputs[0]['name'],
        'input_shape': inputs[0]['shape'],
        'input_element_type': inputs[0]['element_type'],
        'output_name': outputs[0]['name'],
        'output_shape': outputs[0]['shape'],
        'output_element_type': outputs[0]['element_type'],
    }


def resolve_probe(explicit: Path | None) -> Path | None:
    if explicit is not None:
        return explicit
    env_value = os.environ.get('LITTLEGREEN_ONNX_SHAPE_PROBE', '').strip()
    if env_value:
        return Path(env_value)
    sibling = Path(sys.argv[0]).resolve().parent / 'policy_onnx_contract_probe'
    return sibling if sibling.is_file() else None


def probe_onnx_contract(onnx_path: Path, probe_path: Path) -> dict[str, Any]:
    completed = subprocess.run(
        [str(probe_path), str(onnx_path)], check=False, capture_output=True, text=True, timeout=30.0
    )
    if completed.returncode != 0:
        detail = completed.stderr.strip() or completed.stdout.strip() or 'no diagnostic output'
        raise ValueError(f'ONNX shape probe failed with exit {completed.returncode}: {detail}')
    payload = json.loads(completed.stdout)
    if not isinstance(payload, dict):
        raise ValueError('ONNX shape probe result is not a JSON object')
    return payload


def validate_onnx_shape(shape_info: dict[str, Any], observations: int, errors: list[str]) -> None:
    input_shape = shape_info.get('input_shape')
    output_shape = shape_info.get('output_shape')
    if input_shape != [1, observations]:
        errors.append(f'ONNX input shape is {input_shape!r}, expected [1,{observations}]')
    if output_shape != [1, NUM_ACTIONS]:
        errors.append(f'ONNX output shape is {output_shape!r}, expected [1,{NUM_ACTIONS}]')
    if int(shape_info.get('input_element_type', -1)) != 1:
        errors.append('ONNX input tensor must be float32')
    if int(shape_info.get('output_element_type', -1)) != 1:
        errors.append('ONNX output tensor must be float32')


def companion_path(explicit: Path | None, directory: Path, name: str) -> Path:
    return explicit if explicit is not None else directory / name


def validate_companion_bundle(
    policy: dict[str, Any], policy_path: Path, onnx_path: Path,
    deployment_path: Path, checksum_path: Path, manifest_path: Path,
    errors: list[str], warnings: list[str], require_complete: bool,
) -> None:
    paths = {
        'deployment_contract.yaml': deployment_path,
        'policy.sha256': checksum_path,
        'bundle_manifest.yaml': manifest_path,
    }
    missing = [name for name, path in paths.items() if not path.is_file()]
    if missing:
        message = 'complete bundle missing: ' + ', '.join(missing)
        (errors if require_complete else warnings).append(message)
        if require_complete:
            return

    actual_onnx_sha = sha256_file(onnx_path)
    actual_yaml_sha = sha256_file(policy_path)

    if checksum_path.is_file():
        tokens = checksum_path.read_text(encoding='utf-8').strip().split()
        if not tokens or tokens[0].lower() != actual_onnx_sha:
            errors.append('policy.sha256 does not match policy.onnx')

    if manifest_path.is_file():
        manifest = yaml.safe_load(manifest_path.read_text(encoding='utf-8'))
        if not isinstance(manifest, dict):
            errors.append('bundle_manifest.yaml must contain a mapping')
        else:
            files = manifest.get('files', {})
            if files.get('onnx', {}).get('sha256') != actual_onnx_sha:
                errors.append('bundle manifest ONNX hash mismatch')
            if files.get('yaml', {}).get('sha256') != actual_yaml_sha:
                errors.append('bundle manifest policy YAML hash mismatch')
            interface = manifest.get('interface', {})
            if int(interface.get('observations', -1)) != int(policy.get('num_observations', -2)):
                errors.append('bundle manifest observation count mismatch')
            if int(interface.get('actions', -1)) != int(policy.get('num_actions', -2)):
                errors.append('bundle manifest action count mismatch')
            if interface.get('phase_mode') != policy.get('phase_mode'):
                errors.append('bundle manifest phase_mode mismatch')

    if deployment_path.is_file():
        contract = yaml.safe_load(deployment_path.read_text(encoding='utf-8'))
        if not isinstance(contract, dict):
            errors.append('deployment_contract.yaml must contain a mapping')
        else:
            artifact = contract.get('artifact', {})
            observation = contract.get('observation_contract', {})
            action = contract.get('action_contract', {})
            timing = contract.get('timing', {})
            if contract.get('task') != task_name(policy):
                errors.append('deployment contract task mismatch')
            if contract.get('task_role') != task_role(policy):
                errors.append('deployment contract task_role mismatch')
            if artifact.get('policy_sha256') != actual_onnx_sha:
                errors.append('deployment contract ONNX hash mismatch')
            if artifact.get('onnx_input_shape') != [1, int(policy.get('num_observations', -1))]:
                errors.append('deployment contract ONNX input shape mismatch')
            if artifact.get('onnx_output_shape') != [1, NUM_ACTIONS]:
                errors.append('deployment contract ONNX output shape mismatch')
            if observation.get('name') != policy.get('observation_contract_name'):
                errors.append('deployment contract observation name mismatch')
            if observation.get('actor_count') != policy.get('num_observations'):
                errors.append('deployment contract actor count mismatch')
            if observation.get('phase_mode') != policy.get('phase_mode'):
                errors.append('deployment contract phase_mode mismatch')
            if int(action.get('version', -1)) != int(policy.get('action_contract_version', -2)):
                errors.append('deployment contract action version mismatch')
            if action.get('joint_names') != policy.get('action_joint_names'):
                errors.append('deployment contract action joint order mismatch')
            if action.get('transform') != policy.get('action_transform'):
                errors.append('deployment contract action transform mismatch')
            if action.get('previous_action_observation') != policy.get('previous_action_observation'):
                errors.append('deployment contract previous-action semantics mismatch')
            for key in ('physics_dt', 'policy_dt', 'decimation'):
                if timing.get(key) != policy.get(key):
                    errors.append(f'deployment contract timing {key} mismatch')


def audit(
    policy_path: Path,
    joint_map_path: Path,
    onnx_override: Path | None = None,
    shape_probe: Path | None = None,
    skip_shape_check: bool = False,
    deployment_contract: Path | None = None,
    checksum_file: Path | None = None,
    bundle_manifest: Path | None = None,
) -> tuple[list[str], list[str], dict[str, Any] | None]:
    errors: list[str] = []
    warnings: list[str] = []
    policy = yaml.safe_load(policy_path.read_text(encoding='utf-8'))
    joint_map = yaml.safe_load(joint_map_path.read_text(encoding='utf-8'))
    if not isinstance(policy, dict) or not isinstance(joint_map, dict):
        raise ValueError('policy YAML and joint map must each contain a mapping')

    observations = validate_observation_contract(policy, errors, warnings)
    if int(policy.get('num_actions', -1)) != NUM_ACTIONS:
        errors.append(f"num_actions is {policy.get('num_actions')}, expected {NUM_ACTIONS}")
    try:
        if not math.isfinite(float(policy.get('policy_dt'))) or float(policy.get('policy_dt')) <= 0.0:
            raise ValueError
    except (TypeError, ValueError):
        errors.append('policy_dt must be finite and positive')

    version = int(policy.get('action_contract_version', 0))
    if version not in (3, 4):
        errors.append(f'action_contract_version is {version}, expected 3 or 4')
    expected_transform = {
        3: 'bounded_default_centered_symmetric_residual',
        4: 'bounded_default_centered_vector_residual',
    }.get(version)
    if expected_transform and policy.get('action_transform') != expected_transform:
        errors.append(f"action_transform is {policy.get('action_transform')!r}, expected {expected_transform!r}")

    entries = sorted(joint_map.get('joints', []), key=lambda item: int(item['policy_action_index']))
    if len(entries) != NUM_ACTIONS:
        errors.append(f'joint_map has {len(entries)} action joints, expected {NUM_ACTIONS}')
        return errors, warnings, None

    names = [str(item['name']) for item in entries]
    if policy.get('action_joint_names') is not None and policy.get('action_joint_names') != names:
        errors.append('action_joint_names do not match canonical joint_map order')
    defaults = require_sequence(policy, 'action_default_rad', NUM_ACTIONS)
    lower = require_sequence(policy, 'action_target_lower_rad', NUM_ACTIONS)
    upper = require_sequence(policy, 'action_target_upper_rad', NUM_ACTIONS)
    scales = require_sequence(policy, 'action_residual_scale_rad', NUM_ACTIONS)
    action_lower = require_scalar_or_sequence(policy, 'action_limit_lower', NUM_ACTIONS)
    action_upper = require_scalar_or_sequence(policy, 'action_limit_upper', NUM_ACTIONS)
    indices = require_sequence(policy, 'action_indices', NUM_ACTIONS)
    sim_names = require_sequence(policy, 'joints', int(policy.get('num_joints', 0)))
    sim_defaults = require_sequence(policy, 'default_joint_positions', int(policy.get('num_joints', 0)))

    for i, entry in enumerate(entries):
        sim_index = int(indices[i])
        if sim_index != int(entry['sim_joint_index']):
            errors.append(f'action[{i}] {names[i]} sim index mismatch')
            continue
        if not 0 <= sim_index < len(sim_names):
            errors.append(f'action[{i}] has out-of-range sim index {sim_index}')
            continue
        if str(sim_names[sim_index]) != names[i]:
            errors.append(f'action[{i}] joint name mismatch')
        checks = (
            ('action_default_rad', float(defaults[i]), float(entry['default_joint_rad'])),
            ('default_joint_positions[action_indices]', float(sim_defaults[sim_index]), float(entry['default_joint_rad'])),
            ('action_target_lower_rad', float(lower[i]), float(entry['limit_lower_rad'])),
            ('action_target_upper_rad', float(upper[i]), float(entry['limit_upper_rad'])),
        )
        for label, exported, mapped in checks:
            if not close(exported, mapped):
                errors.append(
                    f'action[{i}] {names[i]} {label} mismatch: policy={exported:.10f}, joint_map={mapped:.10f}'
                )
        if float(scales[i]) <= 0.0:
            errors.append(f'action[{i}] {names[i]} residual scale must be positive')
        if not close(float(action_lower[i]), -1.0) or not close(float(action_upper[i]), 1.0):
            errors.append(f'action[{i}] {names[i]} normalized limits must be [-1, 1]')

    if policy.get('deployment_requires_action_contract_transform') is not True:
        errors.append('deployment_requires_action_contract_transform must be true')
    nonuniform = max(map(float, scales)) - min(map(float, scales)) > TOLERANCE_RAD
    if version == 3 and policy.get('deployment_requires_action_contract_v3_transform') is not True:
        errors.append('contract v3 requires deployment_requires_action_contract_v3_transform: true')
    if version == 3 and nonuniform:
        errors.append('contract v3 requires a uniform residual scale')
    if version == 4 and not nonuniform:
        errors.append('contract v4 requires a non-uniform residual scale vector')
    if version == 4:
        nominal_lower = require_sequence(policy, 'action_nominal_residual_lower_rad', NUM_ACTIONS)
        nominal_upper = require_sequence(policy, 'action_nominal_residual_upper_rad', NUM_ACTIONS)
        if policy.get('deployment_requires_action_contract_v4_transform') is not True:
            errors.append('contract v4 requires deployment_requires_action_contract_v4_transform: true')
        for i in range(NUM_ACTIONS):
            expected_lower = max(float(lower[i]), float(defaults[i]) - float(scales[i]))
            expected_upper = min(float(upper[i]), float(defaults[i]) + float(scales[i]))
            if not close(float(nominal_lower[i]), expected_lower):
                errors.append(f'action[{i}] {names[i]} nominal lower mismatch')
            if not close(float(nominal_upper[i]), expected_upper):
                errors.append(f'action[{i}] {names[i]} nominal upper mismatch')
    if policy.get('previous_action_observation') != 'bounded_normalized_action':
        errors.append('previous_action_observation must be bounded_normalized_action')

    if onnx_override is not None:
        onnx_path = onnx_override
    else:
        relative = policy.get('policy_checkpoint_relative_path') or policy.get('policy_checkpoint_filename')
        if not relative:
            raise ValueError('policy YAML has no relative ONNX path or filename')
        onnx_path = (policy_path.parent / str(relative)).resolve()

    shape_info: dict[str, Any] | None = None
    if not onnx_path.is_file():
        errors.append(f'paired ONNX file is missing: {onnx_path}')
    else:
        expected_sha = str(policy.get('policy_sha256', '')).lower()
        actual_sha = sha256_file(onnx_path)
        if actual_sha != expected_sha:
            errors.append(f'ONNX SHA-256 mismatch: YAML={expected_sha}, file={actual_sha}')
        if skip_shape_check:
            warnings.append('ONNX tensor-shape inspection was explicitly skipped')
        else:
            try:
                if shape_probe is not None:
                    shape_info = probe_onnx_contract(onnx_path, shape_probe)
                else:
                    shape_info = inspect_onnx_contract(onnx_path)
                validate_onnx_shape(shape_info, observations, errors)
            except (OSError, ValueError, json.JSONDecodeError, subprocess.TimeoutExpired) as exc:
                errors.append(f'ONNX tensor-interface inspection failed: {exc}')

        require_complete = observations == PHASE_OBSERVATIONS and policy.get('observation_contract_name') == V231_CONTRACT_NAME
        directory = policy_path.parent
        validate_companion_bundle(
            policy, policy_path, onnx_path,
            companion_path(deployment_contract, directory, 'deployment_contract.yaml'),
            companion_path(checksum_file, directory, 'policy.sha256'),
            companion_path(bundle_manifest, directory, 'bundle_manifest.yaml'),
            errors, warnings, require_complete,
        )

    return errors, warnings, shape_info


def main() -> int:
    default_policy, default_joint_map = default_paths()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--policy-yaml', type=Path, default=default_policy)
    parser.add_argument('--joint-map', type=Path, default=default_joint_map)
    parser.add_argument('--onnx', type=Path, default=None)
    parser.add_argument('--deployment-contract', type=Path, default=None)
    parser.add_argument('--policy-sha256-file', type=Path, default=None)
    parser.add_argument('--bundle-manifest', type=Path, default=None)
    parser.add_argument('--onnx-shape-probe', type=Path, default=None)
    parser.add_argument('--skip-onnx-shape-check', action='store_true')
    args = parser.parse_args()

    try:
        probe = resolve_probe(args.onnx_shape_probe.expanduser().resolve() if args.onnx_shape_probe else None)
        errors, warnings, shape_info = audit(
            args.policy_yaml.expanduser().resolve(),
            args.joint_map.expanduser().resolve(),
            args.onnx.expanduser().resolve() if args.onnx else None,
            probe,
            args.skip_onnx_shape_check,
            args.deployment_contract.expanduser().resolve() if args.deployment_contract else None,
            args.policy_sha256_file.expanduser().resolve() if args.policy_sha256_file else None,
            args.bundle_manifest.expanduser().resolve() if args.bundle_manifest else None,
        )
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as exc:
        print(f'POLICY BUNDLE AUDIT: CONFIG ERROR\n{exc}', file=sys.stderr)
        return CONFIG_ERROR
    except Exception as exc:
        print(f'POLICY BUNDLE AUDIT: INTERNAL ERROR\n{exc}', file=sys.stderr)
        return INTERNAL_ERROR

    if errors:
        print('POLICY BUNDLE AUDIT: FAIL')
        for item in errors:
            print(f'FAIL  {item}')
        for item in warnings:
            print(f'WARN  {item}')
        return TEST_FAIL

    policy = yaml.safe_load(args.policy_yaml.read_text(encoding='utf-8'))
    print('POLICY BUNDLE AUDIT: PASS')
    print(f"task: {task_name(policy)} ({task_role(policy)})")
    print(
        f"observation_contract: v{policy.get('observation_contract_version', 1)} "
        f"{policy.get('observation_contract_name', 'legacy_45_compatibility')}"
    )
    print(f"phase_mode: {policy.get('phase_mode', 'none')}")
    print(f"interface: obs[{policy.get('num_observations')}] -> actions[{policy.get('num_actions')}]")
    print(f"action_contract: v{policy['action_contract_version']} {policy.get('deployment_contract_profile', '')}")
    print(f"policy_dt: {policy.get('policy_dt')} s")
    print(f"policy_sha256: {policy.get('policy_sha256')}")
    if shape_info is not None:
        print(f"onnx_input: {shape_info.get('input_name')} {shape_info.get('input_shape')}")
        print(f"onnx_output: {shape_info.get('output_name')} {shape_info.get('output_shape')}")
    for item in warnings:
        print(f'WARN  {item}')
    return PASS


if __name__ == '__main__':
    raise SystemExit(main())
