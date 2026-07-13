#!/usr/bin/env python3
"""Offline audit of a paired LittleGreen policy YAML/ONNX bundle."""
from __future__ import annotations

import argparse
import hashlib
import math
import sys
from pathlib import Path
from typing import Any

import yaml

try:
    from ament_index_python.packages import get_package_share_directory
except ImportError:  # Allows source-tree use before ROS is installed.
    get_package_share_directory = None

PASS = 0
TEST_FAIL = 2
CONFIG_ERROR = 5
INTERNAL_ERROR = 70
TOLERANCE_RAD = 1.0e-5


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


def audit(policy_path: Path, joint_map_path: Path, onnx_override: Path | None) -> list[str]:
    errors: list[str] = []
    policy = yaml.safe_load(policy_path.read_text(encoding='utf-8'))
    joint_map = yaml.safe_load(joint_map_path.read_text(encoding='utf-8'))
    if not isinstance(policy, dict) or not isinstance(joint_map, dict):
        raise ValueError('policy YAML and joint map must each contain a mapping')

    if int(policy.get('num_observations', -1)) != 45:
        errors.append(f"num_observations is {policy.get('num_observations')}, expected 45")
    if int(policy.get('num_actions', -1)) != 12:
        errors.append(f"num_actions is {policy.get('num_actions')}, expected 12")
    try:
        if not math.isfinite(float(policy.get('policy_dt'))) or float(policy.get('policy_dt')) <= 0.0:
            errors.append('policy_dt must be finite and positive')
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
        errors.append(
            f"action_transform is {policy.get('action_transform')!r}, expected {expected_transform!r}"
        )

    entries = sorted(joint_map.get('joints', []), key=lambda item: int(item['policy_action_index']))
    if len(entries) != 12:
        errors.append(f'joint_map has {len(entries)} action joints, expected 12')
        return errors

    names = [str(item['name']) for item in entries]
    defaults = require_sequence(policy, 'action_default_rad', 12)
    lower = require_sequence(policy, 'action_target_lower_rad', 12)
    upper = require_sequence(policy, 'action_target_upper_rad', 12)
    scales = require_sequence(policy, 'action_residual_scale_rad', 12)
    action_lower = require_scalar_or_sequence(policy, 'action_limit_lower', 12)
    action_upper = require_scalar_or_sequence(policy, 'action_limit_upper', 12)
    indices = require_sequence(policy, 'action_indices', 12)
    sim_names = require_sequence(policy, 'joints', int(policy.get('num_joints', 0)))
    sim_defaults = require_sequence(policy, 'default_joint_positions', int(policy.get('num_joints', 0)))

    for i, entry in enumerate(entries):
        sim_index = int(indices[i])
        if sim_index != int(entry['sim_joint_index']):
            errors.append(
                f'action[{i}] {names[i]} sim index mismatch: policy={sim_index}, joint_map={entry["sim_joint_index"]}'
            )
            continue
        if not 0 <= sim_index < len(sim_names):
            errors.append(f'action[{i}] has out-of-range sim index {sim_index}')
            continue
        if str(sim_names[sim_index]) != names[i]:
            errors.append(
                f'action[{i}] joint name mismatch: policy={sim_names[sim_index]!r}, joint_map={names[i]!r}'
            )
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
        if not (float(lower[i]) <= float(defaults[i]) <= float(upper[i])):
            errors.append(f'action[{i}] {names[i]} default is outside physical bounds')

    if policy.get('deployment_requires_action_contract_transform') is not True:
        errors.append('deployment_requires_action_contract_transform must be true')

    nonuniform = max(map(float, scales)) - min(map(float, scales)) > TOLERANCE_RAD
    if version == 3 and policy.get('deployment_requires_action_contract_v3_transform') is not True:
        errors.append('contract v3 requires deployment_requires_action_contract_v3_transform: true')
    if version == 3 and nonuniform:
        errors.append('contract v3 requires a uniform residual scale; use v4 for a vector profile')
    if version == 4 and not nonuniform:
        errors.append('contract v4 requires a non-uniform residual scale vector')

    if version == 4:
        if not str(policy.get('action_contract_name', '')).strip():
            errors.append('contract v4 requires action_contract_name')
        nominal_lower = require_sequence(policy, 'action_nominal_residual_lower_rad', 12)
        nominal_upper = require_sequence(policy, 'action_nominal_residual_upper_rad', 12)
        if not str(policy.get('deployment_contract_profile', '')).strip():
            errors.append('contract v4 requires deployment_contract_profile')
        if policy.get('deployment_requires_action_contract_v4_transform') is not True:
            errors.append('contract v4 requires deployment_requires_action_contract_v4_transform: true')
        for i in range(12):
            expected_lower = max(float(lower[i]), float(defaults[i]) - float(scales[i]))
            expected_upper = min(float(upper[i]), float(defaults[i]) + float(scales[i]))
            if not close(float(nominal_lower[i]), expected_lower):
                errors.append(
                    f'action[{i}] {names[i]} nominal lower mismatch: '
                    f'policy={float(nominal_lower[i]):.10f}, expected={expected_lower:.10f}'
                )
            if not close(float(nominal_upper[i]), expected_upper):
                errors.append(
                    f'action[{i}] {names[i]} nominal upper mismatch: '
                    f'policy={float(nominal_upper[i]):.10f}, expected={expected_upper:.10f}'
                )

    if policy.get('previous_action_observation') != 'bounded_normalized_action':
        errors.append('previous_action_observation must be bounded_normalized_action')

    if onnx_override is not None:
        onnx_path = onnx_override
    else:
        relative = policy.get('policy_checkpoint_relative_path') or policy.get('policy_checkpoint_filename')
        if not relative:
            raise ValueError('policy YAML has no relative ONNX path or filename')
        onnx_path = (policy_path.parent / str(relative)).resolve()
    if not onnx_path.is_file():
        errors.append(f'paired ONNX file is missing: {onnx_path}')
    else:
        expected_sha = str(policy.get('policy_sha256', '')).lower()
        actual_sha = sha256_file(onnx_path)
        if not expected_sha:
            errors.append('policy_sha256 is missing')
        elif actual_sha != expected_sha:
            errors.append(f'ONNX SHA-256 mismatch: YAML={expected_sha}, file={actual_sha}')

    return errors


def main() -> int:
    default_policy, default_joint_map = default_paths()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--policy-yaml', type=Path, default=default_policy)
    parser.add_argument('--joint-map', type=Path, default=default_joint_map)
    parser.add_argument('--onnx', type=Path, default=None, help='Optional explicit ONNX path.')
    args = parser.parse_args()

    try:
        errors = audit(args.policy_yaml.expanduser().resolve(), args.joint_map.expanduser().resolve(),
                       args.onnx.expanduser().resolve() if args.onnx else None)
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as exc:
        print(f'POLICY BUNDLE AUDIT: CONFIG ERROR\n{exc}', file=sys.stderr)
        return CONFIG_ERROR
    except Exception as exc:  # Defensive boundary for a preflight command.
        print(f'POLICY BUNDLE AUDIT: INTERNAL ERROR\n{exc}', file=sys.stderr)
        return INTERNAL_ERROR

    if errors:
        print('POLICY BUNDLE AUDIT: FAIL')
        for item in errors:
            print(f'FAIL  {item}')
        return TEST_FAIL

    policy = yaml.safe_load(args.policy_yaml.read_text(encoding='utf-8'))
    print('POLICY BUNDLE AUDIT: PASS')
    print(f"contract: v{policy['action_contract_version']} {policy.get('deployment_contract_profile', '')}")
    print(f"task: {policy.get('metadata', {}).get('task', 'unknown')}")
    print(f"policy_dt: {policy.get('policy_dt')} s")
    print(f"policy_sha256: {policy.get('policy_sha256')}")
    return PASS


if __name__ == '__main__':
    raise SystemExit(main())
