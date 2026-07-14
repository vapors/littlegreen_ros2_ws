#!/usr/bin/env python3
"""Offline audit of a paired LittleGreen policy YAML/ONNX deployment bundle."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import subprocess
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
LEGACY_OBSERVATIONS = 45
PHASE_OBSERVATIONS = 47
NUM_ACTIONS = 12
PHASE_CONTRACT_NAME = 'littlegreen_hardware_phase_guided_47_v1'
PHASE_LAYOUT = [
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
            f'num_observations is {count}; supported contracts are '
            f'{LEGACY_OBSERVATIONS}-D legacy and {PHASE_OBSERVATIONS}-D phase-guided'
        )
        return count

    if count == LEGACY_OBSERVATIONS:
        version = policy.get('observation_contract_version')
        name = policy.get('observation_contract_name')
        if version is None or name is None:
            warnings.append(
                'legacy 45-D bundle has no explicit observation-contract metadata; '
                'accepted for legacy compatibility'
            )
        else:
            if int(version) != 1:
                errors.append('45-D bundle requires observation_contract_version: 1')
            if str(name) not in {
                'littlegreen_hardware_45_v1',
                'littlegreen_hardware_45_legacy',
            }:
                errors.append(
                    '45-D observation_contract_name must be '
                    'littlegreen_hardware_45_v1 or littlegreen_hardware_45_legacy'
                )
        if policy.get('gait_phase_enabled') is True:
            errors.append('45-D bundle cannot set gait_phase_enabled: true')
        return count

    required = {
        'observation_contract_version': 2,
        'observation_contract_name': PHASE_CONTRACT_NAME,
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
                matches = math.isfinite(float(actual)) and abs(float(actual) - expected) <= 1.0e-9
            except (TypeError, ValueError):
                matches = False
        else:
            matches = actual == expected
        if not matches:
            errors.append(f'{key} is {actual!r}, expected {expected!r}')

    if policy.get('observation_layout') != PHASE_LAYOUT:
        errors.append('47-D observation_layout does not match the supported append-only phase layout')

    try:
        policy_dt = float(policy.get('policy_dt'))
        if not math.isfinite(policy_dt) or policy_dt <= 0.0:
            raise ValueError('non-finite or non-positive policy_dt')
        if abs(policy_dt - 0.02) > 1.0e-9:
            errors.append('phase-guided observation contract v1 requires policy_dt: 0.02')
        exact_ticks = 0.72 / policy_dt
        if abs(exact_ticks - round(exact_ticks)) > 1.0e-6 or round(exact_ticks) != 36:
            errors.append('gait phase must resolve to exactly 36 policy ticks per 0.72 s period')
    except (TypeError, ValueError, OverflowError, ZeroDivisionError):
        errors.append('phase-guided observation contract requires finite positive policy_dt')

    if int(policy.get('action_contract_version', 0)) != 4:
        errors.append('47-D phase-guided bundle requires action_contract_version: 4')
    return count


def resolve_probe(explicit: Path | None) -> Path | None:
    if explicit is not None:
        return explicit
    env_value = os.environ.get('LITTLEGREEN_ONNX_SHAPE_PROBE', '').strip()
    if env_value:
        return Path(env_value)
    sibling = Path(sys.argv[0]).resolve().parent / 'policy_onnx_contract_probe'
    return sibling if sibling.is_file() else None


def probe_onnx_contract(onnx_path: Path, probe_path: Path) -> dict[str, Any]:
    try:
        completed = subprocess.run(
            [str(probe_path), str(onnx_path)],
            check=False,
            capture_output=True,
            text=True,
            timeout=30.0,
        )
    except subprocess.TimeoutExpired as exc:
        raise ValueError('ONNX shape probe timed out after 30 seconds') from exc
    except OSError as exc:
        raise ValueError(f'failed to execute ONNX shape probe: {exc}') from exc
    if completed.returncode != 0:
        detail = completed.stderr.strip() or completed.stdout.strip() or 'no diagnostic output'
        raise ValueError(
            f'ONNX shape probe failed with exit {completed.returncode}: {detail}'
        )
    try:
        payload = json.loads(completed.stdout)
    except json.JSONDecodeError as exc:
        raise ValueError(f'ONNX shape probe returned invalid JSON: {exc}') from exc
    if not isinstance(payload, dict):
        raise ValueError('ONNX shape probe result is not a JSON object')
    return payload


def validate_onnx_shape(
    shape_info: dict[str, Any], observations: int, errors: list[str]
) -> None:
    input_shape = shape_info.get('input_shape')
    output_shape = shape_info.get('output_shape')
    if not isinstance(input_shape, list) or len(input_shape) != 2:
        errors.append(f'ONNX input shape is {input_shape!r}, expected rank-2 [1,{observations}]')
    else:
        if int(input_shape[-1]) != observations:
            errors.append(
                f'ONNX input shape is {input_shape!r}, YAML requires [1,{observations}]'
            )
        if int(input_shape[0]) > 0 and int(input_shape[0]) != 1:
            errors.append(f'ONNX input batch dimension is {input_shape[0]}, expected 1 or dynamic')

    if not isinstance(output_shape, list) or len(output_shape) != 2:
        errors.append(f'ONNX output shape is {output_shape!r}, expected rank-2 [1,{NUM_ACTIONS}]')
    else:
        if int(output_shape[-1]) != NUM_ACTIONS:
            errors.append(
                f'ONNX output shape is {output_shape!r}, expected [1,{NUM_ACTIONS}]'
            )
        if int(output_shape[0]) > 0 and int(output_shape[0]) != 1:
            errors.append(f'ONNX output batch dimension is {output_shape[0]}, expected 1 or dynamic')

    if int(shape_info.get('input_element_type', -1)) != 1:
        errors.append('ONNX input tensor must be float32')
    if int(shape_info.get('output_element_type', -1)) != 1:
        errors.append('ONNX output tensor must be float32')


def audit(
    policy_path: Path,
    joint_map_path: Path,
    onnx_override: Path | None,
    shape_probe: Path | None,
    skip_shape_check: bool,
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
    if len(entries) != NUM_ACTIONS:
        errors.append(f'joint_map has {len(entries)} action joints, expected {NUM_ACTIONS}')
        return errors, warnings, None

    names = [str(item['name']) for item in entries]
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
            errors.append(
                f'action[{i}] {names[i]} sim index mismatch: '
                f'policy={sim_index}, joint_map={entry["sim_joint_index"]}'
            )
            continue
        if not 0 <= sim_index < len(sim_names):
            errors.append(f'action[{i}] has out-of-range sim index {sim_index}')
            continue
        if str(sim_names[sim_index]) != names[i]:
            errors.append(
                f'action[{i}] joint name mismatch: '
                f'policy={sim_names[sim_index]!r}, joint_map={names[i]!r}'
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
                    f'action[{i}] {names[i]} {label} mismatch: '
                    f'policy={exported:.10f}, joint_map={mapped:.10f}'
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
        nominal_lower = require_sequence(policy, 'action_nominal_residual_lower_rad', NUM_ACTIONS)
        nominal_upper = require_sequence(policy, 'action_nominal_residual_upper_rad', NUM_ACTIONS)
        if not str(policy.get('deployment_contract_profile', '')).strip():
            errors.append('contract v4 requires deployment_contract_profile')
        if policy.get('deployment_requires_action_contract_v4_transform') is not True:
            errors.append('contract v4 requires deployment_requires_action_contract_v4_transform: true')
        for i in range(NUM_ACTIONS):
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

    shape_info: dict[str, Any] | None = None
    if not onnx_path.is_file():
        errors.append(f'paired ONNX file is missing: {onnx_path}')
    else:
        expected_sha = str(policy.get('policy_sha256', '')).lower()
        actual_sha = sha256_file(onnx_path)
        if not expected_sha:
            errors.append('policy_sha256 is missing')
        elif actual_sha != expected_sha:
            errors.append(f'ONNX SHA-256 mismatch: YAML={expected_sha}, file={actual_sha}')

        if skip_shape_check:
            warnings.append('ONNX tensor-shape inspection was explicitly skipped')
        elif shape_probe is None:
            errors.append(
                'ONNX tensor-shape probe is unavailable; run the installed audit or pass '
                '--onnx-shape-probe'
            )
        elif not shape_probe.is_file() or not os.access(shape_probe, os.X_OK):
            errors.append(f'ONNX tensor-shape probe is not executable: {shape_probe}')
        else:
            shape_info = probe_onnx_contract(onnx_path, shape_probe)
            if observations in (LEGACY_OBSERVATIONS, PHASE_OBSERVATIONS):
                validate_onnx_shape(shape_info, observations, errors)

    return errors, warnings, shape_info


def main() -> int:
    default_policy, default_joint_map = default_paths()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--policy-yaml', type=Path, default=default_policy)
    parser.add_argument('--joint-map', type=Path, default=default_joint_map)
    parser.add_argument('--onnx', type=Path, default=None, help='Optional explicit ONNX path.')
    parser.add_argument(
        '--onnx-shape-probe',
        type=Path,
        default=None,
        help='Explicit policy_onnx_contract_probe executable. Installed audits find it automatically.',
    )
    parser.add_argument(
        '--skip-onnx-shape-check',
        action='store_true',
        help='Source-development escape hatch only; never use for deployment acceptance.',
    )
    args = parser.parse_args()

    try:
        probe = resolve_probe(args.onnx_shape_probe.expanduser().resolve() if args.onnx_shape_probe else None)
        errors, warnings, shape_info = audit(
            args.policy_yaml.expanduser().resolve(),
            args.joint_map.expanduser().resolve(),
            args.onnx.expanduser().resolve() if args.onnx else None,
            probe,
            args.skip_onnx_shape_check,
        )
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
        for item in warnings:
            print(f'WARN  {item}')
        return TEST_FAIL

    policy = yaml.safe_load(args.policy_yaml.read_text(encoding='utf-8'))
    print('POLICY BUNDLE AUDIT: PASS')
    print(
        f"observation_contract: v{policy.get('observation_contract_version', 1)} "
        f"{policy.get('observation_contract_name', 'legacy_45_compatibility')}"
    )
    print(f"interface: obs[{policy.get('num_observations')}] -> actions[{policy.get('num_actions')}]")
    print(f"action_contract: v{policy['action_contract_version']} {policy.get('deployment_contract_profile', '')}")
    print(f"task: {policy.get('metadata', {}).get('task', 'unknown')}")
    print(f"policy_dt: {policy.get('policy_dt')} s")
    print(f"policy_sha256: {policy.get('policy_sha256')}")
    if shape_info is not None:
        print(f"onnx_input_shape: {shape_info.get('input_shape')}")
        print(f"onnx_output_shape: {shape_info.get('output_shape')}")
    for item in warnings:
        print(f'WARN  {item}')
    return PASS


if __name__ == '__main__':
    raise SystemExit(main())
