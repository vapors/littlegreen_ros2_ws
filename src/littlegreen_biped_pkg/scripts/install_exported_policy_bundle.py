#!/usr/bin/env python3
"""Audit and atomically install a Track-1 deployment bundle into Track 2.

The source bundle must contain policy.onnx, policy.yaml, deployment_contract.yaml,
policy.sha256, and bundle_manifest.yaml.  No servo or policy-live state is changed;
this tool only updates the packaged policy artifacts after a successful contract audit.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import math
import os
import shutil
import sys
import tempfile
from pathlib import Path

import yaml

PASS = 0
AUDIT_FAIL = 2
CONFIG_ERROR = 5


def load_audit_module(script_dir: Path):
    path = script_dir / 'policy_bundle_audit.py'
    spec = importlib.util.spec_from_file_location('lgh_policy_bundle_audit', path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f'unable to load audit module: {path}')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def source_defaults(script_path: Path) -> tuple[Path, Path]:
    package = script_path.resolve().parents[1]
    configs = package / 'src' / 'configs'
    checkpoints = package / 'src' / 'checkpoints'
    return configs, checkpoints


def atomic_copy(source: Path, destination: Path) -> None:
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        prefix=destination.name + '.', suffix='.tmp', dir=destination.parent, delete=False
    ) as stream:
        temp = Path(stream.name)
    try:
        shutil.copy2(source, temp)
        os.replace(temp, destination)
    finally:
        temp.unlink(missing_ok=True)


def _task_name(policy: dict) -> str:
    metadata = policy.get('metadata')
    if isinstance(metadata, dict) and metadata.get('task'):
        return str(metadata['task'])
    return str(policy.get('task', ''))


def _task_role(policy: dict) -> str:
    metadata = policy.get('metadata')
    if isinstance(metadata, dict) and metadata.get('task_role'):
        return str(metadata['task_role'])
    return str(policy.get('task_role', ''))


def normalize_handoff_profile(handoff_json: Path, policy_yaml: Path, joint_map_yaml: Path) -> dict:
    source = json.loads(handoff_json.read_text(encoding='utf-8'))
    policy = yaml.safe_load(policy_yaml.read_text(encoding='utf-8'))
    joint_map = yaml.safe_load(joint_map_yaml.read_text(encoding='utf-8'))
    if not isinstance(source, dict) or not isinstance(policy, dict) or not isinstance(joint_map, dict):
        raise ValueError('handoff JSON, policy YAML, and joint map must contain mappings')
    if int(source.get('schema_version', -1)) != 2:
        raise ValueError('zero-command handoff JSON must use analyzer schema_version 2')
    if float(source.get('max_abs_previous_action_obs_vs_action_term', math.inf)) > 1.0e-6:
        raise ValueError('handoff previous-action observation cross-check is not zero')
    task = _task_name(policy)
    if source.get('task') != task:
        raise ValueError(f'handoff task {source.get("task")!r} does not match policy task {task!r}')
    entries = sorted(joint_map.get('joints', []), key=lambda item: int(item['policy_action_index']))
    if len(entries) != 12:
        raise ValueError('joint map must contain 12 policy joints')
    names = [str(item['name']) for item in entries]
    if source.get('joint_order') != names:
        raise ValueError('handoff joint_order does not match canonical Track-2 joint order')
    pose = source.get('median_joint_position_rad')
    previous = source.get('median_previous_bounded_action')
    q_default = source.get('q_default_rad')
    if not isinstance(pose, list) or len(pose) != 12:
        raise ValueError('handoff median_joint_position_rad must contain 12 values')
    if not isinstance(previous, list) or len(previous) != 12:
        raise ValueError('handoff median_previous_bounded_action must contain 12 values')
    if not isinstance(q_default, list) or len(q_default) != 12:
        raise ValueError('handoff q_default_rad must contain 12 values')
    policy_default = policy.get('action_default_rad')
    if not isinstance(policy_default, list) or len(policy_default) != 12:
        raise ValueError('policy action_default_rad must contain 12 values')
    for i, (a, b) in enumerate(zip(q_default, policy_default)):
        if not math.isfinite(float(a)) or abs(float(a) - float(b)) > 1.0e-5:
            raise ValueError(f'handoff q_default mismatch at action[{i}]')
    effective_pose = []
    clamps = []
    for i, (value, entry) in enumerate(zip(pose, entries)):
        q = float(value)
        lo = float(entry['limit_lower_rad'])
        hi = float(entry['limit_upper_rad'])
        if not math.isfinite(q):
            raise ValueError(f'non-finite handoff pose at action[{i}]')
        qc = min(max(q, lo), hi)
        if abs(qc - q) > 0.002:
            raise ValueError(
                f'handoff pose exceeds physical limit by more than 0.002 rad at action[{i}]'
            )
        effective_pose.append(qc)
        if qc != q:
            clamps.append({
                'index': i,
                'joint': names[i],
                'source_rad': q,
                'effective_rad': qc,
                'delta_rad': qc - q,
            })
    previous_values = [float(x) for x in previous]
    if any(not math.isfinite(x) or x < -1.0 or x > 1.0 for x in previous_values):
        raise ValueError('handoff previous-action seed must be finite and within [-1,1]')
    command = [float(x) for x in source.get('command', [])]
    if command != [0.0, 0.0, 0.0]:
        raise ValueError('learned zero-command handoff command must be [0,0,0]')
    phase = [0.0, 1.0]
    import hashlib
    profile = {
        'schema_version': 1,
        'mode': 'learned_zero_command',
        'task': task,
        'policy_sha256': str(policy.get('policy_sha256', '')),
        'source': {
            'analyzer_schema_version': int(source['schema_version']),
            'checkpoint': str(source.get('checkpoint', '')),
            'handoff_json_sha256': hashlib.sha256(handoff_json.read_bytes()).hexdigest(),
            'selected_samples': int(source.get('selection', {}).get('selected_samples', 0)),
            'selection': source.get('selection', {}),
        },
        'joint_order': names,
        'joint_position_rad': effective_pose,
        'source_median_joint_position_rad': [float(x) for x in pose],
        'physical_limit_clamps': clamps,
        'previous_action_bounded': previous_values,
        'command': command,
        'phase': phase,
        'source_median_projected_gravity_b': source.get('median_projected_gravity_b', []),
        'previous_action_observation_indices': source.get('previous_action_observation_indices'),
        'previous_action_observation_semantics': source.get('previous_action_observation_semantics'),
        'max_abs_previous_action_obs_vs_action_term': float(
            source.get('max_abs_previous_action_obs_vs_action_term', math.inf)
        ),
        'notes': [
            'q_default remains the protected observation/action reference and is not replaced by this pose.',
            'This profile is the default pre-position/start state for live locomotion policy handoff, not an automatic power-on motion.',
            'The driver must hold pose override during the ramp; policy authority remains disabled until explicitly armed and enabled.',
        ],
    }
    return profile


def main() -> int:
    script_path = Path(__file__)
    default_configs, default_checkpoints = source_defaults(script_path)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bundle-dir', type=Path, required=True)
    parser.add_argument('--destination-config-dir', type=Path, default=default_configs)
    parser.add_argument('--checkpoint-dir', type=Path, default=default_checkpoints)
    parser.add_argument('--joint-map', type=Path, default=None)
    parser.add_argument('--onnx-shape-probe', type=Path, default=None)
    parser.add_argument('--skip-onnx-shape-check', action='store_true')
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument(
        '--handoff-json', type=Path, default=None,
        help='Optional Track-1 zero_command_handoff_pose.json. If omitted, auto-detect it in bundle-dir.',
    )
    args = parser.parse_args()

    try:
        bundle = args.bundle_dir.expanduser().resolve()
        configs = args.destination_config_dir.expanduser().resolve()
        checkpoints = args.checkpoint_dir.expanduser().resolve()
        required = {
            'policy.onnx': bundle / 'policy.onnx',
            'policy.yaml': bundle / 'policy.yaml',
            'deployment_contract.yaml': bundle / 'deployment_contract.yaml',
            'policy.sha256': bundle / 'policy.sha256',
            'bundle_manifest.yaml': bundle / 'bundle_manifest.yaml',
        }
        missing = [name for name, path in required.items() if not path.is_file()]
        if missing:
            raise ValueError(f'bundle is missing required files: {missing}')

        joint_map = (
            args.joint_map.expanduser().resolve()
            if args.joint_map else configs / 'joint_map.yaml'
        )
        if not joint_map.is_file():
            raise ValueError(f'joint map is missing: {joint_map}')

        handoff_source = args.handoff_json.expanduser().resolve() if args.handoff_json else None
        if handoff_source is None:
            candidate = bundle / 'zero_command_handoff_pose.json'
            if candidate.is_file():
                handoff_source = candidate
        handoff_profile = None
        if handoff_source is not None:
            if not handoff_source.is_file():
                raise ValueError(f'handoff JSON is missing: {handoff_source}')
            handoff_profile = normalize_handoff_profile(
                handoff_source, required['policy.yaml'], joint_map
            )

        audit = load_audit_module(script_path.resolve().parent)
        probe = audit.resolve_probe(
            args.onnx_shape_probe.expanduser().resolve() if args.onnx_shape_probe else None
        )
        errors, warnings, _ = audit.audit(
            required['policy.yaml'],
            joint_map,
            required['policy.onnx'],
            probe,
            args.skip_onnx_shape_check,
            required['deployment_contract.yaml'],
            required['policy.sha256'],
            required['bundle_manifest.yaml'],
        )
        for warning in warnings:
            print(f'WARN  {warning}')
        if errors:
            print('POLICY BUNDLE INSTALL: AUDIT FAIL')
            for error in errors:
                print(f'FAIL  {error}')
            return AUDIT_FAIL

        destinations = {
            configs / 'policy.onnx': required['policy.onnx'],
            configs / 'policy_latest.yaml': required['policy.yaml'],
            configs / 'policy.yaml': required['policy.yaml'],
            configs / 'deployment_contract.yaml': required['deployment_contract.yaml'],
            configs / 'policy.sha256': required['policy.sha256'],
            configs / 'bundle_manifest.yaml': required['bundle_manifest.yaml'],
            checkpoints / 'policy.onnx': required['policy.onnx'],
        }
        print('POLICY BUNDLE INSTALL: AUDIT PASS')
        for destination, source in destinations.items():
            print(f'{source.name} -> {destination}')
        existing_handoff = configs / 'policy_handoff.yaml'
        if handoff_profile is not None:
            print(
                'zero_command_handoff_pose.json -> '
                f'{existing_handoff} (SHA-bound learned zero-command profile)'
            )
        elif _task_role(yaml.safe_load(required['policy.yaml'].read_text(encoding='utf-8'))) == 'locomotion':
            print('WARN  no zero-command handoff JSON supplied; live v2.9.3 locomotion authority will require a matching existing profile')
        if args.dry_run:
            print('DRY RUN: no files changed')
            return PASS
        for destination, source in destinations.items():
            atomic_copy(source, destination)
        if handoff_profile is not None:
            existing_handoff.parent.mkdir(parents=True, exist_ok=True)
            with tempfile.NamedTemporaryFile(
                mode='w', encoding='utf-8', prefix='policy_handoff.', suffix='.yaml',
                dir=existing_handoff.parent, delete=False
            ) as stream:
                temp_handoff = Path(stream.name)
                yaml.safe_dump(handoff_profile, stream, sort_keys=False)
            try:
                os.replace(temp_handoff, existing_handoff)
            finally:
                temp_handoff.unlink(missing_ok=True)
        print('POLICY BUNDLE INSTALL: COMPLETE')
        print('Rebuild/source the workspace before running shadow or live handoff mode.')
        return PASS
    except (OSError, ValueError, RuntimeError) as exc:
        print(f'POLICY BUNDLE INSTALL: CONFIG ERROR\n{exc}', file=sys.stderr)
        return CONFIG_ERROR


if __name__ == '__main__':
    raise SystemExit(main())
