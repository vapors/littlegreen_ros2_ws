#!/usr/bin/env python3
"""Add the canonical v1.4.7 47-D observation metadata to a genuine exported policy YAML.

This tool never changes num_observations, action fields, ONNX bytes, or policy_sha256.
It refuses 45-D bundles and unexpected Track 1 tasks.
"""
from __future__ import annotations

import argparse
import math
import re
import sys
from pathlib import Path
from typing import Any

import yaml

PASS = 0
REFUSED = 3
CONFIG_ERROR = 5
EXPECTED_TASK = 'Velocity-Lilgreen-Hardware-ST3215-Loaded-v7'
PHASE_FIELDS: dict[str, Any] = {
    'observation_contract_version': 2,
    'observation_contract_name': 'littlegreen_hardware_phase_guided_47_v1',
    'observation_layout': [
        'command_velocity_3',
        'base_angular_velocity_3',
        'projected_gravity_3',
        'joint_position_relative_to_default_12',
        'joint_velocity_12',
        'previous_bounded_normalized_action_12',
        'gait_phase_sin_cos_2',
    ],
    'gait_phase_enabled': True,
    'gait_phase_period_s': 0.72,
    'gait_phase_encoding': 'sin_cos_2pi',
    'gait_phase_append_order': 'after_previous_action',
    'gait_phase_training_timebase': 'episode_step_time',
    'gait_phase_training_reset_semantics': 'environment_episode_reset',
}


def annotate(policy: dict[str, Any]) -> dict[str, Any]:
    if int(policy.get('num_observations', -1)) != 47:
        raise ValueError('refusing bundle: num_observations must already be 47')
    if int(policy.get('num_actions', -1)) != 12:
        raise ValueError('refusing bundle: num_actions must be 12')
    if int(policy.get('action_contract_version', -1)) != 4:
        raise ValueError('refusing bundle: action_contract_version must be 4')
    policy_dt = float(policy.get('policy_dt', 0.0))
    if not math.isfinite(policy_dt) or abs(policy_dt - 0.02) > 1.0e-9:
        raise ValueError('refusing bundle: policy_dt must be finite and equal to 0.02')
    metadata = policy.get('metadata')
    if not isinstance(metadata, dict):
        raise ValueError('refusing bundle: metadata must be a mapping')
    task = str(metadata.get('task', ''))
    if task != EXPECTED_TASK:
        raise ValueError(f'refusing bundle: task is {task!r}, expected {EXPECTED_TASK!r}')
    checksum = str(policy.get('policy_sha256', '')).strip().lower()
    if not re.fullmatch(r'[0-9a-f]{64}', checksum):
        raise ValueError('refusing bundle: policy_sha256 must be 64 lowercase/uppercase hex characters')

    annotated = dict(policy)
    annotated.update(PHASE_FIELDS)
    return annotated


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--policy-yaml', type=Path, required=True)
    parser.add_argument(
        '--output',
        type=Path,
        default=None,
        help='Output YAML. Default: POLICY_STEM.phase_guided.yaml beside the input.',
    )
    args = parser.parse_args()

    source = args.policy_yaml.expanduser().resolve()
    output = args.output.expanduser().resolve() if args.output else source.with_name(
        source.stem + '.phase_guided.yaml'
    )
    try:
        policy = yaml.safe_load(source.read_text(encoding='utf-8'))
        if not isinstance(policy, dict):
            raise ValueError('policy YAML must contain a mapping')
        annotated = annotate(policy)
        if output == source:
            raise ValueError('refusing in-place edit; choose a distinct --output path')
        output.write_text(yaml.safe_dump(annotated, sort_keys=False), encoding='utf-8')
    except (OSError, TypeError, ValueError, yaml.YAMLError) as exc:
        print(f'PHASE POLICY ANNOTATION: REFUSED\n{exc}', file=sys.stderr)
        return REFUSED if isinstance(exc, ValueError) else CONFIG_ERROR

    print('PHASE POLICY ANNOTATION: COMPLETE')
    print(f'input:  {source}')
    print(f'output: {output}')
    print('Next: run policy_bundle_audit against the output YAML and paired ONNX.')
    return PASS


if __name__ == '__main__':
    raise SystemExit(main())
