#!/usr/bin/env python3
"""
Compare Track 1 golden vectors with the Track 2 47-D/action-v4 contract.

The fixture is transport-neutral YAML.  It can contain Track 1 observations, raw
actor outputs, and q-targets.  ONNX inference is optional and is used when the
onnxruntime Python package is available on the validation host.
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path
from typing import Any

import yaml

PASS = 0
MISMATCH = 2
CONFIG_ERROR = 5


def sequence(mapping: dict[str, Any], key: str, count: int) -> list[float]:
    value = mapping.get(key)
    if not isinstance(value, list) or len(value) != count:
        raise ValueError(f'{key} must contain {count} values')
    result = [float(item) for item in value]
    if not all(math.isfinite(item) for item in result):
        raise ValueError(f'{key} contains non-finite values')
    return result


def build_observation(vector: dict[str, Any], policy: dict[str, Any]) -> list[float]:
    if str(policy.get('phase_mode', '')) == 'neutral_static':
        phase_sin, phase_cos = 0.0, 1.0
    else:
        phase = float(vector['phase_fraction'])
        if not 0.0 <= phase < 1.0:
            raise ValueError('phase_fraction must be in [0,1)')
        angle = 2.0 * math.pi * phase
        phase_sin, phase_cos = math.sin(angle), math.cos(angle)
    observation = (
        sequence(vector, 'command_velocity', 3)
        + sequence(vector, 'base_angular_velocity', 3)
        + sequence(vector, 'projected_gravity', 3)
        + sequence(vector, 'joint_position_relative_default', 12)
        + sequence(vector, 'joint_velocity', 12)
        + sequence(vector, 'previous_bounded_action', 12)
        + [phase_sin, phase_cos]
    )
    if len(observation) != 47:
        raise AssertionError('internal observation length error')
    return observation


def bounded_action_and_target(
    raw_action: list[float], policy: dict[str, Any]
) -> tuple[list[float], list[float]]:
    defaults = sequence(policy, 'action_default_rad', 12)
    scales = sequence(policy, 'action_residual_scale_rad', 12)
    lower = sequence(policy, 'action_target_lower_rad', 12)
    upper = sequence(policy, 'action_target_upper_rad', 12)
    bounded = [max(-1.0, min(1.0, value)) for value in raw_action]
    target = [
        max(lower[i], min(upper[i], defaults[i] + bounded[i] * scales[i]))
        for i in range(12)
    ]
    return bounded, target


def max_abs_difference(a: list[float], b: list[float]) -> float:
    if len(a) != len(b):
        return math.inf
    return max((abs(x - y) for x, y in zip(a, b)), default=0.0)


def run_onnx(path: Path, observation: list[float]) -> list[float]:
    try:
        import numpy as np
        import onnxruntime as ort
    except ImportError as exc:
        raise ValueError(
            'onnxruntime Python package is unavailable; use --skip-onnx or run '
            'on the Orange Pi validation host'
        ) from exc
    session = ort.InferenceSession(str(path), providers=['CPUExecutionProvider'])
    inputs = session.get_inputs()
    outputs = session.get_outputs()
    if len(inputs) != 1 or len(outputs) != 1:
        raise ValueError('ONNX model must expose one input and one output')
    raw = session.run(
        [outputs[0].name],
        {inputs[0].name: np.asarray([observation], dtype=np.float32)},
    )[0]
    return [float(value) for value in raw.reshape(-1)]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fixture', type=Path, required=True)
    parser.add_argument('--policy-yaml', type=Path, required=True)
    parser.add_argument('--onnx', type=Path, default=None)
    parser.add_argument('--skip-onnx', action='store_true')
    parser.add_argument('--tolerance', type=float, default=1.0e-5)
    args = parser.parse_args()

    try:
        policy = yaml.safe_load(args.policy_yaml.read_text(encoding='utf-8'))
        fixture = yaml.safe_load(args.fixture.read_text(encoding='utf-8'))
        if not isinstance(policy, dict) or not isinstance(fixture, dict):
            raise ValueError('policy and fixture must contain mappings')
        vectors = fixture.get('vectors')
        if not isinstance(vectors, list) or not vectors:
            raise ValueError('fixture must contain a non-empty vectors sequence')
        failures: list[str] = []
        for index, vector in enumerate(vectors):
            if not isinstance(vector, dict):
                raise ValueError(f'vector[{index}] must be a mapping')
            name = str(vector.get('name', f'vector_{index}'))
            observation = build_observation(vector, policy)
            expected_observation = vector.get('track1_observation')
            if expected_observation is not None:
                expected = [float(value) for value in expected_observation]
                difference = max_abs_difference(observation, expected)
                print(
                    f'{name}: Track1 observation vs Track2 builder '
                    f'max_abs_diff={difference:.9g}'
                )
                if difference > args.tolerance:
                    failures.append(f'{name} observation mismatch {difference:.9g}')

            raw_action: list[float] | None = None
            if args.onnx is not None and not args.skip_onnx:
                raw_action = run_onnx(args.onnx, observation)
                expected_raw = vector.get('track1_raw_action')
                if expected_raw is not None:
                    difference = max_abs_difference(
                        raw_action, [float(value) for value in expected_raw]
                    )
                    print(f'{name}: Track1 raw action vs ONNX max_abs_diff={difference:.9g}')
                    if difference > args.tolerance:
                        failures.append(f'{name} raw-action mismatch {difference:.9g}')
            elif vector.get('track1_raw_action') is not None:
                raw_action = [float(value) for value in vector['track1_raw_action']]

            if raw_action is not None:
                bounded, target = bounded_action_and_target(raw_action, policy)
                expected_bounded = vector.get('track1_bounded_action')
                expected_target = vector.get('track1_q_target')
                if expected_bounded is not None:
                    difference = max_abs_difference(
                        bounded, [float(value) for value in expected_bounded]
                    )
                    print(f'{name}: bounded-action max_abs_diff={difference:.9g}')
                    if difference > args.tolerance:
                        failures.append(f'{name} bounded-action mismatch {difference:.9g}')
                if expected_target is not None:
                    difference = max_abs_difference(
                        target, [float(value) for value in expected_target]
                    )
                    print(f'{name}: q-target max_abs_diff={difference:.9g}')
                    if difference > args.tolerance:
                        failures.append(f'{name} q-target mismatch {difference:.9g}')
        if failures:
            print('GOLDEN VECTOR COMPARISON: FAIL')
            for failure in failures:
                print(f'FAIL  {failure}')
            return MISMATCH
        print('GOLDEN VECTOR COMPARISON: PASS')
        return PASS
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as exc:
        print(f'GOLDEN VECTOR COMPARISON: CONFIG ERROR\n{exc}', file=sys.stderr)
        return CONFIG_ERROR


if __name__ == '__main__':
    raise SystemExit(main())
