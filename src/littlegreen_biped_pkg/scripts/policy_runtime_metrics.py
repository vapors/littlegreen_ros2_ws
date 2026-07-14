#!/usr/bin/env python3
"""Record Track-1-aligned policy metrics from LittleGreen shadow or live runtime."""
from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, UInt8MultiArray
import yaml

PASS = 0
REFUSED_PRECONDITION = 3
TIMEOUT_OR_UNAVAILABLE = 4
CONFIG_ERROR = 5
OPERATOR_ABORT = 7
INTERNAL_ERROR = 70


@dataclass
class TimedValue:
    value: Any = None
    monotonic_s: float = 0.0


class RuntimeMetricsNode(Node):
    def __init__(self, policy_yaml: Path, joint_map: Path, freshness_sec: float,
                 standing_threshold: float, velocity_limit_rad_s: float) -> None:
        super().__init__('policy_runtime_metrics')
        self.freshness_sec = freshness_sec
        self.standing_threshold = standing_threshold
        self.velocity_limit_rad_s = velocity_limit_rad_s
        self.policy = yaml.safe_load(policy_yaml.read_text(encoding='utf-8'))
        if not isinstance(self.policy, dict):
            raise ValueError('policy YAML must contain a mapping')
        self.num_observations = int(self.policy.get('num_observations', -1))
        if self.num_observations not in (45, 47):
            raise ValueError(
                f'unsupported num_observations={self.num_observations}; expected 45 or 47'
            )
        self.phase_enabled = self.num_observations == 47
        if self.phase_enabled:
            required_phase = {
                'observation_contract_version': 2,
                'observation_contract_name': 'littlegreen_hardware_phase_guided_47_v1',
                'gait_phase_enabled': True,
                'gait_phase_period_s': 0.72,
                'gait_phase_encoding': 'sin_cos_2pi',
                'gait_phase_append_order': 'after_previous_action',
                'gait_phase_training_timebase': 'episode_step_time',
                'gait_phase_training_reset_semantics': 'environment_episode_reset',
            }
            for key, expected in required_phase.items():
                actual = self.policy.get(key)
                if isinstance(expected, float):
                    valid = isinstance(actual, (int, float)) and abs(float(actual) - expected) <= 1.0e-9
                else:
                    valid = actual == expected
                if not valid:
                    raise ValueError(f'{key}={actual!r}, expected {expected!r}')
            expected_layout = [
                'command_velocity_3',
                'base_angular_velocity_3',
                'projected_gravity_3',
                'joint_position_relative_to_default_12',
                'joint_velocity_12',
                'previous_bounded_normalized_action_12',
                'gait_phase_sin_cos_2',
            ]
            if self.policy.get('observation_layout') != expected_layout:
                raise ValueError('47-D observation_layout does not match the supported contract')
            if int(self.policy.get('action_contract_version', -1)) != 4:
                raise ValueError('47-D policy requires action_contract_version: 4')
            policy_dt = float(self.policy.get('policy_dt', 0.0))
            if not math.isfinite(policy_dt) or abs(policy_dt - 0.02) > 1.0e-9:
                raise ValueError('47-D policy requires finite policy_dt: 0.02')
        mapping = yaml.safe_load(joint_map.read_text(encoding='utf-8'))
        if not isinstance(mapping, dict) or not isinstance(mapping.get('joints'), list):
            raise ValueError('joint map must contain a joints sequence')
        entries = sorted(mapping['joints'], key=lambda item: int(item['policy_action_index']))
        if len(entries) != 12:
            raise ValueError('joint map must contain exactly 12 policy joints')
        self.joint_names = [str(item['name']) for item in entries]
        self.defaults = [float(item['default_joint_rad']) for item in entries]

        self.observation = TimedValue()
        self.raw_action = TimedValue()
        self.clipped_action = TimedValue()
        self.target_unclipped = TimedValue()
        self.target = TimedValue()
        self.mask = TimedValue()
        self.joint_state = TimedValue()
        self.command = TimedValue()
        self.gait_phase = TimedValue()
        self.rows: list[dict[str, float | int | bool]] = []

        self.create_subscription(Float64MultiArray, '/policy_debug/observation', self._obs, qos_profile_sensor_data)
        self.create_subscription(Float64MultiArray, '/policy_debug/raw_action', self._raw, qos_profile_sensor_data)
        self.create_subscription(Float64MultiArray, '/policy_debug/clipped_raw_action', self._clipped, qos_profile_sensor_data)
        self.create_subscription(Float64MultiArray, '/policy_debug/target_unclipped', self._target_unclipped, qos_profile_sensor_data)
        self.create_subscription(Float64MultiArray, '/policy_debug/target_clipped', self._target, qos_profile_sensor_data)
        self.create_subscription(UInt8MultiArray, '/policy_debug/saturation_mask', self._mask, qos_profile_sensor_data)
        if self.phase_enabled:
            self.create_subscription(
                Float64MultiArray,
                '/policy_debug/gait_phase',
                self._gait_phase,
                qos_profile_sensor_data,
            )
        self.create_subscription(JointState, '/joint_states', self._joint, qos_profile_sensor_data)
        self.create_subscription(Twist, '/command_velocity', self._command, 10)

    @staticmethod
    def _set(slot: TimedValue, value: Any) -> None:
        slot.value = value
        slot.monotonic_s = time.monotonic()

    def _obs(self, msg: Float64MultiArray) -> None:
        self._set(self.observation, list(map(float, msg.data)))

    def _raw(self, msg: Float64MultiArray) -> None:
        self._set(self.raw_action, list(map(float, msg.data)))

    def _clipped(self, msg: Float64MultiArray) -> None:
        self._set(self.clipped_action, list(map(float, msg.data)))

    def _mask(self, msg: UInt8MultiArray) -> None:
        self._set(self.mask, list(map(int, msg.data)))
        self._record_if_ready()

    def _gait_phase(self, msg: Float64MultiArray) -> None:
        self._set(self.gait_phase, list(map(float, msg.data)))

    def _joint(self, msg: JointState) -> None:
        by_name = {name: i for i, name in enumerate(msg.name)}
        if all(name in by_name for name in self.joint_names):
            position = [float(msg.position[by_name[name]]) for name in self.joint_names]
            velocity = [
                float(msg.velocity[by_name[name]]) if by_name[name] < len(msg.velocity) else float('nan')
                for name in self.joint_names
            ]
        elif len(msg.position) >= 12:
            position = list(map(float, msg.position[:12]))
            velocity = list(map(float, msg.velocity[:12])) if len(msg.velocity) >= 12 else [float('nan')] * 12
        else:
            return
        self._set(self.joint_state, (position, velocity))

    def _command(self, msg: Twist) -> None:
        self._set(self.command, [float(msg.linear.x), float(msg.linear.y), float(msg.angular.z)])

    def _fresh(self, *slots: TimedValue) -> bool:
        now = time.monotonic()
        return all(slot.value is not None and now - slot.monotonic_s <= self.freshness_sec for slot in slots)

    def _target_unclipped(self, msg: Float64MultiArray) -> None:
        self._set(self.target_unclipped, list(map(float, msg.data)))

    def _target(self, msg: Float64MultiArray) -> None:
        self._set(self.target, list(map(float, msg.data)))

    def _record_if_ready(self) -> None:
        required = [
            self.observation, self.raw_action, self.clipped_action,
            self.target_unclipped, self.target, self.mask,
        ]
        if self.phase_enabled:
            required.append(self.gait_phase)
        if not self._fresh(*required):
            return
        obs = self.observation.value
        raw = self.raw_action.value
        clipped = self.clipped_action.value
        target_unclipped = self.target_unclipped.value
        target = self.target.value
        mask = self.mask.value
        if not (
            len(obs) == self.num_observations and len(raw) == len(clipped) == len(target_unclipped)
            == len(target) == len(mask) == 12
        ):
            return

        cmd = obs[0:3]
        command_norm = math.sqrt(sum(value * value for value in cmd))
        standing = command_norm < self.standing_threshold
        residual = [abs(target[i] - self.defaults[i]) for i in range(12)]
        q_relative = list(map(float, obs[9:21]))
        q_rms = math.sqrt(statistics.fmean(value * value for value in q_relative))
        q_max = max(abs(value) for value in q_relative)
        base_ang_vel = list(map(float, obs[3:6]))
        row: dict[str, float | int | bool] = {
            'monotonic_s': time.monotonic(),
            'command_vx_mps': cmd[0],
            'command_vy_mps': cmd[1],
            'command_yaw_rad_s': cmd[2],
            'command_norm': command_norm,
            'standing_command': standing,
            'projected_gravity_x': obs[6],
            'projected_gravity_y': obs[7],
            'projected_gravity_z': obs[8],
            'base_angular_velocity_norm_rad_s': math.sqrt(sum(value * value for value in base_ang_vel)),
            'standing_upright_observable': bool(obs[8] < -0.97),
            'standing_quiet_yaw_observable': bool(abs(obs[5]) < 0.20),
            'standing_near_default_observable': bool(q_max < 0.20),
            'joint_posture_rms_rad': q_rms,
            'joint_posture_max_rad': q_max,
            'raw_action_mean_abs': statistics.fmean(abs(value) for value in raw),
            'raw_action_std': statistics.pstdev(raw),
            'raw_action_min': min(raw),
            'raw_action_max': max(raw),
            'raw_action_excess_fraction': sum(abs(value) > 1.0 for value in raw) / 12.0,
            'bounded_saturation_fraction': sum(abs(value) >= 0.999 for value in clipped) / 12.0,
            'raw_action_clip_fraction': sum(bool(value & 0x01) for value in mask) / 12.0,
            'target_limit_fraction': sum(bool(value & 0x06) for value in mask) / 12.0,
            'target_clip_abs_mean_rad': statistics.fmean(
                abs(target_unclipped[i] - target[i]) for i in range(12)
            ),
            'target_clip_abs_max_rad': max(
                abs(target_unclipped[i] - target[i]) for i in range(12)
            ),
            'target_residual_abs_mean_rad': statistics.fmean(residual),
            'target_residual_abs_max_rad': max(residual),
        }
        if self.phase_enabled:
            phase_sine = float(obs[45])
            phase_cosine = float(obs[46])
            phase_angle = math.atan2(phase_sine, phase_cosine)
            phase_fraction = (phase_angle / (2.0 * math.pi)) % 1.0
            phase_debug = self.gait_phase.value
            if len(phase_debug) != 6:
                return
            debug_fraction = float(phase_debug[0])
            debug_tick = int(round(float(phase_debug[1])))
            debug_period_ticks = int(round(float(phase_debug[2])))
            debug_sine = float(phase_debug[3])
            debug_cosine = float(phase_debug[4])
            debug_half_cycle = int(round(float(phase_debug[5])))
            row.update({
                'gait_phase_sine': phase_sine,
                'gait_phase_cosine': phase_cosine,
                'gait_phase_fraction': phase_fraction,
                'gait_phase_tick': debug_tick,
                'gait_phase_period_ticks': debug_period_ticks,
                'gait_phase_debug_fraction': debug_fraction,
                'gait_phase_debug_half_cycle': debug_half_cycle,
                'gait_phase_observation_debug_abs_error': max(
                    abs(phase_sine - debug_sine),
                    abs(phase_cosine - debug_cosine),
                ),
                'gait_phase_fraction_debug_abs_error': abs(phase_fraction - debug_fraction),
                'gait_phase_unit_circle_error': abs(
                    math.sqrt(phase_sine * phase_sine + phase_cosine * phase_cosine) - 1.0
                ),
                'gait_phase_expected_half_cycle': 0 if phase_fraction < 0.5 else 1,
                'gait_phase_expected_left_stance': bool(phase_fraction < 0.5),
                'gait_phase_expected_right_stance': bool(phase_fraction >= 0.5),
            })
        if self._fresh(self.joint_state):
            position, velocity = self.joint_state.value
            if len(position) == 12:
                errors = [abs(target[i] - position[i]) for i in range(12)]
                row['joint_tracking_error_abs_mean_rad'] = statistics.fmean(errors)
                row['joint_tracking_error_abs_max_rad'] = max(errors)
            finite_velocity = [abs(value) for value in velocity if math.isfinite(value)]
            if finite_velocity:
                row['joint_velocity_limit_fraction'] = (
                    sum(value >= 0.95 * self.velocity_limit_rad_s for value in finite_velocity)
                    / len(finite_velocity)
                )
        self.rows.append(row)


def mean(rows: list[dict[str, Any]], key: str) -> float:
    values = [float(row[key]) for row in rows if key in row and math.isfinite(float(row[key]))]
    return statistics.fmean(values) if values else float('nan')


def percentile(rows: list[dict[str, Any]], key: str, fraction: float) -> float:
    values = sorted(float(row[key]) for row in rows if key in row and math.isfinite(float(row[key])))
    if not values:
        return float('nan')
    index = min(len(values) - 1, max(0, round((len(values) - 1) * fraction)))
    return values[index]


def write_results(output_dir: Path, node: RuntimeMetricsNode, elapsed: float) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    fieldnames = sorted({key for row in node.rows for key in row})
    with (output_dir / 'timeseries.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(node.rows)

    standing = [row for row in node.rows if bool(row.get('standing_command'))]
    metric_keys = [
        'raw_action_mean_abs', 'raw_action_std', 'raw_action_min', 'raw_action_max',
        'raw_action_excess_fraction', 'bounded_saturation_fraction', 'raw_action_clip_fraction',
        'target_limit_fraction', 'target_clip_abs_mean_rad', 'target_clip_abs_max_rad',
        'target_residual_abs_mean_rad', 'target_residual_abs_max_rad',
        'joint_tracking_error_abs_mean_rad', 'joint_tracking_error_abs_max_rad',
        'joint_velocity_limit_fraction', 'projected_gravity_x', 'projected_gravity_z',
        'base_angular_velocity_norm_rad_s', 'joint_posture_rms_rad', 'joint_posture_max_rad',
        'standing_upright_observable', 'standing_quiet_yaw_observable',
        'standing_near_default_observable',
        'gait_phase_sine', 'gait_phase_cosine', 'gait_phase_fraction',
        'gait_phase_tick', 'gait_phase_period_ticks', 'gait_phase_debug_fraction',
        'gait_phase_debug_half_cycle', 'gait_phase_observation_debug_abs_error',
        'gait_phase_fraction_debug_abs_error', 'gait_phase_unit_circle_error',
        'gait_phase_expected_half_cycle',
        'gait_phase_expected_left_stance', 'gait_phase_expected_right_stance',
    ]
    summary: dict[str, Any] = {
        'schema_version': 2,
        'generated_at_utc': datetime.now(timezone.utc).isoformat(),
        'source_task': node.policy.get('metadata', {}).get('task'),
        'num_observations': node.num_observations,
        'observation_contract_version': node.policy.get('observation_contract_version', 1),
        'observation_contract_name': node.policy.get(
            'observation_contract_name', 'legacy_45_compatibility'
        ),
        'gait_phase_enabled': node.phase_enabled,
        'gait_phase_period_s': node.policy.get('gait_phase_period_s') if node.phase_enabled else None,
        'action_contract_version': node.policy.get('action_contract_version'),
        'deployment_contract_profile': node.policy.get('deployment_contract_profile'),
        'action_residual_scale_rad': node.policy.get('action_residual_scale_rad'),
        'action_default_rad': node.policy.get('action_default_rad'),
        'training_actuator_model_name': node.policy.get('training_actuator_model_name'),
        'training_actuator_model_stage': node.policy.get('training_actuator_model_stage'),
        'training_actuator_response_delay_scale': node.policy.get('training_actuator_response_delay_scale'),
        'training_actuator_velocity_scale_range': node.policy.get('training_actuator_velocity_scale_range'),
        'training_loaded_velocity_scale_range': node.policy.get('training_loaded_velocity_scale_range'),
        'duration_sec': elapsed,
        'sample_count': len(node.rows),
        'sample_rate_hz': len(node.rows) / elapsed if elapsed > 0 else 0.0,
        'standing_command_sample_count': len(standing),
        'global_mean': {key: mean(node.rows, key) for key in metric_keys},
        'standing_command_mean': {key: mean(standing, key) for key in metric_keys},
        'standing_projected_gravity_x': {
            'p05': percentile(standing, 'projected_gravity_x', 0.05),
            'median': percentile(standing, 'projected_gravity_x', 0.50),
            'p95': percentile(standing, 'projected_gravity_x', 0.95),
        },
        'not_observable_from_current_runtime_topics': [
            'base COM height',
            'COM forward offset relative to the feet',
            'foot contact / single support / double support',
            'swing clearance and foot lift counts',
            'foot slip',
            'physical joint torque',
            'root linear velocity and zero-command XY drift',
            'stable-standing all-conditions result because foot contact and root linear velocity are unavailable',
            'actual foot contact timing; gait phase is an expected policy clock, not a contact sensor',
        ],
        'observable_standing_condition_notes': {
            'upright': 'projected_gravity_z < -0.97',
            'quiet_yaw': 'abs(base angular velocity z) < 0.20 rad/s',
            'near_default': 'max abs(q - q_default) < 0.20 rad',
            'not_available': ['quiet_xy', 'both_feet', 'foot_slip'],
        },
        'gait_phase_notes': {
            'available': node.phase_enabled,
            'meaning': 'expected phase supplied to the policy; not measured foot contact',
            'half_cycle_0': 'phase [0.0,0.5): expected left stance / right swing',
            'half_cycle_1': 'phase [0.5,1.0): expected right stance / left swing',
        },
    }
    (output_dir / 'summary.yaml').write_text(yaml.safe_dump(summary, sort_keys=False), encoding='utf-8')


def default_paths() -> tuple[Path, Path]:
    share = Path(get_package_share_directory('littlegreen_biped_pkg'))
    return share / 'configs/policy_latest.yaml', share / 'configs/joint_map.yaml'


def main() -> int:
    policy_default, map_default = default_paths()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration-sec', type=float, default=20.0)
    parser.add_argument('--policy-yaml', type=Path, default=policy_default)
    parser.add_argument('--joint-map', type=Path, default=map_default)
    parser.add_argument('--freshness-sec', type=float, default=0.20)
    parser.add_argument('--standing-command-threshold', type=float, default=0.05)
    parser.add_argument('--joint-velocity-limit-rad-s', type=float, default=4.72)
    parser.add_argument('--output-dir', type=Path, default=None)
    args, ros_args = parser.parse_known_args()

    if args.duration_sec <= 0 or args.freshness_sec <= 0 or args.joint_velocity_limit_rad_s <= 0:
        print('POLICY RUNTIME METRICS: REFUSED — duration, freshness, and velocity limit must be positive', file=sys.stderr)
        return REFUSED_PRECONDITION

    output_dir = args.output_dir
    if output_dir is None:
        stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
        output_dir = Path.home() / '.ros' / 'littlegreen_policy_metrics' / stamp
    output_dir = output_dir.expanduser().resolve()

    rclpy.init(args=ros_args)
    node: RuntimeMetricsNode | None = None
    start = time.monotonic()
    interrupted = False
    try:
        node = RuntimeMetricsNode(
            args.policy_yaml.expanduser().resolve(), args.joint_map.expanduser().resolve(),
            args.freshness_sec, args.standing_command_threshold, args.joint_velocity_limit_rad_s,
        )
        print(f'Collecting LittleGreen policy runtime metrics for {args.duration_sec:.1f} s...')
        while rclpy.ok() and time.monotonic() - start < args.duration_sec:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        interrupted = True
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
        print(f'POLICY RUNTIME METRICS: CONFIG ERROR\n{exc}', file=sys.stderr)
        return CONFIG_ERROR
    except Exception as exc:
        print(f'POLICY RUNTIME METRICS: INTERNAL ERROR\n{exc}', file=sys.stderr)
        return INTERNAL_ERROR
    finally:
        elapsed = max(1.0e-9, time.monotonic() - start)
        if node is not None and node.rows:
            write_results(output_dir, node, elapsed)
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()

    if node is None or not node.rows:
        print(
            'POLICY RUNTIME METRICS: UNAVAILABLE — no synchronized policy debug samples were received.\n'
            'Confirm publish_policy_debug:=true and that shadow/live policy inference is running.',
            file=sys.stderr,
        )
        return TIMEOUT_OR_UNAVAILABLE

    print('POLICY RUNTIME METRICS: COMPLETE' if not interrupted else 'POLICY RUNTIME METRICS: OPERATOR ABORT')
    print(f'samples: {len(node.rows)}')
    print(f'output: {output_dir}')
    return OPERATOR_ABORT if interrupted else PASS


if __name__ == '__main__':
    raise SystemExit(main())
