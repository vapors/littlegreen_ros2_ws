#!/usr/bin/env python3
"""Whole-body standing/crouch load-characterization runner for LGH Track 2.

Modes
-----
capture_pose
    Explicitly disables all ST3215 torque through the native driver, lets the
    operator manually place the robot at a measured base-COM height, captures
    the settled 12-joint pose from cycle telemetry, and stores it in a YAML pose
    library.

evaluate
    Loads named poses from the library, enables torque while holding the measured
    physical pose, releases the driver override only after a live reference stream
    is present, then runs slow guarded whole-body transitions and static holds.

The runner intentionally keeps the learned policy and outer PD controller out of
this experiment. Hardware speed/ACC stay fixed at the v2.4.2+ profile while the
runner controls only the 50 Hz position-reference trajectory.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import math
import os
import select
import statistics
import sys
import termios
import threading
import time
import tty
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import rclpy
import yaml
from lgh_st3215_driver.msg import ServoTelemetry
from lgh_st3215_tools.dataset_manifest import write_manifest
from lgh_st3215_tools.diagnostic_compat import diagnostic_level_to_int
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray, UInt32MultiArray
from std_srvs.srv import Trigger

try:
    from ament_index_python.packages import get_package_share_directory
except Exception:  # pragma: no cover
    get_package_share_directory = None

NUM_JOINTS = 12
DEFAULT_POSE_LIBRARY = Path.home() / '.ros' / 'lgh_standing_poses.yaml'
DEFAULT_CAPTURE_AUDIT_ROOT = Path.home() / '.ros' / 'lgh_standing_pose_capture_audits'


@dataclass(frozen=True)
class JointConfig:
    name: str
    index: int
    servo_id: int
    training_default_rad: float
    min_rad: float
    max_rad: float


class AbortRequested(RuntimeError):
    pass


class TerminalKeys:
    def __init__(self) -> None:
        self.old_termios = None
        self.enabled = False

    def __enter__(self):
        if sys.stdin.isatty():
            self.old_termios = termios.tcgetattr(sys.stdin.fileno())
            tty.setcbreak(sys.stdin.fileno())
            self.enabled = True
        return self

    def __exit__(self, exc_type, exc, tb):
        if self.old_termios is not None:
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, self.old_termios)
        self.old_termios = None
        self.enabled = False

    def read_key(self) -> Optional[str]:
        if not self.enabled:
            return None
        readable, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not readable:
            return None
        return sys.stdin.read(1)


def utc_stamp() -> str:
    return datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open('rb') as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def percentile(values: Sequence[float], pct: float) -> float:
    clean = sorted(float(v) for v in values if math.isfinite(float(v)))
    if not clean:
        return float('nan')
    if len(clean) == 1:
        return clean[0]
    x = (pct / 100.0) * (len(clean) - 1)
    lo = int(math.floor(x))
    hi = int(math.ceil(x))
    if lo == hi:
        return clean[lo]
    w = x - lo
    return clean[lo] * (1.0 - w) + clean[hi] * w


def median(values: Iterable[float]) -> float:
    clean = [float(v) for v in values if math.isfinite(float(v))]
    return statistics.median(clean) if clean else float('nan')


def rms(values: Iterable[float]) -> float:
    clean = [float(v) for v in values if math.isfinite(float(v))]
    if not clean:
        return float('nan')
    return math.sqrt(sum(v * v for v in clean) / len(clean))


def sanitize_csv_value(value: object) -> object:
    if isinstance(value, float) and not math.isfinite(value):
        return ''
    return value


def resolve_servo_map(configured: str) -> Path:
    candidates: List[Path] = []
    if configured:
        candidates.append(Path(configured).expanduser())
    if get_package_share_directory is not None:
        try:
            candidates.append(
                Path(get_package_share_directory('lgh_st3215_driver'))
                / 'config' / 'servo_map.yaml'
            )
        except Exception:
            pass
    candidates.append(
        Path.home() / 'littlegreen_ros2_ws' / 'src' /
        'lgh_st3215_driver' / 'config' / 'servo_map.yaml'
    )
    for path in candidates:
        if path.is_file():
            return path.resolve()
    raise FileNotFoundError('servo_map.yaml not found; tried: ' + ', '.join(map(str, candidates)))


def load_joints(path: Path) -> List[JointConfig]:
    raw = yaml.safe_load(path.read_text())
    joints = raw.get('joints', []) if isinstance(raw, dict) else []
    if len(joints) != NUM_JOINTS:
        raise ValueError(f'expected {NUM_JOINTS} joints in {path}, got {len(joints)}')
    ordered = sorted(joints, key=lambda x: int(x['policy_index']))
    return [
        JointConfig(
            name=str(j['name']),
            index=int(j['policy_index']),
            servo_id=int(j['servo_id']),
            training_default_rad=float(j['training_default_rad']),
            min_rad=float(j['min_rad']),
            max_rad=float(j['max_rad']),
        )
        for j in ordered
    ]


def resolve_track1_contract(configured: str) -> Path:
    candidates: List[Path] = []
    if configured:
        candidates.append(Path(configured).expanduser())
    if get_package_share_directory is not None:
        try:
            candidates.append(
                Path(get_package_share_directory('lgh_st3215_tools'))
                / 'config' / 'track1_action_contract_v3.yaml'
            )
        except Exception:
            pass
    candidates.append(
        Path.home() / 'littlegreen_ros2_ws' / 'src' /
        'lgh_st3215_tools' / 'config' / 'track1_action_contract_v3.yaml'
    )
    for path in candidates:
        if path.is_file():
            return path.resolve()
    raise FileNotFoundError(
        'track1_action_contract_v3.yaml not found; tried: ' + ', '.join(map(str, candidates))
    )


def load_track1_contract(path: Path) -> Dict[str, object]:
    raw = yaml.safe_load(path.read_text())
    if not isinstance(raw, dict):
        raise ValueError(f'Track 1 contract {path} must contain a YAML mapping')
    return raw


def validate_track1_contract(
    joints: Sequence[JointConfig], contract: Dict[str, object], tolerance: float = 1.0e-9
) -> None:
    expected_names = [j.name for j in joints]
    names = list(contract.get('joint_order', []))
    defaults = list(contract.get('training_default_rad', []))
    lower = list(contract.get('lower_limit_rad', []))
    upper = list(contract.get('upper_limit_rad', []))
    if names != expected_names:
        raise ValueError('Track 1 contract joint_order does not match servo_map policy order')
    for label, values in [('training_default_rad', defaults), ('lower_limit_rad', lower), ('upper_limit_rad', upper)]:
        if len(values) != NUM_JOINTS:
            raise ValueError(f'Track 1 contract {label} must contain 12 values')
    mismatches: List[str] = []
    for i, joint in enumerate(joints):
        checks = [
            ('training_default_rad', joint.training_default_rad, float(defaults[i])),
            ('min_rad/lower_limit_rad', joint.min_rad, float(lower[i])),
            ('max_rad/upper_limit_rad', joint.max_rad, float(upper[i])),
        ]
        for label, map_value, contract_value in checks:
            if abs(map_value - contract_value) > tolerance:
                mismatches.append(
                    f'{joint.name} {label}: servo_map={map_value:.9f} contract={contract_value:.9f}'
                )
    if mismatches:
        raise ValueError('servo_map does not match Track 1 action contract v3:\n  ' + '\n  '.join(mismatches))


def pose_contract_audit_rows(
    target: Sequence[float], joints: Sequence[JointConfig], margin_rad: float
) -> List[Dict[str, object]]:
    rows: List[Dict[str, object]] = []
    for joint, q in zip(joints, target):
        q = float(q)
        lo = joint.min_rad + margin_rad
        hi = joint.max_rad - margin_rad
        delta = q - joint.training_default_rad
        if q < lo:
            status = 'BELOW_GUARDED_LIMIT'
            violation = lo - q
        elif q > hi:
            status = 'ABOVE_GUARDED_LIMIT'
            violation = q - hi
        else:
            status = 'OK'
            violation = 0.0
        rows.append({
            'joint_name': joint.name,
            'q_captured_rad': q,
            'training_default_rad': joint.training_default_rad,
            'delta_from_training_default_rad': delta,
            'guarded_lower_rad': lo,
            'guarded_upper_rad': hi,
            'nearest_guard_margin_rad': min(q - lo, hi - q),
            'limit_violation_rad': violation,
            'status': status,
        })
    return rows


def print_pose_contract_audit(
    pose_name: str,
    rows: Sequence[Dict[str, object]],
    measured_height_m: float,
    contract: Dict[str, object],
) -> None:
    desired_height = float(contract.get('desired_base_com_height_m', float('nan')))
    print('\nTrack 1 contract audit')
    print(f'  pose: {pose_name}')
    print(f'  measured base COM height: {measured_height_m:.6f} m')
    if math.isfinite(desired_height):
        print(f'  Track 1 nominal base COM: {desired_height:.6f} m')
        print(f'  height delta: {measured_height_m - desired_height:+.6f} m')
    print('')
    print('  joint                                  captured    q_default      delta      guarded range                 status')
    for row in rows:
        print(
            f"  {row['joint_name']:<38} "
            f"{float(row['q_captured_rad']):+9.6f}  "
            f"{float(row['training_default_rad']):+9.6f}  "
            f"{float(row['delta_from_training_default_rad']):+9.6f}  "
            f"[{float(row['guarded_lower_rad']):+8.4f}, {float(row['guarded_upper_rad']):+8.4f}]  "
            f"{row['status']}"
        )


def write_capture_audit(
    root: Path,
    pose_name: str,
    measured_height_m: float,
    q_target: Sequence[float],
    q_std: Sequence[float],
    raw_steps: Sequence[int],
    rows: Sequence[Dict[str, object]],
    map_path: Path,
    contract_path: Path,
    contract: Dict[str, object],
) -> Tuple[Path, Path]:
    root.mkdir(parents=True, exist_ok=True)
    stem = f'{utc_stamp()}_{pose_name}_capture_audit'
    yaml_path = root / f'{stem}.yaml'
    csv_path = root / f'{stem}.csv'
    desired_height = contract.get('desired_base_com_height_m')
    payload = {
        'schema_version': 1,
        'experiment_type': 'standing_pose_capture_contract_audit',
        'timestamp_utc': datetime.now(timezone.utc).isoformat(),
        'pose_name': pose_name,
        'base_com_height_mean_m': float(measured_height_m),
        'track1_desired_base_com_height_m': float(desired_height) if desired_height is not None else None,
        'servo_map_path': str(map_path),
        'servo_map_sha256': sha256_file(map_path),
        'track1_contract_path': str(contract_path),
        'track1_contract_sha256': sha256_file(contract_path),
        'q_captured_rad': [float(v) for v in q_target],
        'q_std_rad': [float(v) for v in q_std],
        'raw_position_steps_median': [int(v) for v in raw_steps],
        'joint_audit': list(rows),
        'accepted_for_pose_library': not any(r['status'] != 'OK' for r in rows),
    }
    yaml_path.write_text(yaml.safe_dump(payload, sort_keys=False, width=140))
    with csv_path.open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    return yaml_path, csv_path


def load_pose_library(path: Path, joints: Sequence[JointConfig]) -> Dict[str, object]:
    if not path.exists():
        return {
            'schema_version': 1,
            'robot': 'LittleGreen',
            'joint_order': [j.name for j in joints],
            'standing_base_com_height_mean_m': None,
            'poses': {},
        }
    raw = yaml.safe_load(path.read_text())
    if not isinstance(raw, dict):
        raise ValueError(f'pose library {path} must contain a YAML mapping')
    order = raw.get('joint_order')
    expected = [j.name for j in joints]
    if order is not None and list(order) != expected:
        raise ValueError('pose library joint_order does not match servo_map canonical order')
    raw.setdefault('schema_version', 1)
    raw.setdefault('robot', 'LittleGreen')
    raw['joint_order'] = expected
    raw.setdefault('standing_base_com_height_mean_m', None)
    raw.setdefault('poses', {})
    if not isinstance(raw['poses'], dict):
        raise ValueError('pose library poses must be a mapping')
    return raw


def save_pose_library(path: Path, data: Dict[str, object]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_suffix(path.suffix + '.tmp')
    temp.write_text(yaml.safe_dump(data, sort_keys=False, width=120))
    temp.replace(path)


def validate_pose_target(
    pose_name: str,
    target: Sequence[float],
    joints: Sequence[JointConfig],
    margin_rad: float,
) -> None:
    if len(target) != NUM_JOINTS:
        raise ValueError(f'{pose_name}: target_rad must contain 12 values')
    for joint, q in zip(joints, target):
        q = float(q)
        if not math.isfinite(q):
            raise ValueError(f'{pose_name}: {joint.name} target is not finite')
        lo = joint.min_rad + margin_rad
        hi = joint.max_rad - margin_rad
        if q < lo or q > hi:
            raise ValueError(
                f'{pose_name}: {joint.name} target {q:.6f} rad outside guarded range '
                f'[{lo:.6f}, {hi:.6f}]'
            )


BASE_CSV_FIELDS = [
    'wall_time_ns', 'header_stamp_ns', 'cycle_index',
    'cycle_start_monotonic_ns', 'cycle_end_monotonic_ns',
    'phase', 'segment_id', 'pose_name', 'source_pose_name', 'transition_kind',
    'base_com_height_mean_m', 'commanded_speed_limit_rad_s',
    'cycle_work_us', 'feedback_sweep_us', 'read_start_index',
    'write_attempted', 'write_ok', 'sync_write_us',
    'telemetry_dropped_count', 'torque_enabled_state',
    'imu_ang_vel_x', 'imu_ang_vel_y', 'imu_ang_vel_z',
    'imu_orientation_x', 'imu_orientation_y', 'imu_orientation_z', 'imu_orientation_w',
]
ARRAY_PREFIXES = [
    'q_ref', 'q_meas', 'qdot', 'error', 'load_ratio', 'current_a',
    'voltage_v', 'temperature_c', 'moving', 'read_ok', 'feedback_age_ms',
    'raw_position_steps', 'raw_speed', 'raw_load', 'raw_current', 'target_steps',
]
CSV_FIELDS = BASE_CSV_FIELDS + [
    f'{prefix}_{i:02d}' for prefix in ARRAY_PREFIXES for i in range(NUM_JOINTS)
]


class StandingLoadNode(Node):
    def __init__(self, joints: Sequence[JointConfig], csv_path: Optional[Path]) -> None:
        super().__init__('standing_load_characterization_runner')
        self.joints = list(joints)

        command_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        self.command_pub = self.create_publisher(
            Float64MultiArray, '/servo_target_radians', command_qos
        )
        self.create_subscription(
            ServoTelemetry, '/st3215_driver/telemetry', self._telemetry_cb, sensor_qos
        )
        self.create_subscription(
            DiagnosticArray, '/st3215_driver/diagnostics', self._diagnostics_cb, reliable_qos
        )
        self.create_subscription(JointState, '/joint_states', self._joint_state_cb, sensor_qos)
        self.create_subscription(
            UInt32MultiArray, '/joint_feedback_age_ms', self._feedback_age_cb, sensor_qos
        )
        self.create_subscription(Imu, '/imu/data', self._imu_cb, sensor_qos)

        self.disable_torque_client = self.create_client(
            Trigger, '/st3215_driver/disable_torque_all'
        )
        self.enable_torque_hold_client = self.create_client(
            Trigger, '/st3215_driver/enable_torque_hold_current'
        )
        self.release_override_client = self.create_client(
            Trigger, '/st3215_driver/release_pose_override'
        )
        self.hold_current_pose_client = self.create_client(
            Trigger, '/st3215_driver/hold_current_pose'
        )

        self.latest_telemetry: Optional[ServoTelemetry] = None
        self.last_telemetry_monotonic: Optional[float] = None
        self.last_telemetry_dropped_count = 0
        self.drop_guard_baseline: Optional[int] = None
        self.current_position: Optional[List[float]] = None
        self.current_velocity: Optional[List[float]] = None
        self.feedback_age_ms: Optional[List[int]] = None
        self.last_joint_state_monotonic: Optional[float] = None
        self.diag_level: Optional[int] = None
        self.diag_message = ''
        self.diag_values: Dict[str, str] = {}
        self.last_diag_monotonic: Optional[float] = None
        self.imu: Optional[Tuple[float, float, float, float, float, float, float]] = None

        self.phase = 'preflight'
        self.segment_id = 0
        self.pose_name = ''
        self.source_pose_name = ''
        self.transition_kind = ''
        self.base_com_height_mean_m = float('nan')
        self.commanded_speed_limit_rad_s = float('nan')
        self.reference = [0.0] * NUM_JOINTS
        self.logging_active = False
        self.records: List[Dict[str, object]] = []
        self.capture_buffer: List[Dict[str, object]] = []
        self.capture_active = False

        self.csv_file = None
        self.csv_writer = None
        self.rows_written = 0
        if csv_path is not None:
            self.csv_file = csv_path.open('w', newline='', buffering=1)
            self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=CSV_FIELDS)
            self.csv_writer.writeheader()

    def close(self) -> None:
        if self.csv_file is not None and not self.csv_file.closed:
            self.csv_file.flush()
            os.fsync(self.csv_file.fileno())
            self.csv_file.close()

    def _joint_state_cb(self, msg: JointState) -> None:
        if len(msg.position) < NUM_JOINTS or len(msg.velocity) < NUM_JOINTS:
            return
        q = [float(v) for v in msg.position[:NUM_JOINTS]]
        qdot = [float(v) for v in msg.velocity[:NUM_JOINTS]]
        if all(math.isfinite(v) for v in q + qdot):
            self.current_position = q
            self.current_velocity = qdot
            self.last_joint_state_monotonic = time.monotonic()

    def _feedback_age_cb(self, msg: UInt32MultiArray) -> None:
        if len(msg.data) >= NUM_JOINTS:
            self.feedback_age_ms = [int(v) for v in msg.data[:NUM_JOINTS]]

    def _imu_cb(self, msg: Imu) -> None:
        self.imu = (
            float(msg.angular_velocity.x), float(msg.angular_velocity.y), float(msg.angular_velocity.z),
            float(msg.orientation.x), float(msg.orientation.y), float(msg.orientation.z), float(msg.orientation.w),
        )

    def _diagnostics_cb(self, msg: DiagnosticArray) -> None:
        if not msg.status:
            return
        status = msg.status[0]
        self.diag_level = diagnostic_level_to_int(status.level)
        self.diag_message = str(status.message)
        self.diag_values = {str(kv.key): str(kv.value) for kv in status.values}
        self.last_diag_monotonic = time.monotonic()

    @staticmethod
    def _header_stamp_ns(msg: ServoTelemetry) -> int:
        return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)

    def _snapshot_from_msg(self, msg: ServoTelemetry) -> Dict[str, object]:
        q_ref = [
            float(msg.target_rad_from_steps[i]) if bool(msg.target_valid)
            else float(msg.command_target_rad[i])
            for i in range(NUM_JOINTS)
        ]
        q = [float(v) for v in msg.q_meas_rad]
        record: Dict[str, object] = {
            'wall_time_ns': time.time_ns(),
            'header_stamp_ns': self._header_stamp_ns(msg),
            'cycle_index': int(msg.cycle_index),
            'cycle_start_monotonic_ns': int(msg.cycle_start_monotonic_ns),
            'cycle_end_monotonic_ns': int(msg.cycle_end_monotonic_ns),
            'phase': self.phase,
            'segment_id': self.segment_id,
            'pose_name': self.pose_name,
            'source_pose_name': self.source_pose_name,
            'transition_kind': self.transition_kind,
            'base_com_height_mean_m': self.base_com_height_mean_m,
            'commanded_speed_limit_rad_s': self.commanded_speed_limit_rad_s,
            'cycle_work_us': float(msg.cycle_work_us),
            'feedback_sweep_us': float(msg.feedback_sweep_us),
            'read_start_index': int(msg.read_start_index),
            'write_attempted': int(bool(msg.write_attempted)),
            'write_ok': int(bool(msg.write_ok)),
            'sync_write_us': float(msg.sync_write_us),
            'telemetry_dropped_count': int(msg.telemetry_dropped_count),
            'torque_enabled_state': int(msg.torque_enabled_state),
            'q_ref': q_ref,
            'q_meas': q,
            'qdot': [float(v) for v in msg.qdot_meas_rad_s],
            'error': [q_ref[i] - q[i] for i in range(NUM_JOINTS)],
            'load_ratio': [float(v) for v in msg.load_ratio],
            'current_a': [float(v) for v in msg.current_a],
            'voltage_v': [float(v) for v in msg.voltage_v],
            'temperature_c': [float(v) for v in msg.temperature_c],
            'moving': [int(bool(v)) for v in msg.moving],
            'read_ok': [int(bool(v)) for v in msg.read_ok],
            'feedback_age_ms': [float(v) for v in msg.feedback_age_ms_at_cycle_end],
            'raw_position_steps': [int(v) for v in msg.raw_position_steps],
            'raw_speed': [int(v) for v in msg.raw_speed],
            'raw_load': [int(v) for v in msg.raw_load],
            'raw_current': [int(v) for v in msg.raw_current],
            'target_steps': [int(v) for v in msg.target_steps],
            'imu': self.imu,
        }
        return record

    def _write_csv_record(self, record: Dict[str, object]) -> None:
        if self.csv_writer is None:
            return
        imu = record.get('imu')
        row: Dict[str, object] = {k: record.get(k, '') for k in BASE_CSV_FIELDS}
        if imu is not None:
            names = [
                'imu_ang_vel_x', 'imu_ang_vel_y', 'imu_ang_vel_z',
                'imu_orientation_x', 'imu_orientation_y', 'imu_orientation_z', 'imu_orientation_w',
            ]
            for name, value in zip(names, imu):
                row[name] = value
        for prefix in ARRAY_PREFIXES:
            values = record[prefix]
            for i in range(NUM_JOINTS):
                row[f'{prefix}_{i:02d}'] = values[i]
        self.csv_writer.writerow({k: sanitize_csv_value(v) for k, v in row.items()})
        self.rows_written += 1
        if self.rows_written % 250 == 0 and self.csv_file is not None:
            self.csv_file.flush()

    def _telemetry_cb(self, msg: ServoTelemetry) -> None:
        self.latest_telemetry = msg
        self.last_telemetry_monotonic = time.monotonic()
        self.last_telemetry_dropped_count = int(msg.telemetry_dropped_count)
        if not self.logging_active and not self.capture_active:
            return
        record = self._snapshot_from_msg(msg)
        if self.capture_active:
            self.capture_buffer.append(record)
        if self.logging_active:
            if self.records and int(record['cycle_index']) <= int(self.records[-1]['cycle_index']):
                return
            self.records.append(record)
            self._write_csv_record(record)

    def publish_reference(self, target: Sequence[float]) -> None:
        if len(target) != NUM_JOINTS:
            raise ValueError('reference must contain 12 values')
        msg = Float64MultiArray()
        msg.data = [float(v) for v in target]
        self.reference = list(msg.data)
        self.command_pub.publish(msg)

    def call_trigger(self, client, label: str, timeout_sec: float = 2.0) -> Tuple[bool, str]:
        if not client.wait_for_service(timeout_sec=timeout_sec):
            return False, f'{label} service unavailable'
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done():
            return False, f'{label} service timed out'
        try:
            response = future.result()
        except Exception as exc:  # pragma: no cover
            return False, f'{label} service error: {exc}'
        return bool(response.success), str(response.message)

    def feedback_ready(self, max_age_ms: int, timeout_sec: float) -> bool:
        if self.current_position is None or self.feedback_age_ms is None:
            return False
        if self.last_joint_state_monotonic is None:
            return False
        if time.monotonic() - self.last_joint_state_monotonic > timeout_sec:
            return False
        return len(self.feedback_age_ms) >= NUM_JOINTS and max(self.feedback_age_ms) <= max_age_ms

    def graph_guard(self) -> Tuple[bool, str, Dict[str, object]]:
        nodes = []
        for name, ns in self.get_node_names_and_namespaces():
            full = (ns.rstrip('/') + '/' + name) if ns.rstrip('/') else '/' + name
            nodes.append(full)
        forbidden = [
            n for n in nodes
            if n.endswith('/littlegreen_biped_node')
            or n.endswith('/pd_controller_node')
            or n.endswith('/reference_shaper_node')
        ]
        publishers = []
        for info in self.get_publishers_info_by_topic('/servo_target_radians'):
            if info.node_name == self.get_name():
                continue
            full = (info.node_namespace.rstrip('/') + '/' + info.node_name) if info.node_namespace.rstrip('/') else '/' + info.node_name
            publishers.append(full)
        details = {'forbidden_nodes': sorted(set(forbidden)), 'external_servo_publishers': sorted(set(publishers))}
        if forbidden:
            return False, f'policy/PD/shaper node(s) active: {forbidden}', details
        if publishers:
            return False, f'competing /servo_target_radians publisher(s): {publishers}', details
        return True, 'graph guard passed', details

    def common_ready(
        self,
        *,
        max_feedback_age_ms: int,
        joint_state_timeout_sec: float,
        require_torque_state: Optional[int],
        allow_pose_override: bool,
    ) -> Tuple[bool, str]:
        if self.last_diag_monotonic is None or time.monotonic() - self.last_diag_monotonic > 2.5:
            return False, 'driver diagnostics missing or stale'
        if self.last_telemetry_monotonic is None or time.monotonic() - self.last_telemetry_monotonic > 0.5:
            return False, 'driver telemetry missing or stale'
        if self.diag_values.get('writes_enabled', 'false').lower() != 'true':
            return False, 'driver writes_enabled is not true'
        if self.diag_values.get('motion_profile', '') != 'max_envelope_fixed_0_0':
            return False, f"unexpected motion profile {self.diag_values.get('motion_profile', '')!r}"
        if self.diag_values.get('feedback_ready', 'false').lower() != 'true':
            return False, 'driver feedback_ready is not true'
        if self.diag_values.get('pose_move_running', 'false').lower() == 'true':
            return False, 'driver guarded pose move is active'
        if not allow_pose_override and self.diag_values.get('pose_override_active', 'false').lower() == 'true':
            return False, 'driver pose override is active'
        if not self.feedback_ready(max_feedback_age_ms, joint_state_timeout_sec):
            return False, 'joint feedback missing or stale'
        if require_torque_state is not None:
            state = int(getattr(self.latest_telemetry, 'torque_enabled_state', -1)) if self.latest_telemetry is not None else -1
            if state != require_torque_state:
                return False, f'torque_enabled_state={state}, expected {require_torque_state}'
        ok, reason, _ = self.graph_guard()
        if not ok:
            return False, reason
        return True, 'ready'

    def continuous_guard(self, args) -> None:
        if self.drop_guard_baseline is not None and self.last_telemetry_dropped_count > self.drop_guard_baseline:
            raise AbortRequested(
                f'telemetry dropped cycles: {self.drop_guard_baseline} -> {self.last_telemetry_dropped_count}'
            )
        ok, reason = self.common_ready(
            max_feedback_age_ms=args.max_feedback_age_ms,
            joint_state_timeout_sec=args.joint_state_timeout_sec,
            require_torque_state=1,
            allow_pose_override=False,
        )
        if not ok:
            raise AbortRequested(reason)
        if self.latest_telemetry is None:
            raise AbortRequested('telemetry unavailable')
        msg = self.latest_telemetry
        max_current = max(abs(float(v)) for v in msg.current_a)
        max_load = max(abs(float(v)) for v in msg.load_ratio)
        min_voltage = min(float(v) for v in msg.voltage_v)
        temps = [float(v) for v in msg.temperature_c]
        # Temperature outliers have been observed as isolated one-cycle spikes. The
        # robust guard below uses a rolling median implemented by consecutive counts.
        if args.max_current_a > 0 and max_current > args.max_current_a:
            self._current_guard_count += 1
        else:
            self._current_guard_count = 0
        if args.max_load_ratio > 0 and max_load > args.max_load_ratio:
            self._load_guard_count += 1
        else:
            self._load_guard_count = 0
        if args.min_voltage_v > 0 and min_voltage < args.min_voltage_v:
            self._voltage_guard_count += 1
        else:
            self._voltage_guard_count = 0
        if args.max_temp_c > 0 and max(temps) > args.max_temp_c:
            self._temp_guard_count += 1
        else:
            self._temp_guard_count = 0
        n = args.guard_consecutive_cycles
        if self._current_guard_count >= n:
            raise AbortRequested(f'current guard exceeded: {max_current:.3f} A')
        if self._load_guard_count >= n:
            raise AbortRequested(f'load guard exceeded: {max_load:.3f}')
        if self._voltage_guard_count >= n:
            raise AbortRequested(f'voltage guard exceeded: {min_voltage:.2f} V')
        if self._temp_guard_count >= n:
            raise AbortRequested(f'temperature guard exceeded: max {max(temps):.1f} C')

    _current_guard_count = 0
    _load_guard_count = 0
    _voltage_guard_count = 0
    _temp_guard_count = 0


def spin_until(node: Node, deadline: float) -> None:
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=min(0.02, max(0.0, deadline - time.monotonic())))


def wait_for_ready(node: StandingLoadNode, args, *, torque: Optional[int], allow_override: bool) -> None:
    deadline = time.monotonic() + args.preflight_timeout_sec
    last_reason = 'waiting'
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        ok, reason = node.common_ready(
            max_feedback_age_ms=args.max_feedback_age_ms,
            joint_state_timeout_sec=args.joint_state_timeout_sec,
            require_torque_state=torque,
            allow_pose_override=allow_override,
        )
        if ok:
            return
        last_reason = reason
    raise RuntimeError(f'preflight failed: {last_reason}')


def wait_for_enter_with_spin(node: Node, prompt: str) -> None:
    print(prompt, flush=True)
    done = threading.Event()
    error: List[BaseException] = []

    def reader() -> None:
        try:
            input()
        except BaseException as exc:  # pragma: no cover
            error.append(exc)
        finally:
            done.set()

    thread = threading.Thread(target=reader, daemon=True)
    thread.start()
    while rclpy.ok() and not done.is_set():
        rclpy.spin_once(node, timeout_sec=0.05)
    if error:
        raise error[0]


def arm_phrase_with_spin(node: Node, expected: str) -> None:
    print('\nType exactly:')
    print(expected)
    response: List[str] = []
    done = threading.Event()

    def reader() -> None:
        try:
            response.append(input('ARM> ').strip())
        finally:
            done.set()

    threading.Thread(target=reader, daemon=True).start()
    while rclpy.ok() and not done.is_set():
        rclpy.spin_once(node, timeout_sec=0.05)
    if not response or response[0] != expected:
        raise RuntimeError('arming phrase mismatch; no action performed')


def collect_capture(node: StandingLoadNode, duration_sec: float) -> List[Dict[str, object]]:
    node.capture_buffer.clear()
    node.capture_active = True
    deadline = time.monotonic() + duration_sec
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.capture_active = False
    return list(node.capture_buffer)


def capture_pose_mode(
    node: StandingLoadNode,
    joints: Sequence[JointConfig],
    map_path: Path,
    contract_path: Path,
    contract: Dict[str, object],
    args,
) -> int:
    if not args.pose_name:
        raise ValueError('--pose-name is required for capture_pose mode')
    if args.base_com_height_mean_m is None or not math.isfinite(args.base_com_height_mean_m):
        raise ValueError('--base-com-height-mean-m is required for capture_pose mode')

    wait_for_ready(node, args, torque=None, allow_override=True)
    expected = f'TORQUE OFF FOR POSE CAPTURE {args.pose_name}'
    print('\nCAPTURE MODE SAFETY: the robot must be physically supported before torque is disabled.')
    arm_phrase_with_spin(node, expected)
    ok, message = node.call_trigger(node.disable_torque_client, 'disable_torque_all', 3.0)
    print(f'disable_torque_all: success={ok} message={message}')
    if not ok:
        raise RuntimeError(message)

    # Wait for telemetry to report torque disabled.
    deadline = time.monotonic() + 2.0
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if node.latest_telemetry is not None and int(node.latest_telemetry.torque_enabled_state) == 0:
            break
    else:
        raise RuntimeError('torque-disabled state not observed in telemetry')

    print('\nAll-servo torque is OFF. Keep the robot mechanically supported.')
    print(f'Manually place the robot at pose {args.pose_name!r}.')
    print(f'Measured base-COM height to store: {args.base_com_height_mean_m:.4f} m')
    wait_for_enter_with_spin(
        node,
        'When the pose and measured height are stable, press ENTER to begin capture.',
    )

    samples = collect_capture(node, args.capture_window_sec)
    if len(samples) < args.capture_min_samples:
        raise RuntimeError(
            f'capture produced only {len(samples)} cycles; minimum is {args.capture_min_samples}'
        )
    q_target = [median(r['q_meas'][i] for r in samples) for i in range(NUM_JOINTS)]
    q_std = [statistics.pstdev([float(r['q_meas'][i]) for r in samples]) for i in range(NUM_JOINTS)]
    raw_steps = [int(round(median(r['raw_position_steps'][i] for r in samples))) for i in range(NUM_JOINTS)]
    if max(q_std) > args.capture_max_q_std_rad:
        worst = max(range(NUM_JOINTS), key=lambda i: q_std[i])
        raise RuntimeError(
            f'pose capture not stable enough: {joints[worst].name} std={q_std[worst]:.6f} rad '
            f'exceeds --capture-max-q-std-rad={args.capture_max_q_std_rad:.6f}'
        )
    audit_rows = pose_contract_audit_rows(q_target, joints, args.joint_limit_margin_rad)
    print_pose_contract_audit(
        args.pose_name, audit_rows, float(args.base_com_height_mean_m), contract
    )
    audit_yaml, audit_csv = write_capture_audit(
        Path(args.capture_audit_root).expanduser(),
        args.pose_name,
        float(args.base_com_height_mean_m),
        q_target, q_std, raw_steps, audit_rows,
        map_path, contract_path, contract,
    )
    print(f'\nCapture audit saved: {audit_yaml}')
    print(f'Capture audit CSV:   {audit_csv}')
    violations = [r for r in audit_rows if r['status'] != 'OK']
    if violations:
        details = '; '.join(
            f"{r['joint_name']} {r['status']} by {float(r['limit_violation_rad']):.6f} rad"
            for r in violations
        )
        raise ValueError(
            f'{args.pose_name}: captured pose is outside the Track 1 / servo-map guarded contract; '
            f'pose was NOT added to the executable pose library. {details}. '
            f'Review the complete capture audit instead of widening limits blindly.'
        )

    library_path = Path(args.pose_library).expanduser()
    library = load_pose_library(library_path, joints)
    library['track1_contract'] = {
        'action_contract_version': contract.get('action_contract_version'),
        'desired_base_com_height_m': contract.get('desired_base_com_height_m'),
        'residual_half_range_rad': contract.get('residual_half_range_rad'),
        'track1_contract_sha256': sha256_file(contract_path),
        'servo_map_sha256': sha256_file(map_path),
    }
    if args.pose_name == args.standing_pose_name:
        library['standing_base_com_height_mean_m'] = float(args.base_com_height_mean_m)
    library['poses'][args.pose_name] = {
        'base_com_height_mean_m': float(args.base_com_height_mean_m),
        'target_rad': [float(v) for v in q_target],
        'capture': {
            'timestamp_utc': datetime.now(timezone.utc).isoformat(),
            'capture_window_sec': float(args.capture_window_sec),
            'sample_count': len(samples),
            'q_std_rad': [float(v) for v in q_std],
            'raw_position_steps_median': raw_steps,
            'torque_enabled_state': 0,
            'notes': args.notes or '',
        },
    }
    save_pose_library(library_path, library)

    print('\nPose saved:')
    print(f'  library: {library_path}')
    print(f'  pose: {args.pose_name}')
    print(f'  base_com_height_mean_m: {args.base_com_height_mean_m:.4f}')
    print('  target_rad:')
    for joint, q, sigma in zip(joints, q_target, q_std):
        print(f'    {joint.name}: {q:+.6f} rad  capture_std={sigma:.6f}')
    print('\nTorque remains OFF. Keep supporting the robot.')

    if args.reenable_torque_hold_after_capture:
        expected2 = f'ENABLE TORQUE HOLD {args.pose_name}'
        arm_phrase_with_spin(node, expected2)
        ok, message = node.call_trigger(
            node.enable_torque_hold_client, 'enable_torque_hold_current', 4.0
        )
        print(f'enable_torque_hold_current: success={ok} message={message}')
        if not ok:
            raise RuntimeError(message)
        print('Torque is enabled at the measured pose; driver pose override remains active.')
    return 0


def make_eval_sequence(requested: Sequence[str], standing_pose: str, return_between: bool) -> List[str]:
    if not requested:
        raise ValueError('evaluation requires at least one pose')
    if not return_between:
        return list(requested)
    sequence: List[str] = []
    if requested[0] != standing_pose:
        sequence.append(standing_pose)
    for pose in requested:
        if not sequence or sequence[-1] != pose:
            sequence.append(pose)
        if pose != standing_pose and sequence[-1] != standing_pose:
            sequence.append(standing_pose)
    return sequence


def transition_kind_and_speed(
    source_height: Optional[float], target_height: Optional[float], args
) -> Tuple[str, float]:
    if source_height is not None and target_height is not None:
        if target_height < source_height - 1e-4:
            return 'crouch', args.crouch_speed_rad_s
        if target_height > source_height + 1e-4:
            return 'stand_return', args.stand_return_speed_rad_s
    return 'level_transition', args.transition_speed_rad_s


def run_phase(
    node: StandingLoadNode,
    keys: TerminalKeys,
    args,
    *,
    target: Sequence[float],
    duration_sec: float,
    segment_id: int,
    phase: str,
    pose_name: str,
    source_pose_name: str,
    transition_kind: str,
    height_m: Optional[float],
    speed_limit: float,
) -> None:
    node.phase = phase
    node.segment_id = segment_id
    node.pose_name = pose_name
    node.source_pose_name = source_pose_name
    node.transition_kind = transition_kind
    node.base_com_height_mean_m = float(height_m) if height_m is not None else float('nan')
    node.commanded_speed_limit_rad_s = float(speed_limit)

    period = 1.0 / args.command_rate_hz
    next_tick = time.monotonic()
    end_time = next_tick + duration_sec
    next_guard = time.monotonic()
    while rclpy.ok() and time.monotonic() < end_time:
        key = keys.read_key()
        if key in (' ', 'q', 'Q', '\x1b'):
            raise AbortRequested(f'operator abort key {key!r}')
        now = time.monotonic()
        if now >= next_guard:
            node.continuous_guard(args)
            next_guard = now + 0.25
        if now >= next_tick:
            node.publish_reference(target)
            next_tick += period
            if now - next_tick > period:
                next_tick = now + period
        rclpy.spin_once(node, timeout_sec=min(0.005, max(0.0, next_tick - time.monotonic())))


def run_transition(
    node: StandingLoadNode,
    keys: TerminalKeys,
    args,
    *,
    start: Sequence[float],
    target: Sequence[float],
    segment_id: int,
    pose_name: str,
    source_pose_name: str,
    transition_kind: str,
    height_m: Optional[float],
    speed_limit: float,
) -> float:
    max_delta = max(abs(float(b) - float(a)) for a, b in zip(start, target))
    # cubic smoothstep derivative peaks at 1.5, so use 1.5*delta/v to keep
    # commanded peak joint speed <= the requested speed limit.
    duration = max(args.min_transition_sec, 1.5 * max_delta / max(speed_limit, 1e-6))
    period = 1.0 / args.command_rate_hz
    t0 = time.monotonic()
    next_tick = t0
    end = t0 + duration
    next_guard = t0
    node.phase = 'transition'
    node.segment_id = segment_id
    node.pose_name = pose_name
    node.source_pose_name = source_pose_name
    node.transition_kind = transition_kind
    node.base_com_height_mean_m = float(height_m) if height_m is not None else float('nan')
    node.commanded_speed_limit_rad_s = float(speed_limit)

    while rclpy.ok() and time.monotonic() < end:
        key = keys.read_key()
        if key in (' ', 'q', 'Q', '\x1b'):
            raise AbortRequested(f'operator abort key {key!r}')
        now = time.monotonic()
        if now >= next_guard:
            node.continuous_guard(args)
            next_guard = now + 0.25
        if now >= next_tick:
            u = min(1.0, max(0.0, (now - t0) / duration))
            smooth = u * u * (3.0 - 2.0 * u)
            ref = [float(a) + smooth * (float(b) - float(a)) for a, b in zip(start, target)]
            node.publish_reference(ref)
            next_tick += period
            if now - next_tick > period:
                next_tick = now + period
        rclpy.spin_once(node, timeout_sec=min(0.005, max(0.0, next_tick - time.monotonic())))
    node.publish_reference(target)
    return duration


def pose_summary_rows(records: Sequence[Dict[str, object]], joints: Sequence[JointConfig]) -> List[Dict[str, object]]:
    rows: List[Dict[str, object]] = []
    pose_names = sorted({str(r['pose_name']) for r in records if r['phase'] == 'hold'})
    for pose in pose_names:
        subset = [r for r in records if r['phase'] == 'hold' and r['pose_name'] == pose]
        if not subset:
            continue
        for i, joint in enumerate(joints):
            errors = [float(r['error'][i]) for r in subset]
            abs_load = [abs(float(r['load_ratio'][i])) for r in subset]
            abs_current = [abs(float(r['current_a'][i])) for r in subset]
            voltage = [float(r['voltage_v'][i]) for r in subset]
            temp = [float(r['temperature_c'][i]) for r in subset]
            qdot = [float(r['qdot'][i]) for r in subset]
            q_meas = [float(r['q_meas'][i]) for r in subset]
            power = [voltage[k] * abs_current[k] for k in range(len(subset))]
            rows.append({
                'pose_name': pose,
                'base_com_height_mean_m': subset[0]['base_com_height_mean_m'],
                'joint_index': i,
                'joint_name': joint.name,
                'q_target_rad_median': median(r['q_ref'][i] for r in subset),
                'q_meas_rad_median': median(q_meas),
                'q_meas_std_rad': statistics.pstdev(q_meas) if len(q_meas) > 1 else 0.0,
                'steady_error_rad_median': median(errors),
                'steady_error_abs_rad_median': median(abs(v) for v in errors),
                'steady_error_abs_rad_p95': percentile([abs(v) for v in errors], 95),
                'load_abs_median': median(abs_load),
                'load_abs_p95': percentile(abs_load, 95),
                'load_abs_max': max(abs_load),
                'current_abs_a_median': median(abs_current),
                'current_abs_a_p95': percentile(abs_current, 95),
                'current_abs_a_max': max(abs_current),
                'voltage_v_median': median(voltage),
                'voltage_v_min': min(voltage),
                'temperature_c_median': median(temp),
                'temperature_c_p95': percentile(temp, 95),
                'temperature_c_raw_max': max(temp),
                'qdot_rms_rad_s': rms(qdot),
                'qdot_abs_p95_rad_s': percentile([abs(v) for v in qdot], 95),
                'moving_duty_fraction': sum(int(r['moving'][i]) for r in subset) / len(subset),
                'read_ok_fraction': sum(int(r['read_ok'][i]) for r in subset) / len(subset),
                'power_w_median': median(power),
                'power_w_p95': percentile(power, 95),
                'sample_cycles': len(subset),
            })
    return rows


def pose_level_rows(records: Sequence[Dict[str, object]]) -> List[Dict[str, object]]:
    rows = []
    pose_names = sorted({str(r['pose_name']) for r in records if r['phase'] == 'hold'})
    for pose in pose_names:
        subset = [r for r in records if r['phase'] == 'hold' and r['pose_name'] == pose]
        total_power = [
            sum(float(r['voltage_v'][i]) * abs(float(r['current_a'][i])) for i in range(NUM_JOINTS))
            for r in subset
        ]
        omega = []
        for r in subset:
            imu = r.get('imu')
            if imu is not None:
                omega.append(math.sqrt(float(imu[0])**2 + float(imu[1])**2 + float(imu[2])**2))
        rows.append({
            'pose_name': pose,
            'base_com_height_mean_m': subset[0]['base_com_height_mean_m'],
            'total_servo_power_w_median_approx': median(total_power),
            'total_servo_power_w_p95_approx': percentile(total_power, 95),
            'min_joint_voltage_v': min(float(v) for r in subset for v in r['voltage_v']),
            'imu_angular_velocity_rms_rad_s': rms(omega),
            'imu_angular_velocity_p95_rad_s': percentile(omega, 95),
            'sample_cycles': len(subset),
        })
    return rows


def bilateral_rows(pose_rows: Sequence[Dict[str, object]]) -> List[Dict[str, object]]:
    index = {(r['pose_name'], r['joint_name']): r for r in pose_rows}
    rows = []
    for pose in sorted({r['pose_name'] for r in pose_rows}):
        left_names = [r['joint_name'] for r in pose_rows if r['pose_name'] == pose and str(r['joint_name']).startswith('leg_left_')]
        for left in left_names:
            right = left.replace('leg_left_', 'leg_right_', 1)
            a = index.get((pose, left))
            b = index.get((pose, right))
            if a is None or b is None:
                continue
            def asym(key: str) -> float:
                x = abs(float(a[key])); y = abs(float(b[key]))
                return abs(x - y) / max(0.5 * (x + y), 1e-9)
            rows.append({
                'pose_name': pose,
                'joint_family': left.replace('leg_left_', '').replace('_joint', ''),
                'left_joint': left,
                'right_joint': right,
                'steady_error_abs_asymmetry_fraction': asym('steady_error_abs_rad_median'),
                'load_abs_asymmetry_fraction': asym('load_abs_median'),
                'current_abs_asymmetry_fraction': asym('current_abs_a_median'),
                'qdot_rms_asymmetry_fraction': asym('qdot_rms_rad_s'),
            })
    return rows


def transition_rows(records: Sequence[Dict[str, object]], joints: Sequence[JointConfig]) -> List[Dict[str, object]]:
    groups: Dict[Tuple[int, str, str, str], List[Dict[str, object]]] = {}
    for r in records:
        if r['phase'] != 'transition':
            continue
        key = (int(r['segment_id']), str(r['source_pose_name']), str(r['pose_name']), str(r['transition_kind']))
        groups.setdefault(key, []).append(r)
    rows = []
    for (segment_id, source, target, kind), subset in groups.items():
        for i, joint in enumerate(joints):
            errors = [float(r['error'][i]) for r in subset]
            qdot = [float(r['qdot'][i]) for r in subset]
            rows.append({
                'segment_id': segment_id,
                'source_pose_name': source,
                'target_pose_name': target,
                'transition_kind': kind,
                'commanded_speed_limit_rad_s': subset[0]['commanded_speed_limit_rad_s'],
                'duration_sec_observed': (int(subset[-1]['cycle_end_monotonic_ns']) - int(subset[0]['cycle_start_monotonic_ns'])) / 1e9,
                'joint_index': i,
                'joint_name': joint.name,
                'tracking_error_rms_rad': rms(errors),
                'tracking_error_abs_p95_rad': percentile([abs(v) for v in errors], 95),
                'peak_abs_qdot_rad_s': max(abs(v) for v in qdot),
                'peak_abs_load_ratio': max(abs(float(r['load_ratio'][i])) for r in subset),
                'peak_abs_current_a': max(abs(float(r['current_a'][i])) for r in subset),
                'min_voltage_v': min(float(r['voltage_v'][i]) for r in subset),
                'temperature_c_p95': percentile([float(r['temperature_c'][i]) for r in subset], 95),
                'sample_cycles': len(subset),
            })
    return rows


def write_dict_csv(path: Path, rows: Sequence[Dict[str, object]]) -> None:
    if not rows:
        path.write_text('')
        return
    fields = list(rows[0].keys())
    with path.open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({k: sanitize_csv_value(v) for k, v in row.items()})


def evaluate_mode(node: StandingLoadNode, joints: Sequence[JointConfig], map_path: Path, args, output_dir: Path) -> int:
    library_path = Path(args.pose_library).expanduser()
    library = load_pose_library(library_path, joints)
    poses = library['poses']
    requested = [p.strip() for p in args.poses.split(',') if p.strip()]
    if args.target_base_com_height_m is not None:
        candidates = []
        for name, pose in poses.items():
            height = pose.get('base_com_height_mean_m') if isinstance(pose, dict) else None
            if height is not None:
                candidates.append((abs(float(height) - args.target_base_com_height_m), name, float(height)))
        if not candidates:
            raise ValueError('pose library contains no base_com_height_mean_m values')
        delta, nearest_name, nearest_height = min(candidates)
        if delta > args.height_match_tolerance_m:
            raise ValueError(
                f'nearest pose {nearest_name!r} is {nearest_height:.4f} m, '
                f'{delta:.4f} m from requested {args.target_base_com_height_m:.4f} m; '
                f'tolerance is {args.height_match_tolerance_m:.4f} m'
            )
        requested = [nearest_name]
        print(
            f'Selected pose {nearest_name!r} for requested base-COM height '
            f'{args.target_base_com_height_m:.4f} m (captured {nearest_height:.4f} m).'
        )
    missing = [p for p in set(requested + [args.standing_pose_name]) if p not in poses]
    if missing:
        raise ValueError(f'pose(s) not found in {library_path}: {sorted(missing)}')

    for name in set(requested + [args.standing_pose_name]):
        validate_pose_target(name, poses[name]['target_rad'], joints, args.joint_limit_margin_rad)

    sequence = make_eval_sequence(requested, args.standing_pose_name, args.return_between_poses)
    print('\nEvaluation sequence:')
    for i, name in enumerate(sequence, 1):
        print(f'  {i}. {name}: height={poses[name].get("base_com_height_mean_m")} m')
    print(f'  crouch speed limit: {args.crouch_speed_rad_s:.3f} rad/s')
    print(f'  stand-return speed limit: {args.stand_return_speed_rad_s:.3f} rad/s')

    wait_for_ready(node, args, torque=None, allow_override=True)
    arm_phrase_with_spin(node, 'ARM STANDING LOAD EVALUATION')

    ok, message = node.call_trigger(
        node.enable_torque_hold_client, 'enable_torque_hold_current', 4.0
    )
    print(f'enable_torque_hold_current: success={ok} message={message}')
    if not ok:
        raise RuntimeError(message)

    # Hold the current measured pose with an active command stream before releasing override.
    spin_until(node, time.monotonic() + 0.2)
    if node.current_position is None:
        raise RuntimeError('measured position unavailable after torque enable')
    current_ref = list(node.current_position)
    for _ in range(max(1, int(0.5 * args.command_rate_hz))):
        node.publish_reference(current_ref)
        rclpy.spin_once(node, timeout_sec=1.0 / args.command_rate_hz)
    ok, message = node.call_trigger(node.release_override_client, 'release_pose_override', 3.0)
    print(f'release_pose_override: success={ok} message={message}')
    if not ok:
        raise RuntimeError(message)

    wait_for_ready(node, args, torque=1, allow_override=False)
    node.drop_guard_baseline = node.last_telemetry_dropped_count
    node.logging_active = True

    final_status = 'completed'
    transition_plan: List[Dict[str, object]] = []
    try:
        with TerminalKeys() as keys:
            source_name = 'measured_start'
            source_height: Optional[float] = None
            segment_counter = 0
            current_ref = list(node.current_position or current_ref)
            for repeat in range(args.repeats):
                for pose_name in sequence:
                    pose = poses[pose_name]
                    target = [float(v) for v in pose['target_rad']]
                    target_height = pose.get('base_com_height_mean_m')
                    target_height = float(target_height) if target_height is not None else None
                    kind, speed = transition_kind_and_speed(source_height, target_height, args)
                    segment_counter += 1
                    transition_segment_id = segment_counter
                    duration = run_transition(
                        node, keys, args,
                        start=current_ref,
                        target=target,
                        segment_id=transition_segment_id,
                        pose_name=pose_name,
                        source_pose_name=source_name,
                        transition_kind=kind,
                        height_m=target_height,
                        speed_limit=speed,
                    )
                    transition_plan.append({
                        'segment_id': transition_segment_id,
                        'repeat': repeat + 1,
                        'source_pose_name': source_name,
                        'target_pose_name': pose_name,
                        'transition_kind': kind,
                        'speed_limit_rad_s': speed,
                        'commanded_duration_sec': duration,
                    })
                    segment_counter += 1
                    run_phase(
                        node, keys, args,
                        target=target,
                        duration_sec=args.settle_sec,
                        segment_id=segment_counter,
                        phase='settle',
                        pose_name=pose_name,
                        source_pose_name=source_name,
                        transition_kind=kind,
                        height_m=target_height,
                        speed_limit=speed,
                    )
                    hold_sec = args.hold_sec
                    if args.deep_pose_name and pose_name == args.deep_pose_name:
                        hold_sec = args.deep_hold_sec
                    segment_counter += 1
                    run_phase(
                        node, keys, args,
                        target=target,
                        duration_sec=hold_sec,
                        segment_id=segment_counter,
                        phase='hold',
                        pose_name=pose_name,
                        source_pose_name=source_name,
                        transition_kind=kind,
                        height_m=target_height,
                        speed_limit=speed,
                    )
                    current_ref = target
                    source_name = pose_name
                    source_height = target_height
    except AbortRequested as exc:
        final_status = f'aborted: {exc}'
        print(f'\nABORT: {exc}', file=sys.stderr)
        ok, message = node.call_trigger(node.hold_current_pose_client, 'hold_current_pose', 3.0)
        print(f'hold_current_pose: success={ok} message={message}', file=sys.stderr)
    finally:
        node.logging_active = False

    pose_rows = pose_summary_rows(node.records, joints)
    pose_level = pose_level_rows(node.records)
    bilateral = bilateral_rows(pose_rows)
    transitions = transition_rows(node.records, joints)
    write_dict_csv(output_dir / 'pose_joint_summary.csv', pose_rows)
    write_dict_csv(output_dir / 'pose_level_summary.csv', pose_level)
    write_dict_csv(output_dir / 'bilateral_pose_summary.csv', bilateral)
    write_dict_csv(output_dir / 'transition_joint_summary.csv', transitions)

    metadata = {
        'schema_version': 1,
        'experiment_type': 'standing_load_characterization',
        'timestamp_utc': datetime.now(timezone.utc).isoformat(),
        'status': final_status,
        'servo_map_path': str(map_path),
        'servo_map_sha256': sha256_file(map_path),
        'pose_library_path': str(library_path.resolve()),
        'pose_library_sha256': sha256_file(library_path),
        'joint_order': [j.name for j in joints],
        'requested_poses': requested,
        'expanded_sequence': sequence,
        'transition_plan': transition_plan,
        'motion_profile': node.diag_values.get('motion_profile', ''),
        'command_rate_hz': args.command_rate_hz,
        'crouch_speed_rad_s': args.crouch_speed_rad_s,
        'stand_return_speed_rad_s': args.stand_return_speed_rad_s,
        'transition_speed_rad_s': args.transition_speed_rad_s,
        'settle_sec': args.settle_sec,
        'hold_sec': args.hold_sec,
        'deep_hold_sec': args.deep_hold_sec,
        'repeats': args.repeats,
        'support_condition': args.support_condition,
        'telemetry_drop_baseline': node.drop_guard_baseline,
        'telemetry_drop_final': node.last_telemetry_dropped_count,
        'notes': [
            'Policy and outer PD controller are excluded from this runner.',
            'ST3215 speed and acceleration remain fixed by the driver motion profile; transition speed parameters shape only the 50 Hz position reference trajectory.',
            'Pose base_com_height_mean_m values are operator-measured metadata, not estimated from servo telemetry.',
            'load_ratio is an effort proxy; current_a is the direct servo current register reading.',
            'Temperature summaries include robust p95 plus raw max because isolated one-cycle temperature outliers have been observed.',
            'Approximate electrical power is voltage_v * abs(current_a) and should not be treated as calibrated total system power.',
        ],
    }
    (output_dir / 'metadata.yaml').write_text(yaml.safe_dump(metadata, sort_keys=False, width=120))

    summary_lines = [
        '# Standing Load Characterization Summary', '',
        f'Status: {final_status}',
        f'Cycles logged: {len(node.records)}',
        f'Pose holds summarized: {len(pose_level)}',
        f'Telemetry drops: {node.drop_guard_baseline} -> {node.last_telemetry_dropped_count}', '',
    ]
    for row in pose_level:
        summary_lines.append(
            f"{row['pose_name']}: height={row['base_com_height_mean_m']} m, "
            f"approx total power median={row['total_servo_power_w_median_approx']:.3f} W, "
            f"IMU omega RMS={row['imu_angular_velocity_rms_rad_s']:.4f} rad/s"
        )
    (output_dir / 'summary.txt').write_text('\n'.join(summary_lines) + '\n')
    final_code = 0 if final_status == 'completed' else 7
    write_manifest(
        output_dir,
        'standing_characterization',
        config_paths={'servo_map': map_path, 'pose_library': Path(args.pose_library).expanduser()},
        result_status=final_status,
        exit_code=final_code,
        extra={'mode': args.mode},
    )
    return final_code


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=['capture_pose', 'evaluate'], required=True)
    parser.add_argument('--servo-map', default='')
    parser.add_argument('--track1-contract', default='')
    parser.add_argument('--pose-library', default=str(DEFAULT_POSE_LIBRARY))
    parser.add_argument('--standing-pose-name', default='normal_stand')
    parser.add_argument('--support-condition', default='feet loaded, overhead fall arrest slack')
    parser.add_argument('--notes', default='')

    # Capture mode
    parser.add_argument('--pose-name', default='')
    parser.add_argument('--base-com-height-mean-m', type=float, default=None)
    parser.add_argument('--capture-window-sec', type=float, default=2.0)
    parser.add_argument('--capture-min-samples', type=int, default=60)
    parser.add_argument('--capture-max-q-std-rad', type=float, default=0.01)
    parser.add_argument('--capture-audit-root', default=str(DEFAULT_CAPTURE_AUDIT_ROOT))
    parser.add_argument('--reenable-torque-hold-after-capture', action='store_true')

    # Evaluation mode
    parser.add_argument('--poses', default='normal_stand,shallow_crouch,medium_crouch,deep_crouch')
    parser.add_argument('--target-base-com-height-m', type=float, default=None,
                        help='Evaluate the captured pose nearest this measured base-COM height instead of --poses.')
    parser.add_argument('--height-match-tolerance-m', type=float, default=0.015)
    parser.add_argument('--return-between-poses', dest='return_between_poses', action='store_true', default=True)
    parser.add_argument('--no-return-between-poses', dest='return_between_poses', action='store_false')
    parser.add_argument('--crouch-speed-rad-s', type=float, default=0.20)
    parser.add_argument('--stand-return-speed-rad-s', type=float, default=0.15)
    parser.add_argument('--transition-speed-rad-s', type=float, default=0.15)
    parser.add_argument('--min-transition-sec', type=float, default=1.0)
    parser.add_argument('--command-rate-hz', type=float, default=50.0)
    parser.add_argument('--settle-sec', type=float, default=5.0)
    parser.add_argument('--hold-sec', type=float, default=20.0)
    parser.add_argument('--deep-pose-name', default='deep_crouch')
    parser.add_argument('--deep-hold-sec', type=float, default=8.0)
    parser.add_argument('--repeats', type=int, default=1)
    parser.add_argument('--output-root', default=str(Path.home() / 'littlegreen_ros2_ws' / 'track2_standing_reports'))

    # Guards
    parser.add_argument('--preflight-timeout-sec', type=float, default=10.0)
    parser.add_argument('--max-feedback-age-ms', type=int, default=250)
    parser.add_argument('--joint-state-timeout-sec', type=float, default=0.5)
    parser.add_argument('--joint-limit-margin-rad', type=float, default=0.01)
    parser.add_argument('--max-current-a', type=float, default=1.5, help='0 disables current guard')
    parser.add_argument('--max-load-ratio', type=float, default=0.90, help='0 disables load guard')
    parser.add_argument('--min-voltage-v', type=float, default=9.0, help='0 disables voltage guard')
    parser.add_argument('--max-temp-c', type=float, default=60.0, help='0 disables temperature guard')
    parser.add_argument('--guard-consecutive-cycles', type=int, default=5)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    try:
        if args.command_rate_hz <= 0 or args.crouch_speed_rad_s <= 0 or args.stand_return_speed_rad_s <= 0:
            raise ValueError('command rate and transition speeds must be positive')
        if args.repeats < 1:
            raise ValueError('--repeats must be >= 1')

        map_path = resolve_servo_map(args.servo_map)
        joints = load_joints(map_path)
        contract_path = resolve_track1_contract(args.track1_contract)
        contract = load_track1_contract(contract_path)
        validate_track1_contract(joints, contract)
    except (FileNotFoundError, ValueError, yaml.YAMLError) as exc:
        print(f'Configuration error: {exc}', file=sys.stderr)
        return 5
    print('Track 1 action-contract audit: PASS')
    print(f'  servo_map: {map_path}')
    print(f'  contract:  {contract_path}')
    print(f"  action_contract_version: {contract.get('action_contract_version')}")
    print(f"  desired_base_com_height_m: {contract.get('desired_base_com_height_m')}")

    output_dir: Optional[Path] = None
    csv_path: Optional[Path] = None
    if args.mode == 'evaluate':
        output_dir = Path(args.output_root).expanduser() / f'{utc_stamp()}_standing_load'
        output_dir.mkdir(parents=True, exist_ok=False)
        csv_path = output_dir / 'timeseries.csv'

    rclpy.init()
    node = StandingLoadNode(joints, csv_path)
    try:
        if args.mode == 'capture_pose':
            return capture_pose_mode(node, joints, map_path, contract_path, contract, args)
        assert output_dir is not None
        return evaluate_mode(node, joints, map_path, args, output_dir)
    except KeyboardInterrupt:
        print('\nInterrupted by Ctrl+C.', file=sys.stderr)
        if args.mode == 'evaluate':
            ok, message = node.call_trigger(node.hold_current_pose_client, 'hold_current_pose', 2.0)
            print(f'hold_current_pose: success={ok} message={message}', file=sys.stderr)
        return 130
    except AbortRequested as exc:
        print(f'Operator abort: {exc}', file=sys.stderr)
        return 7
    except ValueError as exc:
        print(f'Configuration/contract error: {exc}', file=sys.stderr)
        return 5
    except RuntimeError as exc:
        print(f'Hardware/ROS precondition failed: {exc}', file=sys.stderr)
        return 3
    except Exception as exc:
        print(f'Internal standing-characterization error: {exc}', file=sys.stderr)
        return 70
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
