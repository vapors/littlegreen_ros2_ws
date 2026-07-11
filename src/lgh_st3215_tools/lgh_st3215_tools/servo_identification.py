#!/usr/bin/env python3
"""Guarded one-joint identification runner for LittleGreen.

This tool is intentionally conservative. It is designed for supported-robot tests
with the learned policy disconnected. It can drive the ST3215 path directly or
feed manual references through the outer controller for later PD tuning.

Core safety behavior:
- exactly one selected joint moves; all other joints hold the measured anchor pose;
- graph guard refuses a running LittleGreen policy node or competing command publisher;
- requires fresh complete feedback and healthy write-enabled driver diagnostics;
- refuses an active driver pose override or running default-pose ramp;
- bounded offsets, physical-limit margin, arming phrase, and countdown;
- SPACE/q/ESC/Ctrl+C aborts motion into a captured measured-pose hold;
- abort hold keeps publishing until the operator explicitly presses x to exit;
- step sweeps may use a temporary test center while preserving the original safe anchor;
- every step trial captures a fresh settled local baseline before applying its offset;
- successful tests return to the measured safe anchor pose with a slow smooth ramp;
- CSV, YAML metadata, YAML summary, and a text report are generated.

The runner does not identify motor torque constants. Step tests characterize the
combined closed-loop servo + internal controller + mechanism + load + bus timing.
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
import time
import tty
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import rclpy
import yaml
from diagnostic_msgs.msg import DiagnosticArray
from lgh_st3215_driver.msg import ServoTelemetry
from lgh_st3215_tools.dataset_manifest import write_manifest
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float64MultiArray, Int32MultiArray, UInt32MultiArray
from std_srvs.srv import Trigger

try:
    from ament_index_python.packages import get_package_share_directory
except Exception:  # pragma: no cover - only for source-tree execution without ROS env
    get_package_share_directory = None


NUM_JOINTS = 12
GRAVITY_M_S2 = 9.80665
ENCODER_STEP_RAD = 2.0 * math.pi / 4096.0

CSV_FIELDS = [
    'wall_time_ns',
    'telemetry_header_stamp_ns',
    'experiment_id',
    'phase',
    'trial_id',
    'joint_name',
    'servo_id',
    'runner_command_sequence',
    'telemetry_cycle_index',
    'cycle_start_monotonic_ns',
    'cycle_end_monotonic_ns',
    'telemetry_callback_delay_ms',
    'cycle_work_us',
    'feedback_sweep_us',
    'read_start_index',
    'driver_command_sequence',
    'driver_written_command_sequence',
    'driver_command_receipt_monotonic_ns',
    'driver_command_age_ms',
    'write_due',
    'write_attempted',
    'write_ok',
    'sync_write_start_monotonic_ns',
    'sync_write_end_monotonic_ns',
    'sync_write_us',
    'q_ref_runner_rad',
    'q_ref_driver_rad',
    'target_step',
    'configured_speed_steps_s',
    'configured_acceleration_units',
    'q_meas_rad',
    'qdot_meas_rad_s',
    'outer_qdot_cmd_rad_s',
    'position_error_rad',
    'sample_monotonic_ns',
    'feedback_age_ms',
    'raw_position_step',
    'raw_speed',
    'raw_load',
    'load_ratio',
    'voltage_v',
    'temperature_c',
    'servo_status',
    'moving',
    'raw_current',
    'current_a',
    'read_ok',
    'status_error',
    'telemetry_dropped_count',
    'driver_cycle_rate_hz',
    'imu_ang_vel_x',
    'imu_ang_vel_y',
    'imu_ang_vel_z',
    'imu_orientation_x',
    'imu_orientation_y',
    'imu_orientation_z',
    'imu_orientation_w',
]

@dataclass(frozen=True)
class JointConfig:
    name: str
    index: int
    servo_id: int
    servo_sign: int
    center_step: int
    training_default_rad: float
    min_rad: float
    max_rad: float
    speed: int
    acceleration: int


@dataclass
class Point:
    monotonic_ns: int
    phase: str
    trial_id: str
    q_ref: float
    q: float
    qdot: float


@dataclass
class CycleRecord:
    cycle_index: int
    cycle_start_ns: int
    cycle_end_ns: int
    command_sequence: int
    written_command_sequence: int
    command_receipt_ns: int
    command_target_rad: float
    target_rad: float
    write_attempted: bool
    write_ok: bool
    sync_write_start_ns: int
    sync_write_end_ns: int
    sample_ns: int
    q: float
    qdot: float
    raw_position_step: int
    moving: bool
    load_ratio: float
    current_a: float
    voltage_v: float
    temperature_c: int


@dataclass
class Trial:
    trial_id: str
    direction: str
    amplitude_rad: float
    offset_rad: float
    command_monotonic_ns: int
    q_initial_rad: float
    q_target_rad: float
    hold_duration_sec: float


class AbortRequested(RuntimeError):
    """Raised when the operator or a safety guard requests motion abort."""


class TerminalKeys:
    """Small cbreak-mode helper for non-blocking abort keys."""

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
        self.enabled = False
        self.old_termios = None

    def read_key(self) -> Optional[str]:
        if not self.enabled:
            return None
        readable, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not readable:
            return None
        return sys.stdin.read(1)


class IdentificationNode(Node):
    """ROS interface, graph guards, state capture, and CSV streaming."""

    def __init__(
        self,
        *,
        experiment_id: str,
        joints: Sequence[JointConfig],
        test_joint: JointConfig,
        command_path: str,
        direct_topic: str,
        outer_reference_topic: str,
        max_feedback_age_ms: int,
        joint_state_timeout_sec: float,
        allow_nonmax_motion_profile: bool,
        csv_path: Path,
    ) -> None:
        super().__init__('servo_identification_runner')
        self.experiment_id = experiment_id
        self.joints = list(joints)
        self.test_joint = test_joint
        self.command_path = command_path
        self.direct_topic = direct_topic
        self.outer_reference_topic = outer_reference_topic
        self.command_topic = (
            direct_topic if command_path == 'direct' else outer_reference_topic
        )
        self.max_feedback_age_ms = int(max_feedback_age_ms)
        self.joint_state_timeout_sec = float(joint_state_timeout_sec)
        self.allow_nonmax_motion_profile = bool(allow_nonmax_motion_profile)

        command_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=(
                QoSReliabilityPolicy.BEST_EFFORT
                if command_path == 'direct'
                else QoSReliabilityPolicy.RELIABLE
            ),
        )
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )

        self.command_pub = self.create_publisher(
            Float64MultiArray, self.command_topic, command_qos
        )
        self.create_subscription(
            JointState, '/joint_states', self._joint_state_cb, sensor_qos
        )
        self.create_subscription(
            ServoTelemetry,
            '/st3215_driver/telemetry',
            self._telemetry_cb,
            sensor_qos,
        )
        self.create_subscription(
            UInt32MultiArray,
            '/joint_feedback_age_ms',
            self._feedback_age_cb,
            sensor_qos,
        )
        self.create_subscription(
            Int32MultiArray,
            '/st3215_driver/raw_position_steps',
            self._raw_position_cb,
            sensor_qos,
        )
        self.create_subscription(
            Int32MultiArray,
            '/st3215_driver/raw_speed',
            self._raw_speed_cb,
            sensor_qos,
        )
        self.create_subscription(
            DiagnosticArray,
            '/st3215_driver/diagnostics',
            self._diagnostics_cb,
            reliable_qos,
        )
        self.create_subscription(Imu, '/imu/data', self._imu_cb, sensor_qos)
        self.create_subscription(
            Float64MultiArray,
            '/outer_controller/velocity_command',
            self._outer_velocity_cb,
            sensor_qos,
        )
        self.create_subscription(
            Float64MultiArray,
            '/outer_controller/position_error',
            self._outer_error_cb,
            sensor_qos,
        )
        self.hold_current_pose_client = self.create_client(
            Trigger, '/st3215_driver/hold_current_pose'
        )

        self.current_position: Optional[List[float]] = None
        self.current_velocity: Optional[List[float]] = None
        self.last_joint_state_monotonic: Optional[float] = None
        self.feedback_age_ms: Optional[List[int]] = None
        self.raw_position: Optional[List[int]] = None
        self.raw_speed: Optional[List[int]] = None
        self.outer_velocity: Optional[List[float]] = None
        self.outer_error: Optional[List[float]] = None
        self.imu: Optional[Tuple[float, float, float, float, float, float, float]] = None
        self.last_telemetry_monotonic: Optional[float] = None
        self.last_telemetry_cycle_index: int = 0
        self.last_telemetry_dropped_count: int = 0
        self.telemetry_drop_guard_baseline: Optional[int] = None
        # On the Linux Orange Pi target, Python time.monotonic_ns() and C++
        # steady_clock are expected to share CLOCK_MONOTONIC. Verify that at
        # runtime before using cross-language latency decomposition.
        self.telemetry_clock_delta_ns: Optional[int] = None

        self.driver_diag_values: Dict[str, str] = {}
        self.driver_diag_level: Optional[int] = None
        self.driver_diag_message: str = ''
        self.last_diag_monotonic: Optional[float] = None

        self.reference = [0.0] * NUM_JOINTS
        self.phase = 'preflight'
        self.trial_id = ''
        self.command_sequence = 0
        self.points: List[Point] = []
        self.cycles: List[CycleRecord] = []
        self.logging_active = False

        self.csv_path = csv_path
        self.csv_file = csv_path.open('w', newline='', buffering=1)
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=CSV_FIELDS)
        self.csv_writer.writeheader()
        self._rows_written = 0

    def close(self) -> None:
        if not self.csv_file.closed:
            self.csv_file.flush()
            os.fsync(self.csv_file.fileno())
            self.csv_file.close()

    @staticmethod
    def _header_stamp_ns(msg: ServoTelemetry) -> int:
        return int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)

    def _joint_state_cb(self, msg: JointState) -> None:
        if len(msg.position) < NUM_JOINTS or len(msg.velocity) < NUM_JOINTS:
            return
        q = [float(v) for v in msg.position[:NUM_JOINTS]]
        qdot = [float(v) for v in msg.velocity[:NUM_JOINTS]]
        if not all(math.isfinite(v) for v in q + qdot):
            return
        self.current_position = q
        self.current_velocity = qdot
        self.last_joint_state_monotonic = time.monotonic()

    def _telemetry_cb(self, msg: ServoTelemetry) -> None:
        if len(msg.q_meas_rad) < NUM_JOINTS or len(msg.qdot_meas_rad_s) < NUM_JOINTS:
            return

        callback_monotonic_ns = time.monotonic_ns()
        self.last_telemetry_monotonic = callback_monotonic_ns / 1e9
        cycle_end_ns = int(msg.cycle_end_monotonic_ns)
        if cycle_end_ns > 0:
            self.telemetry_clock_delta_ns = callback_monotonic_ns - cycle_end_ns
        cycle_index = int(msg.cycle_index)
        self.last_telemetry_cycle_index = max(self.last_telemetry_cycle_index, cycle_index)
        self.last_telemetry_dropped_count = int(msg.telemetry_dropped_count)

        if not self.logging_active:
            return
        if self.cycles and cycle_index <= self.cycles[-1].cycle_index:
            return

        idx = self.test_joint.index
        sample_ns = int(msg.sample_monotonic_ns[idx])
        if sample_ns <= 0:
            sample_ns = int(msg.cycle_end_monotonic_ns)

        q = float(msg.q_meas_rad[idx])
        qdot = float(msg.qdot_meas_rad_s[idx])
        q_ref_driver = (
            float(msg.target_rad_from_steps[idx])
            if bool(msg.target_valid)
            else float(msg.command_target_rad[idx])
        )

        self.points.append(
            Point(
                monotonic_ns=sample_ns,
                phase=self.phase,
                trial_id=self.trial_id,
                q_ref=q_ref_driver,
                q=q,
                qdot=qdot,
            )
        )
        self.cycles.append(
            CycleRecord(
                cycle_index=cycle_index,
                cycle_start_ns=int(msg.cycle_start_monotonic_ns),
                cycle_end_ns=int(msg.cycle_end_monotonic_ns),
                command_sequence=int(msg.command_sequence),
                written_command_sequence=int(msg.written_command_sequence),
                command_receipt_ns=int(msg.command_receipt_monotonic_ns),
                command_target_rad=float(msg.command_target_rad[idx]),
                target_rad=q_ref_driver,
                write_attempted=bool(msg.write_attempted),
                write_ok=bool(msg.write_ok),
                sync_write_start_ns=int(msg.sync_write_start_monotonic_ns),
                sync_write_end_ns=int(msg.sync_write_end_monotonic_ns),
                sample_ns=sample_ns,
                q=q,
                qdot=qdot,
                raw_position_step=int(msg.raw_position_steps[idx]),
                moving=bool(msg.moving[idx]),
                load_ratio=float(msg.load_ratio[idx]),
                current_a=float(msg.current_a[idx]),
                voltage_v=float(msg.voltage_v[idx]),
                temperature_c=int(msg.temperature_c[idx]),
            )
        )

        outer_qdot = ''
        if self.outer_velocity is not None and len(self.outer_velocity) > idx:
            outer_qdot = self.outer_velocity[idx]
        outer_error = ''
        if self.outer_error is not None and len(self.outer_error) > idx:
            outer_error = self.outer_error[idx]

        imu_values: Sequence[object] = [''] * 7
        if self.imu is not None:
            imu_values = self.imu

        row = {
            'wall_time_ns': time.time_ns(),
            'telemetry_header_stamp_ns': self._header_stamp_ns(msg),
            'experiment_id': self.experiment_id,
            'phase': self.phase,
            'trial_id': self.trial_id,
            'joint_name': self.test_joint.name,
            'servo_id': self.test_joint.servo_id,
            'runner_command_sequence': self.command_sequence,
            'telemetry_cycle_index': cycle_index,
            'cycle_start_monotonic_ns': int(msg.cycle_start_monotonic_ns),
            'cycle_end_monotonic_ns': int(msg.cycle_end_monotonic_ns),
            'telemetry_callback_delay_ms': (
                self.telemetry_clock_delta_ns / 1e6
                if self.telemetry_clock_delta_ns is not None else ''
            ),
            'cycle_work_us': float(msg.cycle_work_us),
            'feedback_sweep_us': float(msg.feedback_sweep_us),
            'read_start_index': int(msg.read_start_index),
            'driver_command_sequence': int(msg.command_sequence),
            'driver_written_command_sequence': int(msg.written_command_sequence),
            'driver_command_receipt_monotonic_ns': int(msg.command_receipt_monotonic_ns),
            'driver_command_age_ms': float(msg.command_age_ms),
            'write_due': int(bool(msg.write_due)),
            'write_attempted': int(bool(msg.write_attempted)),
            'write_ok': int(bool(msg.write_ok)),
            'sync_write_start_monotonic_ns': int(msg.sync_write_start_monotonic_ns),
            'sync_write_end_monotonic_ns': int(msg.sync_write_end_monotonic_ns),
            'sync_write_us': float(msg.sync_write_us),
            'q_ref_runner_rad': float(self.reference[idx]),
            'q_ref_driver_rad': q_ref_driver,
            'target_step': int(msg.target_steps[idx]),
            'configured_speed_steps_s': int(msg.configured_speed_steps_s[idx]),
            'configured_acceleration_units': int(msg.configured_acceleration_units[idx]),
            'q_meas_rad': q,
            'qdot_meas_rad_s': qdot,
            'outer_qdot_cmd_rad_s': outer_qdot,
            'position_error_rad': outer_error,
            'sample_monotonic_ns': sample_ns,
            'feedback_age_ms': float(msg.feedback_age_ms_at_cycle_end[idx]),
            'raw_position_step': int(msg.raw_position_steps[idx]),
            'raw_speed': int(msg.raw_speed[idx]),
            'raw_load': int(msg.raw_load[idx]),
            'load_ratio': float(msg.load_ratio[idx]),
            'voltage_v': float(msg.voltage_v[idx]),
            'temperature_c': int(msg.temperature_c[idx]),
            'servo_status': int(msg.servo_status[idx]),
            'moving': int(bool(msg.moving[idx])),
            'raw_current': int(msg.raw_current[idx]),
            'current_a': float(msg.current_a[idx]),
            'read_ok': int(bool(msg.read_ok[idx])),
            'status_error': int(msg.status_error[idx]),
            'telemetry_dropped_count': int(msg.telemetry_dropped_count),
            'driver_cycle_rate_hz': self.driver_diag_values.get('cycle_rate_hz', ''),
            'imu_ang_vel_x': imu_values[0],
            'imu_ang_vel_y': imu_values[1],
            'imu_ang_vel_z': imu_values[2],
            'imu_orientation_x': imu_values[3],
            'imu_orientation_y': imu_values[4],
            'imu_orientation_z': imu_values[5],
            'imu_orientation_w': imu_values[6],
        }
        self.csv_writer.writerow(row)
        self._rows_written += 1
        if self._rows_written % 100 == 0:
            self.csv_file.flush()

    def _feedback_age_cb(self, msg: UInt32MultiArray) -> None:
        if len(msg.data) >= NUM_JOINTS:
            self.feedback_age_ms = [int(v) for v in msg.data[:NUM_JOINTS]]

    def _raw_position_cb(self, msg: Int32MultiArray) -> None:
        if len(msg.data) >= NUM_JOINTS:
            self.raw_position = [int(v) for v in msg.data[:NUM_JOINTS]]

    def _raw_speed_cb(self, msg: Int32MultiArray) -> None:
        if len(msg.data) >= NUM_JOINTS:
            self.raw_speed = [int(v) for v in msg.data[:NUM_JOINTS]]

    def _outer_velocity_cb(self, msg: Float64MultiArray) -> None:
        if len(msg.data) >= NUM_JOINTS:
            self.outer_velocity = [float(v) for v in msg.data[:NUM_JOINTS]]

    def _outer_error_cb(self, msg: Float64MultiArray) -> None:
        if len(msg.data) >= NUM_JOINTS:
            self.outer_error = [float(v) for v in msg.data[:NUM_JOINTS]]

    def _imu_cb(self, msg: Imu) -> None:
        self.imu = (
            float(msg.angular_velocity.x),
            float(msg.angular_velocity.y),
            float(msg.angular_velocity.z),
            float(msg.orientation.x),
            float(msg.orientation.y),
            float(msg.orientation.z),
            float(msg.orientation.w),
        )

    def _diagnostics_cb(self, msg: DiagnosticArray) -> None:
        if not msg.status:
            return
        status = msg.status[0]
        level = status.level
        if isinstance(level, (bytes, bytearray)):
            if len(level) != 1:
                raise ValueError(f'unexpected DiagnosticStatus.level byte length: {len(level)}')
            self.driver_diag_level = level[0]
        else:
            self.driver_diag_level = int(level)
        self.driver_diag_message = str(status.message)
        self.driver_diag_values = {str(kv.key): str(kv.value) for kv in status.values}
        self.last_diag_monotonic = time.monotonic()

    def publish_reference(self, reference: Sequence[float]) -> None:
        if len(reference) != NUM_JOINTS:
            raise ValueError('reference must contain exactly 12 values')
        msg = Float64MultiArray()
        msg.data = [float(v) for v in reference]
        self.reference = list(msg.data)
        self.command_sequence += 1
        self.command_pub.publish(msg)

    def latch_driver_current_pose_hold(self, timeout_sec: float = 2.0) -> Tuple[bool, str]:
        if not self.hold_current_pose_client.wait_for_service(timeout_sec=timeout_sec):
            return False, 'hold_current_pose service unavailable'
        future = self.hold_current_pose_client.call_async(Trigger.Request())
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done():
            return False, 'hold_current_pose service timed out'
        try:
            response = future.result()
        except Exception as exc:  # pragma: no cover - runtime transport path
            return False, f'hold_current_pose service error: {exc}'
        return bool(response.success), str(response.message)

    def feedback_ready(self) -> bool:
        if self.current_position is None or self.current_velocity is None:
            return False
        if self.feedback_age_ms is None or len(self.feedback_age_ms) < NUM_JOINTS:
            return False
        if self.last_joint_state_monotonic is None:
            return False
        if time.monotonic() - self.last_joint_state_monotonic > self.joint_state_timeout_sec:
            return False
        if max(self.feedback_age_ms) > self.max_feedback_age_ms:
            return False
        return True

    def driver_ready_for_motion(self) -> Tuple[bool, str]:
        if self.last_diag_monotonic is None:
            return False, 'driver diagnostics not received'
        if time.monotonic() - self.last_diag_monotonic > 2.5:
            return False, 'driver diagnostics stale'
        if self.last_telemetry_monotonic is None:
            return False, 'cycle telemetry not received'
        if time.monotonic() - self.last_telemetry_monotonic > 0.5:
            return False, 'cycle telemetry stale'
        if self.telemetry_clock_delta_ns is None:
            return False, 'telemetry monotonic clock comparison unavailable'
        if abs(self.telemetry_clock_delta_ns) > 60_000_000_000:
            return False, (
                'runner and driver monotonic clocks do not appear comparable; '
                f'delta={self.telemetry_clock_delta_ns / 1e9:.3f}s'
            )
        values = self.driver_diag_values
        if values.get('writes_enabled', 'false').lower() != 'true':
            return False, 'driver writes_enabled is not true'
        if not self.allow_nonmax_motion_profile:
            profile = values.get('motion_profile', '')
            if profile != 'max_envelope_fixed_0_0':
                return False, (
                    'driver motion_profile is not max_envelope_fixed_0_0; '
                    f'got {profile!r}. Use --allow-nonmax-motion-profile only for intentional profile studies.'
                )
        if values.get('feedback_ready', 'false').lower() != 'true':
            return False, 'driver feedback_ready is not true'
        if values.get('pose_move_running', 'false').lower() == 'true':
            return False, 'driver default-pose ramp is running'
        if values.get('pose_override_active', 'false').lower() == 'true':
            return False, 'driver pose override is active; release it explicitly before identification'
        if not self.hold_current_pose_client.service_is_ready():
            return False, 'driver hold_current_pose abort-latch service is not ready'
        if not self.feedback_ready():
            return False, 'joint feedback is missing or stale'
        return True, 'ready'

    def _full_node_name(self, namespace: str, name: str) -> str:
        ns = namespace.rstrip('/')
        if not ns:
            return '/' + name
        return ns + '/' + name

    def external_publishers(self, topic: str) -> List[str]:
        result = []
        for info in self.get_publishers_info_by_topic(topic):
            if info.node_name == self.get_name():
                continue
            result.append(self._full_node_name(info.node_namespace, info.node_name))
        return sorted(set(result))

    def external_subscribers(self, topic: str) -> List[str]:
        result = []
        for info in self.get_subscriptions_info_by_topic(topic):
            if info.node_name == self.get_name():
                continue
            result.append(self._full_node_name(info.node_namespace, info.node_name))
        return sorted(set(result))

    def graph_guard(self) -> Tuple[bool, str, Dict[str, object]]:
        nodes = [self._full_node_name(ns, name) for name, ns in self.get_node_names_and_namespaces()]
        policy_nodes = [n for n in nodes if n.endswith('/littlegreen_biped_node')]
        details: Dict[str, object] = {
            'policy_nodes': policy_nodes,
            'direct_topic_publishers': self.external_publishers(self.direct_topic),
            'outer_reference_publishers': self.external_publishers(self.outer_reference_topic),
            'desired_joint_position_publishers': self.external_publishers('/desired_joint_position'),
        }
        if policy_nodes:
            return False, f'policy node is running: {policy_nodes}', details

        if self.command_path == 'direct':
            conflicts = details['direct_topic_publishers']
            if conflicts:
                return False, f'competing direct servo command publisher(s): {conflicts}', details
            policy_publishers = sorted(
                set(details['outer_reference_publishers'])
                | set(details['desired_joint_position_publishers'])
            )
            if policy_publishers:
                return False, f'policy/reference publisher(s) still active: {policy_publishers}', details
        else:
            conflicts = details['outer_reference_publishers']
            if conflicts:
                return False, f'competing outer reference publisher(s): {conflicts}', details
            subscribers = self.external_subscribers(self.outer_reference_topic)
            details['outer_reference_subscribers'] = subscribers
            if not any(name.endswith('/pd_controller_node') for name in subscribers):
                return False, 'outer path selected but pd_controller_node is not subscribed to the reference topic', details
            direct_publishers = details['direct_topic_publishers']
            if not any(name.endswith('/pd_controller_node') for name in direct_publishers):
                return False, 'outer path selected but pd_controller_node is not publishing the servo target', details
            unknown_direct = [
                name for name in direct_publishers if not name.endswith('/pd_controller_node')
            ]
            if unknown_direct:
                return False, f'unexpected direct servo publisher(s): {unknown_direct}', details

        return True, 'graph guard passed', details

    def continuous_safety_guard(self) -> None:
        if (
            self.telemetry_drop_guard_baseline is not None
            and self.last_telemetry_dropped_count > self.telemetry_drop_guard_baseline
        ):
            raise AbortRequested(
                'telemetry queue dropped cycle snapshots during the armed test: '
                f'{self.telemetry_drop_guard_baseline} -> {self.last_telemetry_dropped_count}'
            )
        ready, reason = self.driver_ready_for_motion()
        if not ready:
            raise AbortRequested(f'driver/feedback guard failed: {reason}')
        ok, reason, _ = self.graph_guard()
        if not ok:
            raise AbortRequested(f'graph guard failed: {reason}')


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open('rb') as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def resolve_servo_map(configured: str) -> Path:
    candidates: List[Path] = []
    if configured:
        candidates.append(Path(configured).expanduser())
    if get_package_share_directory is not None:
        try:
            candidates.append(
                Path(get_package_share_directory('lgh_st3215_driver'))
                / 'config'
                / 'servo_map.yaml'
            )
        except Exception:
            pass
    candidates.append(
        Path.home()
        / 'littlegreen_ros2_ws'
        / 'src'
        / 'lgh_st3215_driver'
        / 'config'
        / 'servo_map.yaml'
    )
    for path in candidates:
        if path.is_file():
            return path.resolve()
    raise FileNotFoundError('servo_map.yaml not found; tried: ' + ', '.join(map(str, candidates)))


def load_joints(path: Path) -> List[JointConfig]:
    root = yaml.safe_load(path.read_text())
    nodes = root.get('joints', []) if isinstance(root, dict) else []
    if len(nodes) != NUM_JOINTS:
        raise ValueError(f'expected 12 joints in {path}, found {len(nodes)}')
    nodes = sorted(
        nodes,
        key=lambda node: int(node.get('policy_index', node.get('policy_action_index', 0))),
    )
    joints = []
    for index, node in enumerate(nodes):
        policy_index = int(node.get('policy_index', node.get('policy_action_index', index)))
        if policy_index != index:
            raise ValueError('servo map policy indices must be contiguous 0..11')
        joints.append(
            JointConfig(
                name=str(node['name']),
                index=index,
                servo_id=int(node['servo_id']),
                servo_sign=int(node.get('servo_sign', 1)),
                center_step=int(node.get('center_step', node.get('servo_center_step', 2048))),
                training_default_rad=float(node.get('training_default_rad', node.get('default_joint_rad', 0.0))),
                min_rad=float(node.get('min_rad', node.get('limit_lower_rad'))),
                max_rad=float(node.get('max_rad', node.get('limit_upper_rad'))),
                speed=int(node.get('speed', 0)),
                acceleration=int(node.get('acceleration', 0)),
            )
        )
    return joints


def parse_joint_selector(selector: str, joints: Sequence[JointConfig]) -> JointConfig:
    by_name = {joint.name: joint for joint in joints}
    if selector in by_name:
        return by_name[selector]
    try:
        index = int(selector)
    except ValueError as exc:
        raise ValueError(
            f'unknown joint {selector!r}; use canonical name or index 0..11'
        ) from exc
    if index < 0 or index >= len(joints):
        raise ValueError('joint index must be in 0..11')
    return joints[index]


def median_or_nan(values: Iterable[float]) -> float:
    seq = [float(v) for v in values if math.isfinite(float(v))]
    return statistics.median(seq) if seq else math.nan


def percentile(values: Sequence[float], fraction: float) -> float:
    if not values:
        return math.nan
    ordered = sorted(float(v) for v in values)
    x = max(0.0, min(1.0, fraction)) * (len(ordered) - 1)
    lo = int(math.floor(x))
    hi = int(math.ceil(x))
    if lo == hi:
        return ordered[lo]
    u = x - lo
    return ordered[lo] * (1.0 - u) + ordered[hi] * u


def first_crossing_ms(
    points: Sequence[Point],
    command_ns: int,
    predicate,
) -> Optional[float]:
    for p in points:
        if p.monotonic_ns < command_ns:
            continue
        if predicate(p):
            return (p.monotonic_ns - command_ns) / 1e6
    return None


def settling_time_ms(
    points: Sequence[Point],
    command_ns: int,
    target: float,
    tolerance: float,
) -> Optional[float]:
    post = [p for p in points if p.monotonic_ns >= command_ns]
    if not post:
        return None
    for i, p in enumerate(post):
        if abs(p.q - target) <= tolerance and all(
            abs(q.q - target) <= tolerance for q in post[i:]
        ):
            return (p.monotonic_ns - command_ns) / 1e6
    return None


def analyze_trial(
    trial: Trial,
    points: Sequence[Point],
    cycles: Sequence[CycleRecord],
    motion_threshold_rad: float,
) -> Dict[str, object]:
    data = [
        p for p in points
        if p.trial_id == trial.trial_id
        and p.monotonic_ns >= trial.command_monotonic_ns
    ]
    result: Dict[str, object] = asdict(trial)
    result['sample_count'] = len(data)
    if len(data) < 3:
        result['analysis_status'] = 'insufficient_samples'
        return result

    q0 = trial.q_initial_rad
    target = trial.q_target_rad
    delta = target - q0
    result['actual_command_delta_rad'] = delta
    result['requested_vs_actual_delta_error_rad'] = delta - trial.offset_rad
    if abs(delta) < 1e-9:
        result['analysis_status'] = 'zero_step'
        return result

    progress = [(p.q - q0) / delta for p in data]
    steady_window = data[max(0, int(len(data) * 0.8)):]
    steady_q = median_or_nan(p.q for p in steady_window)
    steady_error = target - steady_q
    achieved_delta = steady_q - q0
    peak_velocity = max(abs(p.qdot) for p in data)

    sustained_motion_ms = first_crossing_ms(
        data,
        trial.command_monotonic_ns,
        lambda p: abs(p.q - q0) >= motion_threshold_rad,
    )

    # Command-relative dynamic metrics remain useful when static gain is high enough.
    command_rise_10_ms = first_crossing_ms(
        data, trial.command_monotonic_ns, lambda p: (p.q - q0) / delta >= 0.10
    )
    command_rise_90_ms = first_crossing_ms(
        data, trial.command_monotonic_ns, lambda p: (p.q - q0) / delta >= 0.90
    )
    command_rise_time_ms = None
    if (
        command_rise_10_ms is not None
        and command_rise_90_ms is not None
        and command_rise_90_ms >= command_rise_10_ms
    ):
        command_rise_time_ms = command_rise_90_ms - command_rise_10_ms

    command_tau63_ms = first_crossing_ms(
        data, trial.command_monotonic_ns, lambda p: (p.q - q0) / delta >= 0.632
    )
    overshoot_ratio = max(0.0, max(progress) - 1.0)
    settling_2pct = settling_time_ms(
        data, trial.command_monotonic_ns, target, 0.02 * abs(delta)
    )
    settling_0p01 = settling_time_ms(
        data, trial.command_monotonic_ns, target, 0.01
    )
    static_gain = achieved_delta / delta if math.isfinite(steady_q) else math.nan

    # Achieved-response-relative metrics remain defined even when static gain < 0.9.
    achieved_rise_time_ms = None
    achieved_tau63_ms = None
    achieved_settling_1_count_ms = None
    if math.isfinite(achieved_delta) and abs(achieved_delta) >= ENCODER_STEP_RAD:
        achieved_rise_10_ms = first_crossing_ms(
            data,
            trial.command_monotonic_ns,
            lambda p: (p.q - q0) / achieved_delta >= 0.10,
        )
        achieved_rise_90_ms = first_crossing_ms(
            data,
            trial.command_monotonic_ns,
            lambda p: (p.q - q0) / achieved_delta >= 0.90,
        )
        if (
            achieved_rise_10_ms is not None
            and achieved_rise_90_ms is not None
            and achieved_rise_90_ms >= achieved_rise_10_ms
        ):
            achieved_rise_time_ms = achieved_rise_90_ms - achieved_rise_10_ms
        achieved_tau63_ms = first_crossing_ms(
            data,
            trial.command_monotonic_ns,
            lambda p: (p.q - q0) / achieved_delta >= 0.632,
        )
        achieved_settling_1_count_ms = settling_time_ms(
            data,
            trial.command_monotonic_ns,
            steady_q,
            ENCODER_STEP_RAD,
        )

    damping_ratio = None
    natural_frequency = None
    if 0.0 < overshoot_ratio < 1.0:
        ln_mp = math.log(overshoot_ratio)
        damping_ratio = -ln_mp / math.sqrt(math.pi * math.pi + ln_mp * ln_mp)
        if settling_2pct is not None and settling_2pct > 0.0 and damping_ratio > 0.0:
            natural_frequency = 4.0 / (damping_ratio * (settling_2pct / 1000.0))

    motion_point = next(
        (
            p for p in data
            if p.monotonic_ns >= trial.command_monotonic_ns
            and abs(p.q - q0) >= motion_threshold_rad
        ),
        None,
    )
    target_tolerance = max(0.003, 0.02 * abs(delta))
    receipt_candidates = [
        c for c in cycles
        if c.command_receipt_ns >= trial.command_monotonic_ns
        and abs(c.command_target_rad - target) <= target_tolerance
    ]
    first_receipt = (
        min(receipt_candidates, key=lambda c: c.command_receipt_ns)
        if receipt_candidates else None
    )
    write_candidates = [
        c for c in cycles
        if c.write_attempted and c.write_ok
        and c.sync_write_start_ns >= trial.command_monotonic_ns
        and abs(c.target_rad - target) <= target_tolerance
    ]
    first_write = (
        min(write_candidates, key=lambda c: c.sync_write_start_ns)
        if write_candidates else None
    )

    publish_to_driver_receipt_ms = None
    driver_receipt_to_write_start_ms = None
    first_write_to_motion_ms = None
    write_end_to_motion_ms = None
    first_sync_write_duration_ms = None
    write_end_to_moving_flag_ms = None
    write_end_to_first_encoder_count_ms = None
    first_encoder_step_delta = None

    if first_receipt is not None:
        publish_to_driver_receipt_ms = (
            first_receipt.command_receipt_ns - trial.command_monotonic_ns
        ) / 1e6
    if first_receipt is not None and first_write is not None:
        driver_receipt_to_write_start_ms = (
            first_write.sync_write_start_ns - first_receipt.command_receipt_ns
        ) / 1e6
    if first_write is not None:
        first_sync_write_duration_ms = (
            first_write.sync_write_end_ns - first_write.sync_write_start_ns
        ) / 1e6
    if first_write is not None and motion_point is not None:
        first_write_to_motion_ms = (
            motion_point.monotonic_ns - first_write.sync_write_start_ns
        ) / 1e6
        write_end_to_motion_ms = (
            motion_point.monotonic_ns - first_write.sync_write_end_ns
        ) / 1e6

    # Distinguish controller-active, first encoder movement, and sustained motion.
    if first_write is not None:
        pre_write = [
            c for c in cycles
            if c.sample_ns <= first_write.sync_write_end_ns
        ]
        baseline_step = (
            max(pre_write, key=lambda c: c.sample_ns).raw_position_step
            if pre_write else None
        )
        post_write_cycles = sorted(
            [c for c in cycles if c.sample_ns >= first_write.sync_write_end_ns],
            key=lambda c: c.sample_ns,
        )
        moving_cycle = next((c for c in post_write_cycles if c.moving), None)
        if moving_cycle is not None:
            write_end_to_moving_flag_ms = (
                moving_cycle.sample_ns - first_write.sync_write_end_ns
            ) / 1e6
        if baseline_step is not None:
            encoder_cycle = next(
                (c for c in post_write_cycles if c.raw_position_step != baseline_step),
                None,
            )
            if encoder_cycle is not None:
                write_end_to_first_encoder_count_ms = (
                    encoder_cycle.sample_ns - first_write.sync_write_end_ns
                ) / 1e6
                first_encoder_step_delta = encoder_cycle.raw_position_step - baseline_step

    trial_end_ns = trial.command_monotonic_ns + int(trial.hold_duration_sec * 1e9)
    trial_cycles = [
        c for c in cycles
        if trial.command_monotonic_ns <= c.sample_ns <= trial_end_ns
    ]
    peak_abs_load_ratio = max(
        (abs(c.load_ratio) for c in trial_cycles if math.isfinite(c.load_ratio)),
        default=None,
    )
    peak_abs_current_a = max(
        (abs(c.current_a) for c in trial_cycles if math.isfinite(c.current_a)),
        default=None,
    )
    median_voltage_v = median_or_nan(
        c.voltage_v for c in trial_cycles if math.isfinite(c.voltage_v)
    ) if trial_cycles else None
    max_temperature_c = max(
        (c.temperature_c for c in trial_cycles),
        default=None,
    )

    result.update(
        {
            'analysis_status': 'ok',
            'reference_publish_to_motion_lag_ms': sustained_motion_ms,
            'reference_publish_to_driver_receipt_ms': publish_to_driver_receipt_ms,
            'driver_receipt_to_sync_write_start_ms': driver_receipt_to_write_start_ms,
            'first_sync_write_duration_ms': first_sync_write_duration_ms,
            'first_sync_write_start_to_motion_ms': first_write_to_motion_ms,
            'first_sync_write_end_to_motion_ms': write_end_to_motion_ms,
            'first_sync_write_end_to_moving_flag_ms': write_end_to_moving_flag_ms,
            'first_sync_write_end_to_first_encoder_count_ms': write_end_to_first_encoder_count_ms,
            'first_encoder_step_delta': first_encoder_step_delta,
            'rise_time_10_90_ms': command_rise_time_ms,
            'time_to_63p2_ms': command_tau63_ms,
            'achieved_response_rise_time_10_90_ms': achieved_rise_time_ms,
            'achieved_response_time_to_63p2_ms': achieved_tau63_ms,
            'achieved_response_settling_1_encoder_count_ms': achieved_settling_1_count_ms,
            'encoder_step_rad': ENCODER_STEP_RAD,
            'settling_time_2pct_ms': settling_2pct,
            'settling_time_0p01_rad_ms': settling_0p01,
            'overshoot_ratio': overshoot_ratio,
            'overshoot_percent': overshoot_ratio * 100.0,
            'steady_state_q_rad': steady_q,
            'achieved_displacement_rad': achieved_delta,
            'steady_state_error_rad': steady_error,
            'peak_velocity_rad_s': peak_velocity,
            'static_gain_ratio': static_gain,
            'peak_abs_load_ratio': peak_abs_load_ratio,
            'peak_abs_current_a': peak_abs_current_a,
            'median_voltage_v': median_voltage_v,
            'max_temperature_c': max_temperature_c,
            'damping_ratio_from_overshoot': damping_ratio,
            'natural_frequency_rad_s_est': natural_frequency,
        }
    )
    return result


def bounded_offsets(args) -> List[Tuple[str, float, float]]:
    if args.mode == 'step':
        amplitudes = [float(args.amplitude_rad)]
    elif args.mode == 'step_sweep':
        amplitudes = [float(v) for v in args.amplitudes_rad]
    else:
        return []

    directions = ['positive', 'negative'] if args.direction == 'both' else [args.direction]
    result = []
    for amplitude in amplitudes:
        if amplitude <= 0.0:
            raise ValueError('step amplitudes must be positive')
        if amplitude > args.max_test_offset_rad + 1e-12:
            raise ValueError(
                f'amplitude {amplitude:.6f} rad exceeds max_test_offset_rad={args.max_test_offset_rad:.6f}'
            )
        for direction in directions:
            offset = amplitude if direction == 'positive' else -amplitude
            result.append((direction, amplitude, offset))
    return result


def validate_offset_target(
    joint: JointConfig,
    anchor_rad: float,
    offset_rad: float,
    margin_rad: float,
) -> float:
    if abs(offset_rad) < 1e-12:
        return anchor_rad
    target = anchor_rad + offset_rad
    lo = joint.min_rad + margin_rad
    hi = joint.max_rad - margin_rad
    if target < lo or target > hi:
        raise ValueError(
            f'{joint.name}: target {target:.6f} rad from anchor {anchor_rad:.6f} + offset {offset_rad:.6f} '
            f'is outside guarded range [{lo:.6f}, {hi:.6f}]'
        )
    return target


def make_reference(anchor: Sequence[float], index: int, target: float) -> List[float]:
    result = list(anchor)
    result[index] = float(target)
    return result


def spin_and_sleep_until(node: Node, deadline: float) -> None:
    while rclpy.ok():
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            return
        rclpy.spin_once(node, timeout_sec=min(0.005, remaining))


def run_reference_phase(
    node: IdentificationNode,
    keys: TerminalKeys,
    reference: Sequence[float],
    duration_sec: float,
    phase: str,
    trial_id: str = '',
    guard_period_sec: float = 0.5,
) -> int:
    node.phase = phase
    node.trial_id = trial_id
    period = 1.0 / 50.0
    next_tick = time.monotonic()
    end_time = next_tick + max(0.0, duration_sec)
    next_guard = time.monotonic()
    first_publish_ns = 0

    while rclpy.ok() and time.monotonic() < end_time:
        key = keys.read_key()
        if key in (' ', 'q', 'Q', '\x1b'):
            raise AbortRequested(f'operator abort key {repr(key)}')

        now = time.monotonic()
        if now >= next_guard:
            node.continuous_safety_guard()
            next_guard = now + guard_period_sec

        if now >= next_tick:
            if first_publish_ns == 0:
                first_publish_ns = time.monotonic_ns()
            node.publish_reference(reference)
            next_tick += period
        spin_and_sleep_until(node, min(next_tick, end_time))

    return first_publish_ns


def smooth_transition(
    node: IdentificationNode,
    keys: TerminalKeys,
    start_reference: Sequence[float],
    destination: Sequence[float],
    duration_sec: float,
    *,
    phase: str,
) -> None:
    """Move all commanded joints smoothly between two already-validated references."""
    node.phase = phase
    node.trial_id = ''
    period = 1.0 / 50.0
    steps = max(1, int(math.ceil(max(duration_sec, period) / period)))
    start = list(start_reference)
    goal = list(destination)
    for step in range(1, steps + 1):
        key = keys.read_key()
        if key in (' ', 'q', 'Q', '\x1b'):
            raise AbortRequested(f'operator abort key {repr(key)} during {phase}')
        if step % 25 == 1:
            node.continuous_safety_guard()
        u = step / steps
        smooth_u = u * u * (3.0 - 2.0 * u)
        reference = [a + smooth_u * (b - a) for a, b in zip(start, goal)]
        node.publish_reference(reference)
        deadline = time.monotonic() + period
        spin_and_sleep_until(node, deadline)


def smooth_return(
    node: IdentificationNode,
    keys: TerminalKeys,
    start_reference: Sequence[float],
    anchor: Sequence[float],
    duration_sec: float,
) -> None:
    """Backward-compatible wrapper for non-sweep modes."""
    smooth_transition(
        node,
        keys,
        start_reference,
        anchor,
        duration_sec,
        phase='return_to_anchor',
    )


def recent_position_median(node: IdentificationNode, seconds: float = 0.4) -> float:
    cutoff = time.monotonic_ns() - int(seconds * 1e9)
    values = [p.q for p in node.points if p.monotonic_ns >= cutoff]
    if values:
        return median_or_nan(values)
    if node.current_position is None:
        return math.nan
    return float(node.current_position[node.test_joint.index])


def abort_hold(
    node: IdentificationNode,
    keys: TerminalKeys,
    hold_pose: Sequence[float],
    reason: str,
) -> None:
    latched, latch_message = node.latch_driver_current_pose_hold(timeout_sec=2.0)
    print('\n' + '=' * 72)
    print('ABORT HOLD ACTIVE')
    print(f'Reason: {reason}')
    print(f'Driver hold latch: success={latched} message={latch_message}')
    if latched:
        print('Native driver internal pose override is active; external targets are blocked.')
    else:
        print('WARNING: native hold latch failed; fallback measured-pose publishing is active.')
    print('This is a SOFTWARE POSITION HOLD, not a torque-off E-stop.')
    print('Use the hardware power disconnect for an emergency.')
    print('Press x or X to stop the runner. A successful driver latch remains active after exit.')
    print('=' * 72 + '\n')

    node.phase = 'abort_hold'
    node.trial_id = ''
    period = 1.0 / 50.0
    next_tick = time.monotonic()
    while rclpy.ok():
        key = keys.read_key()
        if key in ('x', 'X'):
            return
        now = time.monotonic()
        if now >= next_tick:
            node.publish_reference(hold_pose)
            next_tick += period
        spin_and_sleep_until(node, next_tick)


def countdown(node: IdentificationNode, seconds: int) -> None:
    """Run the visible arming countdown without starving ROS callbacks.

    Diagnostics are published at a low rate relative to joint feedback.  A plain
    time.sleep() here would stop this single-threaded runner from servicing its
    subscriptions, making otherwise healthy diagnostics appear stale exactly when
    motion begins.  Keep spinning and re-run the continuous guard during the
    countdown so START is reached only with fresh feedback, fresh diagnostics, and
    a still-valid ROS graph.
    """
    print('\nMotion countdown:')
    for value in range(seconds, 0, -1):
        print(f'  {value}...')
        deadline = time.monotonic() + 1.0
        next_guard = time.monotonic()
        while rclpy.ok() and time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_guard:
                node.continuous_safety_guard()
                next_guard = now + 0.5
            spin_and_sleep_until(node, min(deadline, now + 0.05))
    node.continuous_safety_guard()
    print('  START\n')


def wait_for_preflight(node: IdentificationNode, timeout_sec: float) -> Tuple[Dict[str, object], str]:
    deadline = time.monotonic() + timeout_sec
    last_reason = 'waiting for data'
    graph_details: Dict[str, object] = {}
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        ready, driver_reason = node.driver_ready_for_motion()
        graph_ok, graph_reason, graph_details = node.graph_guard()
        if ready and graph_ok:
            return graph_details, 'ready'
        last_reason = f'driver={driver_reason}; graph={graph_reason}'
    return graph_details, last_reason


def load_torque_nm(args) -> Optional[float]:
    if args.load_force_n is not None:
        return abs(float(args.load_force_n) * float(args.lever_arm_m))
    if args.load_mass_kg is not None:
        return abs(float(args.load_mass_kg) * GRAVITY_M_S2 * float(args.lever_arm_m))
    return None


def write_summary(
    *,
    args,
    output_dir: Path,
    node: IdentificationNode,
    trials: Sequence[Trial],
    anchor: Sequence[float],
) -> Dict[str, object]:
    trial_results = [
        analyze_trial(trial, node.points, node.cycles, args.motion_threshold_rad) for trial in trials
    ]
    summary: Dict[str, object] = {
        'experiment_id': node.experiment_id,
        'joint_name': node.test_joint.name,
        'servo_id': node.test_joint.servo_id,
        'mode': args.mode,
        'command_path': args.command_path,
        'sample_count': len(node.points),
        'telemetry_cycle_count': len(node.cycles),
        'telemetry_dropped_count': node.last_telemetry_dropped_count,
        'test_center_offset_rad': args.test_center_offset_rad if args.mode in ('step', 'step_sweep') else None,
        'trial_results': trial_results,
    }

    if node.cycles:
        currents = [c.current_a for c in node.cycles if math.isfinite(c.current_a)]
        loads = [c.load_ratio for c in node.cycles if math.isfinite(c.load_ratio)]
        voltages = [c.voltage_v for c in node.cycles if math.isfinite(c.voltage_v)]
        temperatures = [c.temperature_c for c in node.cycles]

        correlation = None
        paired = [
            (abs(c.load_ratio), abs(c.current_a)) for c in node.cycles
            if math.isfinite(c.load_ratio) and math.isfinite(c.current_a)
        ]
        if len(paired) >= 3:
            xs = [x for x, _ in paired]
            ys = [y for _, y in paired]
            mx = statistics.mean(xs)
            my = statistics.mean(ys)
            sx = math.sqrt(sum((x - mx) ** 2 for x in xs))
            sy = math.sqrt(sum((y - my) ** 2 for y in ys))
            if sx > 1e-12 and sy > 1e-12:
                correlation = sum((x - mx) * (y - my) for x, y in paired) / (sx * sy)

        summary['electrical_load'] = {
            'peak_abs_current_a': max((abs(v) for v in currents), default=None),
            'median_abs_current_a': median_or_nan(abs(v) for v in currents) if currents else None,
            'peak_abs_load_ratio': max((abs(v) for v in loads), default=None),
            'median_abs_load_ratio': median_or_nan(abs(v) for v in loads) if loads else None,
            'min_voltage_v': min(voltages) if voltages else None,
            'max_voltage_v': max(voltages) if voltages else None,
            'min_temperature_c': min(temperatures) if temperatures else None,
            'max_temperature_c': max(temperatures) if temperatures else None,
            'abs_load_vs_abs_current_pearson_r': correlation,
            'interpretation': (
                'ST3215 load is a signed motor-drive duty-cycle proxy, not current. '
                'Use the direct current register for electrical current; correlation is reported only as an empirical diagnostic.'
            ),
        }

    if args.mode == 'step_sweep':
        by_direction: Dict[str, List[Dict[str, object]]] = {}
        for result in trial_results:
            if result.get('analysis_status') != 'ok':
                continue
            by_direction.setdefault(str(result['direction']), []).append(result)
        velocity_saturation = {}
        for direction, results in by_direction.items():
            ordered = sorted(results, key=lambda x: float(x['amplitude_rad']))
            curve = [
                {
                    'amplitude_rad': float(r['amplitude_rad']),
                    'peak_velocity_rad_s': float(r['peak_velocity_rad_s']),
                }
                for r in ordered
            ]
            candidate = None
            for prev, cur in zip(curve, curve[1:]):
                previous_speed = max(abs(prev['peak_velocity_rad_s']), 1e-9)
                gain = (cur['peak_velocity_rad_s'] - prev['peak_velocity_rad_s']) / previous_speed
                if gain < args.velocity_plateau_fraction:
                    candidate = cur['amplitude_rad']
                    break
            velocity_saturation[direction] = {
                'curve': curve,
                'plateau_candidate_amplitude_rad': candidate,
                'plateau_fraction_threshold': args.velocity_plateau_fraction,
            }
        summary['velocity_saturation'] = velocity_saturation

    if args.mode == 'deadband_staircase':
        anchor_q = float(anchor[node.test_joint.index])
        groups: Dict[str, List[Point]] = {}
        for p in node.points:
            if p.trial_id:
                groups.setdefault(p.trial_id, []).append(p)
        steps = []
        for trial_id, points in groups.items():
            steady = points[max(0, int(len(points) * 0.7)):]
            steady_q = median_or_nan(p.q for p in steady)
            offset = median_or_nan(p.q_ref - anchor_q for p in steady)
            movement = steady_q - anchor_q
            steps.append(
                {
                    'trial_id': trial_id,
                    'command_offset_rad': offset,
                    'steady_displacement_rad': movement,
                    'sustained_movement': abs(movement) >= args.motion_threshold_rad,
                }
            )
        useful = [s for s in steps if s['sustained_movement'] and abs(float(s['command_offset_rad'])) > 0.0]
        summary['deadband'] = {
            'steps': steps,
            'smallest_useful_command_rad': (
                min(abs(float(s['command_offset_rad'])) for s in useful) if useful else None
            ),
            'movement_threshold_rad': args.motion_threshold_rad,
        }

    if args.mode == 'triangle':
        tri = [p for p in node.points if p.phase == 'triangle']
        if tri:
            errors = [p.q_ref - p.q for p in tri]
            summary['triangle'] = {
                'tracking_rmse_rad': math.sqrt(sum(e * e for e in errors) / len(errors)),
                'peak_velocity_rad_s': max(abs(p.qdot) for p in tri),
                'sample_count': len(tri),
            }

    if args.mode == 'hold_under_load':
        baseline = [p for p in node.points if p.phase == 'load_baseline']
        loaded = [p for p in node.points if p.phase == 'load_hold']
        baseline_q = median_or_nan(p.q for p in baseline[max(0, int(len(baseline) * 0.5)):])
        loaded_q = median_or_nan(p.q for p in loaded[max(0, int(len(loaded) * 0.5)):])
        deflection = loaded_q - baseline_q
        torque = load_torque_nm(args)
        stiffness = None
        if torque is not None and math.isfinite(deflection) and abs(deflection) >= args.stiffness_min_deflection_rad:
            stiffness = torque / abs(deflection)
        summary['loaded_stiffness'] = {
            'baseline_q_rad': baseline_q,
            'loaded_q_rad': loaded_q,
            'deflection_rad': deflection,
            'applied_torque_nm': torque,
            'effective_static_stiffness_nm_per_rad': stiffness,
            'minimum_deflection_for_estimate_rad': args.stiffness_min_deflection_rad,
        }

    summary_path = output_dir / 'summary.yaml'
    summary_path.write_text(yaml.safe_dump(summary, sort_keys=False))

    lines = [
        'LittleGreen — Servo Identification Summary',
        '=' * 56,
        f"Experiment: {node.experiment_id}",
        f"Joint:      {node.test_joint.name} (servo {node.test_joint.servo_id})",
        f"Mode:       {args.mode}",
        f"Path:       {args.command_path}",
        f"Samples:    {len(node.points)}",
        '',
    ]
    for result in trial_results:
        lines.append(f"Trial {result.get('trial_id')}: {result.get('direction')} {result.get('amplitude_rad')} rad")
        if result.get('analysis_status') != 'ok':
            lines.append(f"  status: {result.get('analysis_status')}")
            continue
        lines.extend(
            [
                f"  lag publish->driver: {result.get('reference_publish_to_driver_receipt_ms')} ms",
                f"  driver->write start: {result.get('driver_receipt_to_sync_write_start_ms')} ms",
                f"  write end->moving flag:   {result.get('first_sync_write_end_to_moving_flag_ms')} ms",
                f"  write end->1st encoder:   {result.get('first_sync_write_end_to_first_encoder_count_ms')} ms",
                f"  write end->sustained:     {result.get('first_sync_write_end_to_motion_ms')} ms",
                f"  lag publish->sustained:   {result.get('reference_publish_to_motion_lag_ms')} ms",
                f"  command rise 10-90:       {result.get('rise_time_10_90_ms')} ms",
                f"  achieved rise 10-90:      {result.get('achieved_response_rise_time_10_90_ms')} ms",
                f"  achieved settle 1 count:  {result.get('achieved_response_settling_1_encoder_count_ms')} ms",
                f"  settle ±0.01 rad:         {result.get('settling_time_0p01_rad_ms')} ms",
                f"  overshoot:                {result.get('overshoot_percent'):.3f} %",
                f"  actual command delta:     {result.get('actual_command_delta_rad'):.6f} rad",
                f"  achieved displacement:    {result.get('achieved_displacement_rad'):.6f} rad",
                f"  static gain:              {result.get('static_gain_ratio'):.6f}",
                f"  steady error:             {result.get('steady_state_error_rad'):.6f} rad",
                f"  peak velocity:            {result.get('peak_velocity_rad_s'):.6f} rad/s",
                f"  peak |load ratio|:        {result.get('peak_abs_load_ratio')}",
                f"  peak |current|:           {result.get('peak_abs_current_a')} A",
            ]
        )
    (output_dir / 'summary.txt').write_text('\n'.join(lines) + '\n')
    return summary


def parse_float_list(text: str) -> List[float]:
    values = []
    for token in text.replace(',', ' ').split():
        values.append(float(token))
    if not values:
        raise argparse.ArgumentTypeError('expected one or more numeric values')
    return values


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Guarded one-joint ST3215 identification runner for LittleGreen.'
    )
    parser.add_argument('--joint', required=True, help='Canonical joint name or index 0..11')
    parser.add_argument(
        '--mode',
        required=True,
        choices=['step', 'step_sweep', 'deadband_staircase', 'triangle', 'hold_under_load'],
    )
    parser.add_argument('--command-path', choices=['direct', 'outer'], default='direct')
    parser.add_argument(
        '--allow-nonmax-motion-profile',
        action='store_true',
        help=(
            'Allow identification when driver diagnostics do not report '
            'motion_profile=max_envelope_fixed_0_0. Keep disabled for Track 1 baseline data.'
        ),
    )
    parser.add_argument('--servo-map', default='')
    parser.add_argument('--output-dir', type=Path, default=Path('identification_reports'))
    parser.add_argument('--notes', default='')
    parser.add_argument('--support-condition', default='securely_supported')

    parser.add_argument('--direction', choices=['positive', 'negative', 'both'], default='both')
    parser.add_argument('--amplitude-rad', type=float, default=0.02)
    parser.add_argument(
        '--amplitudes-rad',
        type=parse_float_list,
        default=[0.02, 0.05, 0.10],
        help='Comma/space-separated amplitudes for step_sweep. Conservative default stops at 0.10 rad.',
    )
    parser.add_argument(
        '--test-center-offset-rad',
        type=float,
        default=0.0,
        help=(
            'Temporary identification-center offset from the measured safe anchor. '
            'For the ankle-pitch 0.02/0.05/0.10 symmetric sweep, use +0.05 rad.'
        ),
    )
    parser.add_argument(
        '--test-center-move-sec',
        type=float,
        default=2.0,
        help='Smooth move duration from safe anchor to temporary test center and back.',
    )
    parser.add_argument(
        '--test-center-settle-sec',
        type=float,
        default=1.5,
        help='Settling hold at the temporary test center before the first trial.',
    )
    parser.add_argument('--max-test-offset-rad', type=float, default=0.20)
    parser.add_argument('--joint-limit-margin-rad', type=float, default=0.01)
    parser.add_argument('--motion-threshold-rad', type=float, default=0.002)
    parser.add_argument('--velocity-plateau-fraction', type=float, default=0.10)

    parser.add_argument('--baseline-sec', type=float, default=1.5)
    parser.add_argument('--step-hold-sec', type=float, default=2.5)
    parser.add_argument('--between-trials-sec', type=float, default=1.0)
    parser.add_argument('--return-sec', type=float, default=2.0)
    parser.add_argument('--final-hold-sec', type=float, default=1.0)

    parser.add_argument(
        '--deadband-offsets-rad',
        type=parse_float_list,
        default=[0.002, 0.005, 0.010, 0.020],
    )
    parser.add_argument('--deadband-dwell-sec', type=float, default=1.5)

    parser.add_argument('--triangle-amplitude-rad', type=float, default=0.02)
    parser.add_argument('--triangle-frequency-hz', type=float, default=0.10)
    parser.add_argument('--triangle-cycles', type=float, default=2.0)

    parser.add_argument('--load-baseline-sec', type=float, default=3.0)
    parser.add_argument('--load-prepare-sec', type=float, default=5.0)
    parser.add_argument('--load-hold-sec', type=float, default=10.0)
    parser.add_argument('--load-offset-rad', type=float, default=0.0)
    parser.add_argument('--load-force-n', type=float, default=None)
    parser.add_argument('--load-mass-kg', type=float, default=None)
    parser.add_argument('--lever-arm-m', type=float, default=0.0)
    parser.add_argument('--stiffness-min-deflection-rad', type=float, default=0.0015)

    parser.add_argument('--max-feedback-age-ms', type=int, default=100)
    parser.add_argument('--joint-state-timeout-sec', type=float, default=0.15)
    parser.add_argument('--preflight-timeout-sec', type=float, default=8.0)
    parser.add_argument('--countdown-sec', type=int, default=3)
    parser.add_argument('--direct-topic', default='/servo_target_radians')
    parser.add_argument('--outer-reference-topic', default='/desired_position')
    parser.add_argument(
        '--allow-all-2048-centers',
        action='store_true',
        help='Override guard that catches an obviously un-applied calibration map.',
    )
    return parser


def validate_args(args) -> None:
    positive_fields = [
        'max_test_offset_rad',
        'motion_threshold_rad',
        'baseline_sec',
        'step_hold_sec',
        'return_sec',
        'test_center_move_sec',
        'test_center_settle_sec',
        'max_feedback_age_ms',
        'joint_state_timeout_sec',
    ]
    for name in positive_fields:
        if float(getattr(args, name)) <= 0.0:
            raise ValueError(f'{name} must be positive')
    if args.max_test_offset_rad > 0.20 + 1e-12:
        raise ValueError('max_test_offset_rad may not exceed 0.20 rad in this guarded runner')
    if abs(args.test_center_offset_rad) > args.max_test_offset_rad + 1e-12:
        raise ValueError('test_center_offset_rad exceeds max_test_offset_rad')
    if args.mode == 'triangle':
        if args.triangle_amplitude_rad <= 0.0 or args.triangle_amplitude_rad > args.max_test_offset_rad:
            raise ValueError('triangle amplitude must be positive and within max_test_offset_rad')
        if args.triangle_frequency_hz <= 0.0 or args.triangle_cycles <= 0.0:
            raise ValueError('triangle frequency and cycles must be positive')
    if args.mode == 'hold_under_load':
        if abs(args.load_offset_rad) > args.max_test_offset_rad:
            raise ValueError('load_offset_rad exceeds max_test_offset_rad')
        if (args.load_force_n is not None or args.load_mass_kg is not None) and args.lever_arm_m <= 0.0:
            raise ValueError('lever_arm_m must be positive when force or mass metadata is supplied')
        if args.load_force_n is not None and args.load_mass_kg is not None:
            raise ValueError('supply either load_force_n or load_mass_kg, not both')


def main() -> int:
    parser = build_arg_parser()
    args, ros_args = parser.parse_known_args()

    try:
        validate_args(args)
        map_path = resolve_servo_map(args.servo_map)
        joints = load_joints(map_path)
        test_joint = parse_joint_selector(args.joint, joints)
        if all(j.center_step == 2048 for j in joints) and not args.allow_all_2048_centers:
            raise ValueError(
                'servo map has all center_step values at 2048. This looks uncalibrated; '
                'apply the validated calibration map or explicitly use --allow-all-2048-centers.'
            )
        planned_steps = bounded_offsets(args)
        for _, _, offset in planned_steps:
            if abs(offset) > args.max_test_offset_rad + 1e-12:
                raise ValueError('planned offset exceeds max_test_offset_rad')
    except Exception as exc:
        print(f'Configuration error: {exc}', file=sys.stderr)
        return 5

    timestamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')
    safe_joint = test_joint.name.replace('/', '_')
    experiment_id = f'{timestamp}_{safe_joint}_{args.mode}_{args.command_path}'
    output_dir = args.output_dir.expanduser().resolve() / experiment_id
    output_dir.mkdir(parents=True, exist_ok=False)
    csv_path = output_dir / 'timeseries.csv'

    rclpy.init(args=ros_args)
    node = IdentificationNode(
        experiment_id=experiment_id,
        joints=joints,
        test_joint=test_joint,
        command_path=args.command_path,
        direct_topic=args.direct_topic,
        outer_reference_topic=args.outer_reference_topic,
        max_feedback_age_ms=args.max_feedback_age_ms,
        joint_state_timeout_sec=args.joint_state_timeout_sec,
        allow_nonmax_motion_profile=args.allow_nonmax_motion_profile,
        csv_path=csv_path,
    )

    trials: List[Trial] = []
    graph_details: Dict[str, object] = {}
    anchor: Optional[List[float]] = None
    test_center_reference: Optional[List[float]] = None
    test_center_target_rad: Optional[float] = None
    aborted = False
    interrupted = False
    abort_reason = ''
    final_status = 'completed'
    summary: Dict[str, object] = {}

    def write_metadata(status: str) -> None:
        metadata = {
            'experiment_id': experiment_id,
            'created_utc': datetime.now(timezone.utc).isoformat(),
            'status': status,
            'joint_name': test_joint.name,
            'joint_index': test_joint.index,
            'servo_id': test_joint.servo_id,
            'test_type': args.mode,
            'command_path': args.command_path,
            'command_topic': node.command_topic,
            'outer_mode_note': (
                'Outer controller gains/mode are recorded from ROS diagnostics topics only; '
                'capture the active pd_config.yaml alongside this report for full provenance.'
                if args.command_path == 'outer'
                else 'Direct path bypasses the outer controller and identifies the native position-servo chain.'
            ),
            'servo_map_path': str(map_path),
            'servo_map_sha256': sha256_file(map_path),
            'servo_map_joint': asdict(test_joint),
            'driver_motion_profile': node.driver_diag_values.get('motion_profile', ''),
            'driver_configured_speed_steps_s': node.driver_diag_values.get('configured_speed_steps_s', ''),
            'driver_configured_acceleration_units': node.driver_diag_values.get('configured_acceleration_units', ''),
            'anchor_pose_rad': anchor,
            'test_center_offset_rad': args.test_center_offset_rad,
            'test_center_target_rad': test_center_target_rad,
            'test_center_reference_rad': test_center_reference,
            'max_test_offset_rad': args.max_test_offset_rad,
            'joint_limit_margin_rad': args.joint_limit_margin_rad,
            'motion_threshold_rad': args.motion_threshold_rad,
            'support_condition': args.support_condition,
            'notes': args.notes,
            'graph_preflight': graph_details,
            'telemetry_monotonic_clock_delta_ns_at_finish': node.telemetry_clock_delta_ns,
            'driver_diagnostics_snapshot': {
                'level': node.driver_diag_level,
                'message': node.driver_diag_message,
                'values': dict(node.driver_diag_values),
            },
            'test_arguments': dict(vars(args)),
            'applied_load_torque_nm': load_torque_nm(args),
            'caveats': [
                'reference_publish_to_motion_lag_ms remains the sustained-motion end-to-end metric; v2.4.1 also reports moving-flag onset and first encoder-count onset after the first matching SyncWrite.',
                'For step and step_sweep modes, every trial captures a fresh settled local baseline and commands q_target = q_local + requested_offset. The temporary test center is used only to provide safe room for symmetric sweeps.',
                'Command-relative static metrics and achieved-response-relative transient metrics are both reported because small ST3215 steps may settle below 90 percent of the commanded displacement.',
                'The Orange Pi runner verifies that Python monotonic_ns and the native driver steady_clock are in a comparable Linux monotonic clock domain before arming cross-language timing measurements.',
                'Step-response fitting characterizes the combined closed-loop servo + joint + load system, not a unique motor spring or damper coefficient.',
                'v2.4.2 Track 1 baseline uses a fixed ST3215 motion profile of speed=0 and acceleration=0 on every SyncWrite; no policy/PD-driven hardware profile modulation is performed.',
                'ST3215 raw speed is logged as raw_speed; qdot_meas_rad_s is the native driver position-delta filtered velocity.',
                'ST3215 load_ratio is a signed drive-output duty-cycle proxy and must not be treated as electrical current. current_a is decoded directly from the 0x45 current register using 6.5 mA/count.',
                'The v2.4 driver reads one contiguous 0x38..0x46 window per servo, so position, speed, load, voltage, temperature, status/moving, and current share the same transaction and sample timestamp.',
                'On abort the runner requests /st3215_driver/hold_current_pose, which latches measured feedback into the native driver command buffer and asserts the internal pose override.',
            ],
        }
        # pathlib and lists in argparse namespace need conversion.
        metadata['test_arguments']['output_dir'] = str(metadata['test_arguments']['output_dir'])
        (output_dir / 'metadata.yaml').write_text(yaml.safe_dump(metadata, sort_keys=False))

    try:
        print('\nLittleGreen — guarded servo identification')
        print('=' * 62)
        print(f'Experiment:   {experiment_id}')
        print(f'Joint:        {test_joint.name}')
        print(f'Servo ID:     {test_joint.servo_id}')
        print(f'Mode:         {args.mode}')
        print(f'Command path: {args.command_path} -> {node.command_topic}')
        print(f'Servo map:    {map_path}')
        print('\nREQUIRED CONDITIONS:')
        print('  robot securely supported; hardware power disconnect reachable')
        print('  LittleGreen policy process stopped/disconnected')
        print('  only this one-joint test command source active')
        print('  driver writes enabled and full feedback healthy')
        print('  driver pose override released before identification')
        print('\nKeyboard abort during motion: SPACE, q/Q, ESC, or Ctrl+C')

        graph_details, reason = wait_for_preflight(node, args.preflight_timeout_sec)
        if reason != 'ready':
            print(f'Preflight failed: {reason}', file=sys.stderr)
            final_status = 'preflight_failed'
            return 3

        assert node.current_position is not None
        anchor = list(node.current_position)
        for i, value in enumerate(anchor):
            if not math.isfinite(value):
                raise ValueError(f'non-finite anchor position at index {i}')
            joint = joints[i]
            if value < joint.min_rad or value > joint.max_rad:
                raise ValueError(
                    f'anchor pose for {joint.name}={value:.6f} rad is outside physical limits '
                    f'[{joint.min_rad:.6f}, {joint.max_rad:.6f}]'
                )

        if args.mode in ('step', 'step_sweep'):
            test_center_target_rad = validate_offset_target(
                test_joint,
                anchor[test_joint.index],
                args.test_center_offset_rad,
                args.joint_limit_margin_rad,
            )
            test_center_reference = make_reference(
                anchor,
                test_joint.index,
                test_center_target_rad,
            )
            # Nominal prevalidation. Each trial is also revalidated against its
            # fresh measured local baseline immediately before motion.
            for _, _, offset in planned_steps:
                validate_offset_target(
                    test_joint,
                    test_center_target_rad,
                    offset,
                    args.joint_limit_margin_rad,
                )
        elif args.mode == 'deadband_staircase':
            for amplitude in args.deadband_offsets_rad:
                if amplitude <= 0.0 or amplitude > args.max_test_offset_rad:
                    raise ValueError('deadband offsets must be positive and within max_test_offset_rad')
                for offset in (amplitude, -amplitude):
                    validate_offset_target(test_joint, anchor[test_joint.index], offset, args.joint_limit_margin_rad)
        elif args.mode == 'triangle':
            for offset in (args.triangle_amplitude_rad, -args.triangle_amplitude_rad):
                validate_offset_target(test_joint, anchor[test_joint.index], offset, args.joint_limit_margin_rad)
        elif args.mode == 'hold_under_load':
            validate_offset_target(
                test_joint,
                anchor[test_joint.index],
                args.load_offset_rad,
                args.joint_limit_margin_rad,
            )

        print('\nMeasured anchor pose accepted.')
        print(f'  selected joint anchor: {anchor[test_joint.index]:+.6f} rad')
        print(f'  guarded physical range: [{test_joint.min_rad + args.joint_limit_margin_rad:+.6f}, '
              f'{test_joint.max_rad - args.joint_limit_margin_rad:+.6f}] rad')
        if args.mode in ('step', 'step_sweep'):
            assert test_center_target_rad is not None
            print(f'  test center offset:     {args.test_center_offset_rad:+.6f} rad')
            print(f'  nominal test center:    {test_center_target_rad:+.6f} rad')
            print('  planned local steps:')
            for direction, amplitude, offset in planned_steps:
                print(f'    {direction:8s} amplitude={amplitude:.4f} rad offset={offset:+.4f} rad')

        phrase = f'ARM {test_joint.name} {args.mode}'
        confirmation = input(f'\nType exactly:\n  {phrase}\n> ').strip()
        if confirmation != phrase:
            print('Cancelled. No motion command was published.')
            final_status = 'cancelled_before_motion'
            return 0

        # The operator may spend an arbitrary amount of time at the blocking ARM
        # prompt.  Reacquire fresh diagnostics/feedback and re-check graph ownership
        # before beginning the countdown.  No command has been published yet.
        graph_details, reason = wait_for_preflight(node, args.preflight_timeout_sec)
        if reason != 'ready':
            print(f'Post-arm preflight failed: {reason}', file=sys.stderr)
            final_status = 'post_arm_preflight_failed'
            return 3

        node.telemetry_drop_guard_baseline = node.last_telemetry_dropped_count
        countdown(node, args.countdown_sec)
        node.logging_active = True

        with TerminalKeys() as keys:
            run_reference_phase(
                node,
                keys,
                anchor,
                args.baseline_sec,
                phase='anchor_baseline',
            )

            if args.mode in ('step', 'step_sweep'):
                assert test_center_reference is not None
                assert test_center_target_rad is not None

                # Move slowly from the measured safe anchor to a temporary test
                # center that provides symmetric limit margin for the sweep.
                smooth_transition(
                    node,
                    keys,
                    anchor,
                    test_center_reference,
                    args.test_center_move_sec,
                    phase='move_to_test_center',
                )
                run_reference_phase(
                    node,
                    keys,
                    test_center_reference,
                    args.test_center_settle_sec,
                    phase='test_center_settle',
                )

                for count, (direction, amplitude, offset) in enumerate(planned_steps, start=1):
                    # Re-establish the test-center command and allow the mechanism
                    # to settle. Then capture a fresh local measured baseline so
                    # every claimed +/- amplitude is relative to actual q, not to
                    # an earlier experiment anchor or imperfect return position.
                    run_reference_phase(
                        node,
                        keys,
                        test_center_reference,
                        args.between_trials_sec,
                        phase='pre_step_hold',
                    )
                    q0 = recent_position_median(node)
                    if not math.isfinite(q0):
                        raise AbortRequested('fresh local trial baseline is not finite')
                    target_q = validate_offset_target(
                        test_joint,
                        q0,
                        offset,
                        args.joint_limit_margin_rad,
                    )
                    target = make_reference(anchor, test_joint.index, target_q)
                    trial_id = f'step_{count:02d}_{direction}_{amplitude:.4f}'
                    print(
                        f'\nTrial {count}/{len(planned_steps)}: {direction} {amplitude:.4f} rad '
                        f'from local q0={q0:+.6f} -> target={target_q:+.6f}'
                    )
                    command_ns = run_reference_phase(
                        node,
                        keys,
                        target,
                        args.step_hold_sec,
                        phase='step_hold',
                        trial_id=trial_id,
                    )
                    trials.append(
                        Trial(
                            trial_id=trial_id,
                            direction=direction,
                            amplitude_rad=amplitude,
                            offset_rad=offset,
                            command_monotonic_ns=command_ns,
                            q_initial_rad=q0,
                            q_target_rad=target_q,
                            hold_duration_sec=args.step_hold_sec,
                        )
                    )
                    smooth_transition(
                        node,
                        keys,
                        target,
                        test_center_reference,
                        args.return_sec,
                        phase='return_to_test_center',
                    )

                run_reference_phase(
                    node,
                    keys,
                    test_center_reference,
                    args.between_trials_sec,
                    phase='final_test_center_hold',
                )
                smooth_transition(
                    node,
                    keys,
                    test_center_reference,
                    anchor,
                    args.test_center_move_sec,
                    phase='return_to_safe_anchor',
                )

            elif args.mode == 'deadband_staircase':
                sequence: List[float] = [0.0]
                sequence += [float(v) for v in args.deadband_offsets_rad]
                sequence += [0.0]
                sequence += [-float(v) for v in args.deadband_offsets_rad]
                sequence += [0.0]
                for count, offset in enumerate(sequence):
                    target_q = validate_offset_target(
                        test_joint,
                        anchor[test_joint.index],
                        offset,
                        args.joint_limit_margin_rad,
                    )
                    target = make_reference(anchor, test_joint.index, target_q)
                    trial_id = f'deadband_{count:02d}_{offset:+.4f}'
                    run_reference_phase(
                        node,
                        keys,
                        target,
                        args.deadband_dwell_sec,
                        phase='deadband_dwell',
                        trial_id=trial_id,
                    )
                smooth_return(node, keys, node.reference, anchor, args.return_sec)

            elif args.mode == 'triangle':
                node.phase = 'triangle'
                node.trial_id = 'triangle'
                period = 1.0 / 50.0
                duration = args.triangle_cycles / args.triangle_frequency_hz
                start_time = time.monotonic()
                next_tick = start_time
                next_guard = start_time
                amp = args.triangle_amplitude_rad
                while rclpy.ok() and time.monotonic() - start_time < duration:
                    key = keys.read_key()
                    if key in (' ', 'q', 'Q', '\x1b'):
                        raise AbortRequested(f'operator abort key {repr(key)}')
                    now = time.monotonic()
                    if now >= next_guard:
                        node.continuous_safety_guard()
                        next_guard = now + 0.5
                    if now >= next_tick:
                        phase_cycles = (now - start_time) * args.triangle_frequency_hz
                        u = phase_cycles % 1.0
                        tri = 4.0 * u - 1.0 if u < 0.5 else 3.0 - 4.0 * u
                        target_q = anchor[test_joint.index] + amp * tri
                        target = make_reference(anchor, test_joint.index, target_q)
                        node.publish_reference(target)
                        next_tick += period
                    spin_and_sleep_until(node, next_tick)
                smooth_return(node, keys, node.reference, anchor, args.return_sec)

            elif args.mode == 'hold_under_load':
                target_q = validate_offset_target(
                    test_joint,
                    anchor[test_joint.index],
                    args.load_offset_rad,
                    args.joint_limit_margin_rad,
                )
                target = make_reference(anchor, test_joint.index, target_q)
                run_reference_phase(
                    node,
                    keys,
                    target,
                    args.load_baseline_sec,
                    phase='load_baseline',
                )
                print('\nAPPLY THE KNOWN LOAD DURING THE PREPARE WINDOW.')
                print(f'Loaded capture starts in {args.load_prepare_sec:.1f} s.')
                run_reference_phase(
                    node,
                    keys,
                    target,
                    args.load_prepare_sec,
                    phase='load_prepare',
                )
                print('LOADED CAPTURE STARTED')
                run_reference_phase(
                    node,
                    keys,
                    target,
                    args.load_hold_sec,
                    phase='load_hold',
                    trial_id='hold_under_load',
                )
                print('LOADED CAPTURE COMPLETE — remove the external load if safe.')
                smooth_return(node, keys, target, anchor, args.return_sec)

            run_reference_phase(
                node,
                keys,
                anchor,
                args.final_hold_sec,
                phase='final_anchor_hold',
            )

    except KeyboardInterrupt:
        aborted = True
        interrupted = True
        abort_reason = 'Ctrl+C'
    except AbortRequested as exc:
        aborted = True
        abort_reason = str(exc)
    except Exception as exc:
        aborted = True
        abort_reason = f'exception: {exc}'
        print(f'Error: {exc}', file=sys.stderr)
    finally:
        if aborted and anchor is not None and node.current_position is not None:
            hold_pose = list(node.current_position)
            try:
                with TerminalKeys() as keys:
                    abort_hold(node, keys, hold_pose, abort_reason)
            except KeyboardInterrupt:
                print('\nSecond Ctrl+C: leaving abort hold and shutting down runner.')
            except Exception as hold_exc:
                print(f'Abort hold error: {hold_exc}', file=sys.stderr)

        try:
            if anchor is not None:
                summary = write_summary(
                    args=args,
                    output_dir=output_dir,
                    node=node,
                    trials=trials,
                    anchor=anchor,
                )
        except Exception as exc:
            print(f'Summary generation failed: {exc}', file=sys.stderr)

        try:
            write_metadata('aborted' if aborted else final_status)
        except Exception as exc:
            print(f'Metadata write failed: {exc}', file=sys.stderr)

        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    write_manifest(
        output_dir,
        'servo_identification',
        config_paths={'servo_map': servo_map_path},
        result_status='aborted' if aborted else 'completed',
        exit_code=130 if interrupted else (7 if aborted else 0),
        extra={'joint': args.joint, 'mode': args.mode},
    )

    print(f'\nArtifacts: {output_dir}')
    print(f'  CSV:      {csv_path}')
    print(f'  metadata: {output_dir / "metadata.yaml"}')
    print(f'  summary:  {output_dir / "summary.yaml"}')
    if aborted:
        print(f'Test aborted: {abort_reason}', file=sys.stderr)
        return 130 if interrupted else 7
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
