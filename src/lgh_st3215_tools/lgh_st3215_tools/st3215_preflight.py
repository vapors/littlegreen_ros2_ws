#!/usr/bin/env python3
"""Read-only ST3215 ROS preflight with explicit modes and meaningful exit status."""
from __future__ import annotations

import argparse
import math
import socket
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import rclpy
from lgh_st3215_driver.msg import ServoTelemetry
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt32MultiArray

from lgh_st3215_tools.diagnostic_compat import diagnostic_level_to_int
from lgh_st3215_tools.exit_codes import ExitCode
from lgh_st3215_tools.result import CheckResult, ToolResult, make_report_dir, utc_now, write_result

SENSOR_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=20,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
)

ERROR_COUNTERS = (
    'sync_write_error_count', 'read_timeout_count', 'checksum_error_count',
    'malformed_frame_count', 'wrong_id_count', 'io_error_count',
    'servo_status_error_count',
)

@dataclass
class Observation:
    diagnostics: dict[str, str] = field(default_factory=dict)
    diagnostic_level: Optional[int] = None
    diagnostic_message: str = ''
    first_counters: dict[str, int] = field(default_factory=dict)
    last_counters: dict[str, int] = field(default_factory=dict)
    joint_positions: Optional[list[float]] = None
    joint_velocities: Optional[list[float]] = None
    ages_ms: Optional[list[int]] = None
    telemetry_count: int = 0
    telemetry_drops_last: int = 0
    diagnostic_count: int = 0
    joint_state_count: int = 0
    age_count: int = 0

class PreflightNode(Node):
    def __init__(self, diagnostics_topic: str, joint_topic: str, age_topic: str, telemetry_topic: str) -> None:
        super().__init__('st3215_preflight')
        self.obs = Observation()
        self.create_subscription(DiagnosticArray, diagnostics_topic, self._diag, 10)
        self.create_subscription(JointState, joint_topic, self._joint, SENSOR_QOS)
        self.create_subscription(UInt32MultiArray, age_topic, self._age, SENSOR_QOS)
        self.create_subscription(ServoTelemetry, telemetry_topic, self._telemetry, SENSOR_QOS)

    def _diag(self, msg: DiagnosticArray) -> None:
        for status in msg.status:
            if status.name != 'ST3215 native single bus':
                continue
            values = {item.key: item.value for item in status.values}
            self.obs.diagnostics = values
            self.obs.diagnostic_level = diagnostic_level_to_int(status.level)
            self.obs.diagnostic_message = status.message
            self.obs.diagnostic_count += 1
            counters: dict[str, int] = {}
            for key in ERROR_COUNTERS:
                try:
                    counters[key] = int(values.get(key, '0'))
                except ValueError:
                    counters[key] = -1
            if not self.obs.first_counters:
                self.obs.first_counters = counters
            self.obs.last_counters = counters
            break

    def _joint(self, msg: JointState) -> None:
        self.obs.joint_state_count += 1
        self.obs.joint_positions = [float(v) for v in msg.position]
        self.obs.joint_velocities = [float(v) for v in msg.velocity]

    def _age(self, msg: UInt32MultiArray) -> None:
        self.obs.age_count += 1
        self.obs.ages_ms = [int(v) for v in msg.data]

    def _telemetry(self, msg: ServoTelemetry) -> None:
        self.obs.telemetry_count += 1
        self.obs.telemetry_drops_last = int(msg.telemetry_dropped_count)


def bool_value(values: dict[str, str], key: str) -> Optional[bool]:
    if key not in values:
        return None
    return values[key].strip().lower() == 'true'

def float_value(values: dict[str, str], key: str) -> Optional[float]:
    try:
        return float(values[key])
    except (KeyError, ValueError):
        return None

def int_value(values: dict[str, str], key: str) -> Optional[int]:
    try:
        return int(values[key])
    except (KeyError, ValueError):
        return None

def add(checks: list[CheckResult], name: str, ok: bool, passed: str, failed: str, *, refusal: bool = False, details=None) -> None:
    status = 'PASS' if ok else ('REFUSED' if refusal else 'FAIL')
    checks.append(CheckResult(name, status, passed if ok else failed, details or {}))

def publishers(node: Node, topic: str) -> list[str]:
    return sorted({info.node_name for info in node.get_publishers_info_by_topic(topic)})

def parse_expect(value: str) -> Optional[bool]:
    if value == 'auto':
        return None
    return value == 'true'

def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=['feedback', 'commissioning', 'runtime'], required=True)
    parser.add_argument('--expect-writes', choices=['auto', 'true', 'false'], default='auto')
    parser.add_argument('--sample-sec', type=float, default=3.0)
    parser.add_argument('--timeout-sec', type=float, default=8.0)
    parser.add_argument('--max-feedback-age-ms', type=int, default=100)
    parser.add_argument('--min-cycle-rate-hz', type=float, default=45.0)
    parser.add_argument('--max-cycle-work-p99-us', type=float, default=15000.0)
    parser.add_argument('--output-root', type=Path, default=None)
    parser.add_argument('--diagnostics-topic', default='/st3215_driver/diagnostics')
    parser.add_argument('--joint-topic', default='/joint_states')
    parser.add_argument('--age-topic', default='/joint_feedback_age_ms')
    parser.add_argument('--telemetry-topic', default='/st3215_driver/telemetry')
    args = parser.parse_args()

    if args.sample_sec <= 0 or args.timeout_sec <= 0 or args.timeout_sec < args.sample_sec:
        print('Invalid timeout/sample duration.', file=sys.stderr)
        return int(ExitCode.CONFIG_ERROR)

    started = utc_now()
    report_dir = make_report_dir(args.output_root, f'st3215_preflight_{args.mode}')
    rclpy.init()
    node = PreflightNode(args.diagnostics_topic, args.joint_topic, args.age_topic, args.telemetry_topic)
    deadline = time.monotonic() + args.timeout_sec
    sample_deadline: Optional[float] = None
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.obs.diagnostic_count and node.obs.joint_state_count and node.obs.age_count:
                if sample_deadline is None:
                    sample_deadline = time.monotonic() + args.sample_sec
                if time.monotonic() >= sample_deadline:
                    break
    except KeyboardInterrupt:
        node.destroy_node(); rclpy.shutdown()
        return int(ExitCode.INTERRUPTED_BY_SIGINT)

    checks: list[CheckResult] = []
    obs = node.obs
    missing = []
    if not obs.diagnostic_count: missing.append(args.diagnostics_topic)
    if not obs.joint_state_count: missing.append(args.joint_topic)
    if not obs.age_count: missing.append(args.age_topic)
    add(checks, 'required_streams', not missing, 'required driver streams received', f'missing streams: {missing}', details={'missing': missing})

    expected_profile = {'feedback': None, 'commissioning': 'commissioning', 'runtime': 'runtime_safe'}[args.mode]
    profile = obs.diagnostics.get('driver_profile')
    if expected_profile:
        add(checks, 'driver_profile', profile == expected_profile,
            f'profile={profile}', f'expected {expected_profile}, got {profile}', refusal=True)

    feedback_ready = bool_value(obs.diagnostics, 'feedback_ready')
    add(checks, 'feedback_ready', feedback_ready is True, 'all servo feedback represented', f'feedback_ready={feedback_ready}')

    writes = bool_value(obs.diagnostics, 'writes_enabled')
    expected_writes = parse_expect(args.expect_writes)
    if args.mode == 'feedback': expected_writes = False
    if expected_writes is not None:
        add(checks, 'writes_state', writes == expected_writes,
            f'writes_enabled={writes}', f'expected writes_enabled={expected_writes}, got {writes}', refusal=True)

    positions_ok = obs.joint_positions is not None and len(obs.joint_positions) == 12 and all(math.isfinite(v) for v in obs.joint_positions)
    velocities_ok = obs.joint_velocities is not None and len(obs.joint_velocities) == 12 and all(math.isfinite(v) for v in obs.joint_velocities)
    add(checks, 'joint_state_shape', positions_ok and velocities_ok,
        'position[12] and velocity[12] are finite', 'joint-state shape or finite-value check failed')

    ages_ok = obs.ages_ms is not None and len(obs.ages_ms) == 12
    max_age = max(obs.ages_ms) if ages_ok else None
    add(checks, 'feedback_age_shape', ages_ok, 'feedback ages contain 12 entries', 'feedback ages do not contain 12 entries')
    if ages_ok:
        add(checks, 'feedback_age_limit', max_age <= args.max_feedback_age_ms,
            f'max age {max_age} ms', f'max age {max_age} ms exceeds {args.max_feedback_age_ms} ms', details={'ages_ms': obs.ages_ms})

    cycle_rate = float_value(obs.diagnostics, 'cycle_rate_hz')
    add(checks, 'cycle_rate', cycle_rate is not None and cycle_rate >= args.min_cycle_rate_hz,
        f'{cycle_rate:.3f} Hz' if cycle_rate is not None else 'cycle rate available',
        f'cycle rate {cycle_rate} below {args.min_cycle_rate_hz} Hz')
    p99 = float_value(obs.diagnostics, 'cycle_work_us_p99')
    add(checks, 'cycle_work_p99', p99 is not None and p99 <= args.max_cycle_work_p99_us,
        f'{p99:.1f} us' if p99 is not None else 'p99 available',
        f'cycle work p99 {p99} us exceeds {args.max_cycle_work_p99_us} us')

    deltas = {key: obs.last_counters.get(key, 0) - obs.first_counters.get(key, 0) for key in ERROR_COUNTERS}
    add(checks, 'error_counter_growth', all(v == 0 for v in deltas.values()),
        'no I/O/protocol error counter growth during sample window', f'counter growth detected: {deltas}', details=deltas)

    all_nodes = {name for name, _ns in node.get_node_names_and_namespaces()}
    if args.mode == 'commissioning':
        policy_active = 'littlegreen_biped_node' in all_nodes
        add(checks, 'policy_disconnected', not policy_active, 'policy node absent', 'littlegreen_biped_node is active', refusal=True)

    topic_nodes = {
        args.joint_topic: publishers(node, args.joint_topic),
        args.age_topic: publishers(node, args.age_topic),
        args.telemetry_topic: publishers(node, args.telemetry_topic),
        '/st3215_driver/raw_position_steps': publishers(node, '/st3215_driver/raw_position_steps'),
        '/st3215_driver/raw_speed': publishers(node, '/st3215_driver/raw_speed'),
        '/servo_target_steps_debug': publishers(node, '/servo_target_steps_debug'),
        '/servo_target_radians': publishers(node, '/servo_target_radians'),
    }
    if args.mode == 'commissioning':
        lab_present = bool(topic_nodes[args.telemetry_topic]) and bool(topic_nodes['/st3215_driver/raw_position_steps']) and bool(topic_nodes['/st3215_driver/raw_speed'])
        add(checks, 'commissioning_topics', lab_present, 'telemetry and raw commissioning topics are present', 'one or more commissioning topics are absent')
    elif args.mode == 'runtime':
        lab_absent = not topic_nodes[args.telemetry_topic] and not topic_nodes['/st3215_driver/raw_position_steps'] and not topic_nodes['/st3215_driver/raw_speed'] and not topic_nodes['/servo_target_steps_debug']
        add(checks, 'runtime_topic_surface', lab_absent, 'laboratory high-rate topics are absent', 'laboratory topics are still published in runtime_safe')

    cmd_publishers = topic_nodes['/servo_target_radians']
    add(checks, 'command_authority_count', len(cmd_publishers) <= 1,
        f'{len(cmd_publishers)} command publisher(s)', f'competing command publishers: {cmd_publishers}', refusal=True, details={'publishers': cmd_publishers})

    refusal = any(c.status == 'REFUSED' for c in checks)
    failed = any(c.status == 'FAIL' for c in checks)
    if missing:
        code = ExitCode.TIMEOUT_OR_UNAVAILABLE
    elif refusal:
        code = ExitCode.REFUSED_PRECONDITION
    elif failed:
        code = ExitCode.TEST_FAIL
    else:
        code = ExitCode.PASS
    status = 'PASS' if code == ExitCode.PASS else ('REFUSED' if code == ExitCode.REFUSED_PRECONDITION else 'FAIL')

    result = ToolResult(
        tool='st3215_preflight', mode=args.mode, status=status, exit_code=int(code),
        started_utc=started, completed_utc=utc_now(), checks=checks,
        metadata={
            'hostname': socket.gethostname(),
            'diagnostic_message': obs.diagnostic_message,
            'diagnostic_level': obs.diagnostic_level,
            'driver_values': obs.diagnostics,
            'topic_publishers': topic_nodes,
            'sample_counts': {
                'diagnostics': obs.diagnostic_count, 'joint_states': obs.joint_state_count,
                'feedback_age': obs.age_count, 'telemetry': obs.telemetry_count,
            },
        },
    )
    write_result(report_dir, result)
    print(f'ST3215 PREFLIGHT: {status}')
    print(f'mode: {args.mode}')
    print(f'checks: {sum(c.status == "PASS" for c in checks)} passed, {sum(c.status == "REFUSED" for c in checks)} refused, {sum(c.status == "FAIL" for c in checks)} failed')
    print(f'report: {report_dir / "report.yaml"}')
    print(f'exit_code: {int(code)}')
    for check in checks:
        if check.status != 'PASS':
            print(f'[{check.status}] {check.name}: {check.message}', file=sys.stderr)

    node.destroy_node()
    rclpy.shutdown()
    return int(code)

if __name__ == '__main__':
    raise SystemExit(main())
