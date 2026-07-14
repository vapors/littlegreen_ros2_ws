#!/usr/bin/env python3
"""Verify raw ST3215 positions against model-zero or policy-default references."""

from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Int32MultiArray, UInt32MultiArray

from lgh_st3215_tools.calibration_common import (
    RADIANS_PER_STEP,
    REFERENCE_CHOICES,
    REFERENCE_MODEL_ZERO,
    REFERENCE_POLICY_DEFAULT,
    expected_step_for_reference,
    format_pose_reference,
    load_servo_map,
    reference_angle_rad,
    sha256_file,
)


class ReferenceVerificationNode(Node):
    def __init__(
        self,
        raw_topic: str,
        age_topic: str,
        diagnostics_topic: str,
        samples: int,
        max_age_ms: int,
    ) -> None:
        super().__init__("st3215_reference_pose_verify")
        self.target_samples = samples
        self.max_age_ms = max_age_ms
        self.samples: list[list[int]] = []
        self.latest_ages: Optional[list[int]] = None
        self.feedback_ready: Optional[bool] = None
        self.writes_enabled: Optional[bool] = None
        self.pose_move_running: Optional[bool] = None
        self.rejected_stale = 0
        self.rejected_shape = 0

        self.create_subscription(
            Int32MultiArray, raw_topic, self._raw_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            UInt32MultiArray, age_topic, self._age_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            DiagnosticArray, diagnostics_topic, self._diagnostics_callback, 10
        )

    def _age_callback(self, msg: UInt32MultiArray) -> None:
        if len(msg.data) == 12:
            self.latest_ages = [int(value) for value in msg.data]

    def _raw_callback(self, msg: Int32MultiArray) -> None:
        if len(self.samples) >= self.target_samples:
            return
        if len(msg.data) != 12:
            self.rejected_shape += 1
            return
        if self.latest_ages is None or len(self.latest_ages) != 12:
            self.rejected_stale += 1
            return
        if any(age > self.max_age_ms for age in self.latest_ages):
            self.rejected_stale += 1
            return
        self.samples.append([int(value) for value in msg.data])

    def _diagnostics_callback(self, msg: DiagnosticArray) -> None:
        for status in msg.status:
            if status.name != "ST3215 native single bus":
                continue
            values = {entry.key: entry.value for entry in status.values}
            if "feedback_ready" in values:
                self.feedback_ready = values["feedback_ready"].lower() == "true"
            if "writes_enabled" in values:
                self.writes_enabled = values["writes_enabled"].lower() == "true"
            if "pose_move_running" in values:
                self.pose_move_running = values["pose_move_running"].lower() == "true"
            break


def default_map_path() -> Path:
    return Path(get_package_share_directory("lgh_st3215_driver")) / "config" / "servo_map.yaml"


def _select_joints(joints, names: list[str]):
    if not names:
        return list(joints)
    by_name = {joint.name: joint for joint in joints}
    unknown = sorted(set(names) - set(by_name))
    if unknown:
        raise ValueError("Unknown --joint name(s): " + ", ".join(unknown))
    requested = set(names)
    return [joint for joint in joints if joint.name in requested]


def run(reference_default: str | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Verify raw servo steps against model-zero or policy-default pose."
    )
    parser.add_argument("--servo-map", type=Path, default=None)
    parser.add_argument(
        "--reference",
        choices=REFERENCE_CHOICES,
        default=reference_default or REFERENCE_MODEL_ZERO,
    )
    parser.add_argument(
        "--joint",
        action="append",
        default=[],
        help="Verify only this joint; may be repeated.",
    )
    parser.add_argument("--raw-topic", default="/st3215_driver/raw_position_steps")
    parser.add_argument("--age-topic", default="/joint_feedback_age_ms")
    parser.add_argument("--diagnostics-topic", default="/st3215_driver/diagnostics")
    parser.add_argument("--samples", type=int, default=100)
    parser.add_argument("--timeout-sec", type=float, default=10.0)
    parser.add_argument("--max-feedback-age-ms", type=int, default=50)
    parser.add_argument("--pass-tolerance-steps", type=int, default=8)
    parser.add_argument("--warn-tolerance-steps", type=int, default=16)
    parser.add_argument("--allow-writes-enabled", action="store_true")
    parser.add_argument("--output-dir", type=Path, default=Path("calibration_reports"))
    args, ros_args = parser.parse_known_args()

    if args.samples < 10:
        parser.error("--samples must be at least 10")
    if args.pass_tolerance_steps < 0 or args.warn_tolerance_steps < args.pass_tolerance_steps:
        parser.error("Step tolerances are inconsistent")

    map_path = args.servo_map.expanduser().resolve() if args.servo_map else default_map_path()
    _, all_joints = load_servo_map(map_path)
    try:
        joints = _select_joints(all_joints, args.joint)
    except ValueError as exc:
        parser.error(str(exc))

    print(f"\nST3215 {args.reference} pose verification")
    print("=" * 48)
    print(f"Servo map: {map_path}")
    print(f"Selected joints: {len(joints)}/{len(all_joints)}")
    print()
    print(format_pose_reference(joints, args.reference))
    print()
    if args.reference == REFERENCE_MODEL_ZERO:
        print(
            "The robot must be physically held at MODEL ZERO. This verifies raw steps "
            "against center_step; physical angle alignment still requires an external fixture/reference."
        )
    else:
        print(
            "Run assume_policy_default/pose_console first, then use this check to verify "
            "that the servos reached the policy-default raw targets."
        )

    rclpy.init(args=ros_args)
    node = ReferenceVerificationNode(
        args.raw_topic,
        args.age_topic,
        args.diagnostics_topic,
        args.samples,
        args.max_feedback_age_ms,
    )
    try:
        preflight_deadline = time.monotonic() + min(5.0, args.timeout_sec)
        while rclpy.ok() and time.monotonic() < preflight_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.feedback_ready is True and node.writes_enabled is not None:
                break
        if node.feedback_ready is not True:
            raise RuntimeError("Driver feedback is not ready")
        if node.pose_move_running:
            raise RuntimeError("Pose ramp is still running; wait for completion")
        if node.writes_enabled and not args.allow_writes_enabled:
            raise RuntimeError(
                "Driver reports writes_enabled=true. Pass --allow-writes-enabled for a "
                "guarded policy-default verification or relaunch feedback-only."
            )

        print(f"Collecting {args.samples} raw position samples...")
        deadline = time.monotonic() + args.timeout_sec
        next_progress = max(1, args.samples // 5)
        while rclpy.ok() and len(node.samples) < args.samples:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise RuntimeError(
                    f"Timed out after {len(node.samples)}/{args.samples} accepted samples"
                )
            rclpy.spin_once(node, timeout_sec=min(0.1, remaining))
            if len(node.samples) >= next_progress:
                print(f"  captured {len(node.samples)}/{args.samples}")
                next_progress += max(1, args.samples // 5)
    except Exception as exc:
        print(f"Verification failed: {exc}", file=sys.stderr)
        return 4
    finally:
        node.destroy_node()
        rclpy.shutdown()

    columns = list(zip(*node.samples))
    rows = []
    fail_count = 0
    warn_count = 0

    print(f"\n{args.reference} verification result")
    print("=" * 48)
    for joint in joints:
        values = [int(value) for value in columns[joint.policy_index]]
        median_step = float(statistics.median(values))
        expected_step = expected_step_for_reference(joint, args.reference)
        error_steps = median_step - expected_step
        abs_error = abs(error_steps)
        if abs_error <= args.pass_tolerance_steps:
            status = "PASS"
        elif abs_error <= args.warn_tolerance_steps:
            status = "WARN"
            warn_count += 1
        else:
            status = "FAIL"
            fail_count += 1
        error_rad = error_steps * joint.servo_sign * RADIANS_PER_STEP
        row = {
            "name": joint.name,
            "policy_index": joint.policy_index,
            "servo_id": joint.servo_id,
            "reference": args.reference,
            "reference_rad": reference_angle_rad(joint, args.reference),
            "expected_step": expected_step,
            "measured_median_step": median_step,
            "measured_min_step": min(values),
            "measured_max_step": max(values),
            "error_steps": error_steps,
            "error_rad_equivalent": error_rad,
            "error_deg_equivalent": math.degrees(error_rad),
            "status": status,
        }
        rows.append(row)
        print(
            f"{joint.policy_index:>2} ID{joint.servo_id:>2} {joint.name:<36} "
            f"expected={expected_step:>4} meas={median_step:>7.1f} "
            f"err={error_steps:>+6.1f}step "
            f"({math.degrees(error_rad):>+6.2f}deg) {status}"
        )

    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    output_dir = args.output_dir.expanduser().resolve() / f"verify-{args.reference}-{timestamp}"
    output_dir.mkdir(parents=True, exist_ok=False)
    report = {
        "schema_version": 2,
        "verification_type": f"{args.reference}_raw_step_tracking",
        "timestamp_utc": timestamp,
        "servo_map": str(map_path),
        "servo_map_sha256": sha256_file(map_path),
        "selected_joint_count": len(joints),
        "sample_count": args.samples,
        "pass_tolerance_steps": args.pass_tolerance_steps,
        "warn_tolerance_steps": args.warn_tolerance_steps,
        "fail_count": fail_count,
        "warn_count": warn_count,
        "rejected_stale_samples": node.rejected_stale,
        "rejected_shape_samples": node.rejected_shape,
        "joints": rows,
    }
    (output_dir / "verification_report.yaml").write_text(
        yaml.safe_dump(report, sort_keys=False)
    )
    with (output_dir / "verification_summary.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    print()
    print(f"Report: {output_dir / 'verification_report.yaml'}")
    print(f"CSV:    {output_dir / 'verification_summary.csv'}")
    print(f"Summary: PASS={len(joints) - warn_count - fail_count} WARN={warn_count} FAIL={fail_count}")
    return 2 if fail_count else 0


def main() -> int:
    return run()


def main_model_zero() -> int:
    return run(REFERENCE_MODEL_ZERO)


def main_policy_default() -> int:
    return run(REFERENCE_POLICY_DEFAULT)


if __name__ == "__main__":
    raise SystemExit(main())
