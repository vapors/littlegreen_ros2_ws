#!/usr/bin/env python3
"""Verify calibrated joint feedback against the known training-default pose."""

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
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt32MultiArray

from lgh_st3215_tools.calibration_common import STEPS_PER_RADIAN, load_servo_map, sha256_file


class VerificationNode(Node):
    def __init__(self, joint_topic: str, age_topic: str, diagnostics_topic: str, samples: int, max_age_ms: int) -> None:
        super().__init__("st3215_default_pose_calibration_verify")
        self.target_samples = samples
        self.max_age_ms = max_age_ms
        self.samples: list[list[float]] = []
        self.latest_ages: Optional[list[int]] = None
        self.feedback_ready: Optional[bool] = None
        self.writes_enabled: Optional[bool] = None
        self.rejected_stale = 0
        self.rejected_shape = 0

        self.create_subscription(JointState, joint_topic, self._joint_callback, qos_profile_sensor_data)
        self.create_subscription(UInt32MultiArray, age_topic, self._age_callback, qos_profile_sensor_data)
        self.create_subscription(DiagnosticArray, diagnostics_topic, self._diag_callback, 10)

    def _age_callback(self, msg: UInt32MultiArray) -> None:
        if len(msg.data) == 12:
            self.latest_ages = [int(value) for value in msg.data]

    def _joint_callback(self, msg: JointState) -> None:
        if len(self.samples) >= self.target_samples:
            return
        if len(msg.position) != 12:
            self.rejected_shape += 1
            return
        if self.latest_ages is None or any(age > self.max_age_ms for age in self.latest_ages):
            self.rejected_stale += 1
            return
        values = [float(value) for value in msg.position]
        if any(not math.isfinite(value) for value in values):
            self.rejected_shape += 1
            return
        self.samples.append(values)

    def _diag_callback(self, msg: DiagnosticArray) -> None:
        for status in msg.status:
            if status.name != "ST3215 native single bus":
                continue
            values = {entry.key: entry.value for entry in status.values}
            if "feedback_ready" in values:
                self.feedback_ready = values["feedback_ready"].lower() == "true"
            if "writes_enabled" in values:
                self.writes_enabled = values["writes_enabled"].lower() == "true"
            break


def default_map_path() -> Path:
    return Path(get_package_share_directory("lgh_st3215_driver")) / "config" / "servo_map.yaml"


def main() -> int:
    parser = argparse.ArgumentParser(description="Verify feedback against the training-default pose.")
    parser.add_argument("--servo-map", type=Path, default=None)
    parser.add_argument("--joint-topic", default="/joint_states")
    parser.add_argument("--age-topic", default="/joint_feedback_age_ms")
    parser.add_argument("--diagnostics-topic", default="/st3215_driver/diagnostics")
    parser.add_argument("--samples", type=int, default=250)
    parser.add_argument("--max-feedback-age-ms", type=int, default=50)
    parser.add_argument("--pass-tolerance-rad", type=float, default=0.02)
    parser.add_argument("--warn-tolerance-rad", type=float, default=0.05)
    parser.add_argument("--timeout-sec", type=float, default=20.0)
    parser.add_argument("--allow-writes-enabled", action="store_true")
    parser.add_argument("--output-dir", type=Path, default=Path("calibration_reports"))
    args, ros_args = parser.parse_known_args()

    if args.samples < 10:
        parser.error("--samples must be at least 10")
    if args.pass_tolerance_rad < 0 or args.warn_tolerance_rad < args.pass_tolerance_rad:
        parser.error("Tolerance values are inconsistent")

    map_path = args.servo_map.expanduser().resolve() if args.servo_map else default_map_path()
    _, joints = load_servo_map(map_path)

    rclpy.init(args=ros_args)
    node = VerificationNode(
        args.joint_topic,
        args.age_topic,
        args.diagnostics_topic,
        args.samples,
        args.max_feedback_age_ms,
    )

    try:
        preflight_deadline = time.monotonic() + 5.0
        while rclpy.ok() and time.monotonic() < preflight_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.feedback_ready is True and node.writes_enabled is not None:
                break
        if node.feedback_ready is not True:
            raise RuntimeError("Driver feedback is not ready")
        if node.writes_enabled and not args.allow_writes_enabled:
            raise RuntimeError(
                "Driver reports writes_enabled=true. Verification sequence expects feedback-only mode."
            )

        print(f"Collecting {args.samples} joint-state samples...")
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

    print("\nDefault-pose calibration verification")
    print("====================================")
    for joint, samples in zip(joints, columns):
        values = [float(value) for value in samples]
        median_q = statistics.median(values)
        min_q = min(values)
        max_q = max(values)
        error = median_q - joint.training_default_rad
        abs_error = abs(error)
        if abs_error <= args.pass_tolerance_rad:
            status = "PASS"
        elif abs_error <= args.warn_tolerance_rad:
            status = "WARN"
            warn_count += 1
        else:
            status = "FAIL"
            fail_count += 1
        equivalent_steps = error * joint.servo_sign * STEPS_PER_RADIAN
        row = {
            "name": joint.name,
            "policy_index": joint.policy_index,
            "servo_id": joint.servo_id,
            "training_default_rad": joint.training_default_rad,
            "measured_median_rad": median_q,
            "measured_min_rad": min_q,
            "measured_max_rad": max_q,
            "error_rad": error,
            "error_deg": math.degrees(error),
            "equivalent_center_error_steps": equivalent_steps,
            "status": status,
        }
        rows.append(row)
        print(
            f"{joint.policy_index:>2} ID{joint.servo_id:>2} {joint.name:<36} "
            f"target={joint.training_default_rad:>+7.3f} "
            f"meas={median_q:>+7.3f} err={error:>+7.4f}rad "
            f"({math.degrees(error):>+6.2f}deg) {status}"
        )

    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    output_dir = args.output_dir.expanduser().resolve() / f"verify-{timestamp}"
    output_dir.mkdir(parents=True, exist_ok=False)
    report = {
        "schema_version": 1,
        "verification_type": "training_default_pose_feedback",
        "timestamp_utc": timestamp,
        "servo_map": str(map_path),
        "servo_map_sha256": sha256_file(map_path),
        "sample_count": args.samples,
        "pass_tolerance_rad": args.pass_tolerance_rad,
        "warn_tolerance_rad": args.warn_tolerance_rad,
        "fail_count": fail_count,
        "warn_count": warn_count,
        "rejected_stale_samples": node.rejected_stale,
        "rejected_shape_samples": node.rejected_shape,
        "joints": rows,
    }
    (output_dir / "verification_report.yaml").write_text(yaml.safe_dump(report, sort_keys=False))
    with (output_dir / "verification_summary.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)

    print()
    print(f"Report: {output_dir / 'verification_report.yaml'}")
    print(f"CSV:    {output_dir / 'verification_summary.csv'}")
    print(f"Summary: PASS={12 - warn_count - fail_count} WARN={warn_count} FAIL={fail_count}")
    if fail_count:
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
