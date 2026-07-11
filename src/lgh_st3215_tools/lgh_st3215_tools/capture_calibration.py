#!/usr/bin/env python3
"""Capture a read-only ST3215 center-step calibration proposal.

The robot must be physically positioned in the known Isaac training-default pose.
The tool collects raw servo step samples, computes proposed software center_step
values, and writes review artifacts.  It never modifies servo EEPROM and never
modifies servo_map.yaml.
"""

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
    STEPS_PER_RADIAN,
    classify_correction,
    expected_step_for_pose,
    format_pose_reference,
    load_servo_map,
    mapped_range_steps,
    patch_center_steps_text,
    proposed_center_step,
    sha256_file,
    steps_to_radians,
)


class CalibrationCaptureNode(Node):
    def __init__(
        self,
        raw_topic: str,
        age_topic: str,
        diagnostics_topic: str,
        sample_count: int,
        max_feedback_age_ms: int,
    ) -> None:
        super().__init__("st3215_default_pose_calibration_capture")
        self.sample_count = sample_count
        self.max_feedback_age_ms = max_feedback_age_ms
        self.samples: list[list[int]] = []
        self.latest_ages: Optional[list[int]] = None
        self.feedback_ready: Optional[bool] = None
        self.writes_enabled: Optional[bool] = None
        self.pose_move_running: Optional[bool] = None
        self.pose_override_active: Optional[bool] = None
        self.rejected_stale = 0
        self.rejected_shape = 0

        self.create_subscription(
            Int32MultiArray,
            raw_topic,
            self._raw_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            UInt32MultiArray,
            age_topic,
            self._age_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            DiagnosticArray,
            diagnostics_topic,
            self._diagnostics_callback,
            10,
        )

    def _age_callback(self, msg: UInt32MultiArray) -> None:
        if len(msg.data) == 12:
            self.latest_ages = [int(value) for value in msg.data]

    def _raw_callback(self, msg: Int32MultiArray) -> None:
        if len(self.samples) >= self.sample_count:
            return
        if len(msg.data) != 12:
            self.rejected_shape += 1
            return
        if self.latest_ages is None or len(self.latest_ages) != 12:
            self.rejected_stale += 1
            return
        if any(age > self.max_feedback_age_ms for age in self.latest_ages):
            self.rejected_stale += 1
            return
        values = [int(value) for value in msg.data]
        if any(value < 0 or value > 4095 for value in values):
            self.rejected_shape += 1
            return
        self.samples.append(values)

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
            if "pose_override_active" in values:
                self.pose_override_active = values["pose_override_active"].lower() == "true"
            break


def default_map_path() -> Path:
    return Path(get_package_share_directory("lgh_st3215_driver")) / "config" / "servo_map.yaml"


def wait_for_preflight(
    node: CalibrationCaptureNode,
    timeout_sec: float,
    allow_writes_enabled: bool,
) -> None:
    deadline = time.monotonic() + timeout_sec
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.feedback_ready is None or node.writes_enabled is None:
            continue
        if not node.feedback_ready:
            continue
        if node.pose_move_running:
            raise RuntimeError("A default-pose ramp is running; calibration capture is blocked")
        if node.writes_enabled and not allow_writes_enabled:
            raise RuntimeError(
                "Driver reports writes_enabled=true. Relaunch feedback-only or pass "
                "--allow-writes-enabled explicitly."
            )
        return
    raise RuntimeError("Timed out waiting for healthy driver diagnostics")


def collect_samples(
    node: CalibrationCaptureNode,
    timeout_sec: float,
    progress_every: int,
) -> None:
    deadline = time.monotonic() + timeout_sec
    next_progress = progress_every
    while rclpy.ok() and len(node.samples) < node.sample_count:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise RuntimeError(
                f"Timed out after collecting {len(node.samples)}/{node.sample_count} samples"
            )
        rclpy.spin_once(node, timeout_sec=min(0.1, remaining))
        if progress_every > 0 and len(node.samples) >= next_progress:
            print(f"  captured {len(node.samples)}/{node.sample_count}")
            next_progress += progress_every


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Capture a read-only default-pose center-step calibration proposal."
    )
    parser.add_argument("--servo-map", type=Path, default=None)
    parser.add_argument("--raw-topic", default="/st3215_driver/raw_position_steps")
    parser.add_argument("--age-topic", default="/joint_feedback_age_ms")
    parser.add_argument("--diagnostics-topic", default="/st3215_driver/diagnostics")
    parser.add_argument("--samples", type=int, default=250)
    parser.add_argument("--capture-timeout-sec", type=float, default=20.0)
    parser.add_argument("--preflight-timeout-sec", type=float, default=5.0)
    parser.add_argument("--max-feedback-age-ms", type=int, default=50)
    parser.add_argument("--fine-threshold-steps", type=int, default=25)
    parser.add_argument("--inspect-threshold-steps", type=int, default=100)
    parser.add_argument("--max-sample-span-steps", type=int, default=8)
    parser.add_argument("--output-dir", type=Path, default=Path("calibration_reports"))
    parser.add_argument("--allow-writes-enabled", action="store_true")
    parser.add_argument("--yes", action="store_true", help="Skip CAPTURE confirmation")
    args, ros_args = parser.parse_known_args()

    if args.samples < 10:
        parser.error("--samples must be at least 10")
    if args.fine_threshold_steps < 0 or args.inspect_threshold_steps < args.fine_threshold_steps:
        parser.error("Calibration thresholds are inconsistent")

    map_path = args.servo_map.expanduser().resolve() if args.servo_map else default_map_path()
    _, joints = load_servo_map(map_path)
    source_hash = sha256_file(map_path)

    print("\nST3215 training-default pose calibration capture")
    print("================================================")
    print(f"Servo map: {map_path}")
    print(f"Map SHA-256: {source_hash}")
    print()
    print(format_pose_reference(joints))
    print()
    print("PRECONDITION: the physical robot must be aligned to this training-default pose.")
    print("This tool is READ-ONLY. It does not write servo EEPROM or edit servo_map.yaml.")
    print()

    if not args.yes:
        confirmation = input("Type CAPTURE exactly when the robot is aligned and stable: ").strip()
        if confirmation != "CAPTURE":
            print("Cancelled. No calibration capture was performed.")
            return 0

    rclpy.init(args=ros_args)
    node = CalibrationCaptureNode(
        raw_topic=args.raw_topic,
        age_topic=args.age_topic,
        diagnostics_topic=args.diagnostics_topic,
        sample_count=args.samples,
        max_feedback_age_ms=args.max_feedback_age_ms,
    )

    try:
        wait_for_preflight(node, args.preflight_timeout_sec, args.allow_writes_enabled)
        print(f"Collecting {args.samples} raw position samples...")
        collect_samples(node, args.capture_timeout_sec, max(1, args.samples // 5))
    except Exception as exc:
        print(f"Calibration capture failed: {exc}", file=sys.stderr)
        return 4
    finally:
        node.destroy_node()
        rclpy.shutdown()

    columns = list(zip(*node.samples))
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    output_dir = args.output_dir.expanduser().resolve() / timestamp
    output_dir.mkdir(parents=True, exist_ok=False)

    proposal_joints = []
    text_rows = []
    centers_by_name: dict[str, int] = {}
    blocking_flags = 0

    for joint, samples in zip(joints, columns):
        sample_values = [int(value) for value in samples]
        measured_median = float(statistics.median(sample_values))
        sample_min = min(sample_values)
        sample_max = max(sample_values)
        sample_span = sample_max - sample_min
        sample_stdev = statistics.pstdev(sample_values)

        proposed_center = proposed_center_step(joint, measured_median)
        correction = proposed_center - joint.center_step
        range_lo, range_hi = mapped_range_steps(joint, proposed_center)
        low_margin = range_lo - joint.min_step
        high_margin = joint.max_step - range_hi
        range_ok = low_margin >= 0.0 and high_margin >= 0.0
        status = classify_correction(
            correction,
            args.fine_threshold_steps,
            args.inspect_threshold_steps,
            range_ok,
        )

        if sample_span > args.max_sample_span_steps:
            status = "UNSTABLE_CAPTURE"
        if status in ("RANGE_CONFLICT", "MECHANICAL_REINDEX_RECOMMENDED", "UNSTABLE_CAPTURE"):
            blocking_flags += 1

        mapped_rad_before = steps_to_radians(joint, measured_median)
        default_error_before = mapped_rad_before - joint.training_default_rad
        expected_old = expected_step_for_pose(joint)
        expected_new = expected_step_for_pose(joint, proposed_center)

        centers_by_name[joint.name] = proposed_center
        proposal_joints.append(
            {
                "name": joint.name,
                "policy_index": joint.policy_index,
                "servo_id": joint.servo_id,
                "servo_sign": joint.servo_sign,
                "joint_zero_rad": joint.joint_zero_rad,
                "training_default_rad": joint.training_default_rad,
                "sample_count": len(sample_values),
                "measured_default_step_median": measured_median,
                "measured_step_min": sample_min,
                "measured_step_max": sample_max,
                "measured_step_span": sample_span,
                "measured_step_stdev": sample_stdev,
                "old_center_step": joint.center_step,
                "proposed_center_step": proposed_center,
                "correction_steps": correction,
                "expected_default_step_old_map": expected_old,
                "expected_default_step_proposed_map": expected_new,
                "mapped_rad_before_calibration": mapped_rad_before,
                "default_pose_error_before_rad": default_error_before,
                "default_pose_error_before_deg": math.degrees(default_error_before),
                "mapped_range_low_step": range_lo,
                "mapped_range_high_step": range_hi,
                "range_low_margin_steps": low_margin,
                "range_high_margin_steps": high_margin,
                "range_ok": range_ok,
                "status": status,
            }
        )
        text_rows.append(
            f"{joint.policy_index:>2} ID{joint.servo_id:>2} {joint.name:<36} "
            f"meas={measured_median:>7.1f} old={joint.center_step:>4} "
            f"new={proposed_center:>4} corr={correction:>+5} "
            f"span={sample_span:>2} err={math.degrees(default_error_before):>+7.2f}deg "
            f"{status}"
        )

    proposal = {
        "schema_version": 1,
        "calibration_type": "training_default_pose_center_step",
        "capture_timestamp_utc": timestamp,
        "source_servo_map": str(map_path),
        "source_servo_map_sha256": source_hash,
        "sample_count": args.samples,
        "raw_topic": args.raw_topic,
        "feedback_age_topic": args.age_topic,
        "max_feedback_age_ms": args.max_feedback_age_ms,
        "steps_per_radian": STEPS_PER_RADIAN,
        "radians_per_step": RADIANS_PER_STEP,
        "fine_threshold_steps": args.fine_threshold_steps,
        "inspect_threshold_steps": args.inspect_threshold_steps,
        "max_sample_span_steps": args.max_sample_span_steps,
        "blocking_flag_count": blocking_flags,
        "rejected_stale_samples": node.rejected_stale,
        "rejected_shape_samples": node.rejected_shape,
        "joints": proposal_joints,
    }

    proposal_path = output_dir / "center_step_proposal.yaml"
    proposal_path.write_text(yaml.safe_dump(proposal, sort_keys=False))

    proposed_map_text = patch_center_steps_text(map_path.read_text(), centers_by_name)
    proposed_map_path = output_dir / "servo_map.proposed.yaml"
    proposed_map_path.write_text(proposed_map_text)

    csv_path = output_dir / "calibration_summary.csv"
    with csv_path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(proposal_joints[0].keys()))
        writer.writeheader()
        writer.writerows(proposal_joints)

    report_lines = [
        "ST3215 training-default calibration proposal",
        "============================================",
        f"capture: {timestamp}",
        f"source map: {map_path}",
        f"source SHA-256: {source_hash}",
        f"samples/joint: {args.samples}",
        f"blocking flags: {blocking_flags}",
        "",
        *text_rows,
        "",
        "Interpretation:",
        "  FINE_SOFTWARE_CORRECTION       suitable for center_step refinement",
        "  INSPECT_MECHANICAL_ALIGNMENT   review horn placement before applying",
        "  MECHANICAL_REINDEX_RECOMMENDED re-index horn, then capture again",
        "  RANGE_CONFLICT                  proposed map exceeds configured step range",
        "  UNSTABLE_CAPTURE                joint moved too much during capture",
        "",
        "This proposal has NOT modified the source servo_map.yaml.",
    ]
    report_path = output_dir / "calibration_report.txt"
    report_path.write_text("\n".join(report_lines) + "\n")

    print("\nCalibration proposal")
    print("====================")
    print("\n".join(text_rows))
    print()
    print(f"Proposal YAML: {proposal_path}")
    print(f"Proposed map:  {proposed_map_path}")
    print(f"CSV summary:   {csv_path}")
    print(f"Text report:   {report_path}")
    if blocking_flags:
        print(
            f"\nWARNING: {blocking_flags} joint(s) have blocking review flags. "
            "Do not apply blindly; inspect/re-index and recapture as needed."
        )
        return 2
    print("\nNo blocking calibration flags were found. Review the proposal before applying it.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
