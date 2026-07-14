#!/usr/bin/env python3
"""Capture a read-only ST3215 center-step calibration proposal.

The recommended reference is model zero: physically align the robot so every
actuated joint is at joint_zero_rad (currently 0 rad), then capture center_step.
The older policy-default reference remains available explicitly for fixture-based
work, but is no longer the default calibration language.

The tool never writes servo EEPROM and never modifies source YAML directly.
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
    RAW_STEP_MAX,
    RAW_STEP_MIN,
    REFERENCE_CHOICES,
    REFERENCE_MODEL_ZERO,
    REFERENCE_POLICY_DEFAULT,
    STEPS_PER_RADIAN,
    classify_correction,
    derived_raw_limits,
    expected_step_for_reference,
    format_pose_reference,
    load_servo_map,
    patch_servo_map_calibration_text,
    proposed_center_step_for_reference,
    raw_limits_valid,
    reference_angle_rad,
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
        super().__init__("st3215_center_step_calibration_capture")
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
        if any(value < RAW_STEP_MIN or value > RAW_STEP_MAX for value in values):
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
            raise RuntimeError("A policy-default pose ramp is running; calibration capture is blocked")
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


def _selected_joints(all_joints, requested_names: list[str]):
    if not requested_names:
        return list(all_joints)
    by_name = {joint.name: joint for joint in all_joints}
    unknown = sorted(set(requested_names) - set(by_name))
    if unknown:
        raise ValueError("Unknown --joint name(s): " + ", ".join(unknown))
    # Preserve canonical policy order and ignore accidental duplicates.
    requested = set(requested_names)
    return [joint for joint in all_joints if joint.name in requested]


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Capture a read-only ST3215 center-step calibration proposal. "
            "The recommended reference is model-zero."
        )
    )
    parser.add_argument("--servo-map", type=Path, default=None)
    parser.add_argument(
        "--reference",
        choices=REFERENCE_CHOICES,
        default=REFERENCE_MODEL_ZERO,
        help=(
            "Physical reference used to infer center_step. model-zero is the "
            "recommended replacement-servo workflow; policy-default retains the "
            "older fixture-based behavior."
        ),
    )
    parser.add_argument(
        "--joint",
        action="append",
        default=[],
        help="Capture only this joint; may be repeated. Omit to capture all 12 joints.",
    )
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
    _, all_joints = load_servo_map(map_path)
    try:
        joints = _selected_joints(all_joints, args.joint)
    except ValueError as exc:
        parser.error(str(exc))
    source_hash = sha256_file(map_path)

    reference_title = (
        "model-zero" if args.reference == REFERENCE_MODEL_ZERO else "policy-default"
    )
    print(f"\nST3215 {reference_title} center-step calibration capture")
    print("=" * (47 + len(reference_title)))
    print(f"Servo map: {map_path}")
    print(f"Map SHA-256: {source_hash}")
    print(f"Selected joints: {len(joints)}/{len(all_joints)}")
    print()
    print(format_pose_reference(joints, args.reference))
    print()
    if args.reference == REFERENCE_MODEL_ZERO:
        print(
            "PRECONDITION: physically align each selected joint to MODEL ZERO "
            "(joint_zero_rad; currently 0 rad)."
        )
        print(
            "At model zero, the measured raw position becomes center_step. "
            "The policy-default stance is a separate commanded pose."
        )
    else:
        print(
            "PRECONDITION: physically align each selected joint to the exact POLICY-DEFAULT pose."
        )
        print(
            "This compatibility mode infers model-zero centers from the policy-default fixture."
        )
    print(
        "Existing min_rad/max_rad are preserved. New min_step/max_step values are "
        "derived from the proposed center."
    )
    print("This tool is READ-ONLY. It does not write servo EEPROM or edit source YAML.")
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
    column_by_policy_index = {
        policy_index: [int(value) for value in columns[policy_index]]
        for policy_index in range(len(columns))
    }
    timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    output_dir = args.output_dir.expanduser().resolve() / timestamp
    output_dir.mkdir(parents=True, exist_ok=False)

    proposal_joints = []
    text_rows = []
    updates_by_name: dict[str, dict[str, int]] = {}
    blocking_flags = 0
    review_flags = 0

    for joint in joints:
        sample_values = column_by_policy_index[joint.policy_index]
        measured_median = float(statistics.median(sample_values))
        sample_min = min(sample_values)
        sample_max = max(sample_values)
        sample_span = sample_max - sample_min
        sample_stdev = statistics.pstdev(sample_values)

        proposed_center = proposed_center_step_for_reference(
            joint, measured_median, args.reference
        )
        correction = proposed_center - joint.center_step
        derived_min_step, derived_max_step = derived_raw_limits(joint, proposed_center)
        derived_ok = raw_limits_valid(derived_min_step, derived_max_step)
        status = classify_correction(
            correction,
            args.fine_threshold_steps,
            args.inspect_threshold_steps,
            derived_ok,
        )

        if sample_span > args.max_sample_span_steps:
            status = "UNSTABLE_CAPTURE"
        if status in (
            "RAW_RANGE_OUT_OF_BOUNDS",
            "MECHANICAL_REINDEX_RECOMMENDED",
            "UNSTABLE_CAPTURE",
        ):
            blocking_flags += 1
        elif status == "INSPECT_MECHANICAL_ALIGNMENT":
            review_flags += 1

        reference_rad = reference_angle_rad(joint, args.reference)
        mapped_rad_before = steps_to_radians(joint, measured_median)
        reference_error_before = mapped_rad_before - reference_rad
        expected_old = expected_step_for_reference(joint, args.reference)
        expected_new = expected_step_for_reference(
            joint, args.reference, proposed_center
        )

        updates_by_name[joint.name] = {
            "center_step": proposed_center,
            "min_step": derived_min_step,
            "max_step": derived_max_step,
        }
        proposal_joints.append(
            {
                "name": joint.name,
                "policy_index": joint.policy_index,
                "servo_id": joint.servo_id,
                "servo_sign": joint.servo_sign,
                "joint_zero_rad": joint.joint_zero_rad,
                "training_default_rad": joint.training_default_rad,
                "min_rad": joint.min_rad,
                "max_rad": joint.max_rad,
                "reference": args.reference,
                "reference_rad": reference_rad,
                "sample_count": len(sample_values),
                "measured_reference_step_median": measured_median,
                "measured_step_min": sample_min,
                "measured_step_max": sample_max,
                "measured_step_span": sample_span,
                "measured_step_stdev": sample_stdev,
                "old_center_step": joint.center_step,
                "proposed_center_step": proposed_center,
                "correction_steps": correction,
                "expected_reference_step_old_map": expected_old,
                "expected_reference_step_proposed_map": expected_new,
                "mapped_rad_before_calibration": mapped_rad_before,
                "reference_error_before_rad": reference_error_before,
                "reference_error_before_deg": math.degrees(reference_error_before),
                "old_min_step": joint.min_step,
                "old_max_step": joint.max_step,
                "derived_min_step": derived_min_step,
                "derived_max_step": derived_max_step,
                "derived_raw_range_inside_servo": derived_ok,
                "raw_servo_step_min": RAW_STEP_MIN,
                "raw_servo_step_max": RAW_STEP_MAX,
                "status": status,
            }
        )
        text_rows.append(
            f"{joint.policy_index:>2} ID{joint.servo_id:>2} {joint.name:<36} "
            f"meas={measured_median:>7.1f} old_center={joint.center_step:>4} "
            f"new_center={proposed_center:>4} corr={correction:>+5} "
            f"raw=[{derived_min_step:>4},{derived_max_step:>4}] "
            f"span={sample_span:>2} err={math.degrees(reference_error_before):>+7.2f}deg "
            f"{status}"
        )

    proposal = {
        "schema_version": 2,
        "calibration_type": f"{args.reference}_center_step",
        "reference": args.reference,
        "capture_timestamp_utc": timestamp,
        "source_servo_map": str(map_path),
        "source_servo_map_sha256": source_hash,
        "selected_joint_count": len(joints),
        "selected_joint_names": [joint.name for joint in joints],
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
        "review_flag_count": review_flags,
        "rejected_stale_samples": node.rejected_stale,
        "rejected_shape_samples": node.rejected_shape,
        "limit_policy": (
            "preserve model-space min_rad/max_rad and derive raw min_step/max_step "
            "from the proposed center_step"
        ),
        "joints": proposal_joints,
    }

    proposal_path = output_dir / "center_step_proposal.yaml"
    proposal_path.write_text(yaml.safe_dump(proposal, sort_keys=False))

    proposed_map_text = patch_servo_map_calibration_text(
        map_path.read_text(), updates_by_name
    )
    proposed_map_path = output_dir / "servo_map.proposed.yaml"
    proposed_map_path.write_text(proposed_map_text)

    csv_path = output_dir / "calibration_summary.csv"
    with csv_path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(proposal_joints[0].keys()))
        writer.writeheader()
        writer.writerows(proposal_joints)

    report_lines = [
        f"ST3215 {reference_title} center-step calibration proposal",
        "=" * 64,
        f"capture: {timestamp}",
        f"source map: {map_path}",
        f"source SHA-256: {source_hash}",
        f"selected joints: {len(joints)}",
        f"samples/joint: {args.samples}",
        f"blocking flags: {blocking_flags}",
        f"review flags: {review_flags}",
        "",
        *text_rows,
        "",
        "Interpretation:",
        "  NO_CHANGE                       no center change required",
        "  FINE_SOFTWARE_CORRECTION        small center_step refinement",
        "  INSPECT_MECHANICAL_ALIGNMENT    review horn/fixture before applying",
        "  MECHANICAL_REINDEX_RECOMMENDED  large correction; re-index and recapture",
        "  RAW_RANGE_OUT_OF_BOUNDS         derived raw endpoint is outside 0..4095",
        "  UNSTABLE_CAPTURE                joint moved too much during capture",
        "",
        "min_rad/max_rad are preserved. min_step/max_step are derived from the new center.",
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
            f"\nWARNING: {blocking_flags} selected joint(s) have blocking flags. "
            "Do not apply them without resolving the stated condition."
        )
        return 2
    if review_flags:
        print(
            f"\nREVIEW: {review_flags} selected joint(s) should have mechanical "
            "alignment reviewed before applying."
        )
    print(
        "\nCapture completed. Review the proposal, then use apply_calibration. "
        "Small corrections are not treated as physical-limit conflicts."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
