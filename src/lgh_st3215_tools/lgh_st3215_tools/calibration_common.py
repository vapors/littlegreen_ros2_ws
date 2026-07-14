#!/usr/bin/env python3
"""Shared calibration helpers for the LittleGreen ST3215 tools.

Terminology used throughout the current workflow:

* model zero: the physical pose where every actuated joint is at joint_zero_rad
  (currently 0 rad for all 12 joints). center_step is calibrated here.
* policy default: the Track 1 training/default stance stored as training_default_rad.
* model-space limits: min_rad/max_rad. These remain the durable physical contract.
* raw limits: min_step/max_step. These are derived deployment values that move with
  center_step while preserving the model-space limits.

This module intentionally contains no ROS node logic.
"""

from __future__ import annotations

import hashlib
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

import yaml

STEPS_PER_REVOLUTION = 4096.0
STEPS_PER_RADIAN = STEPS_PER_REVOLUTION / (2.0 * math.pi)
RADIANS_PER_STEP = (2.0 * math.pi) / STEPS_PER_REVOLUTION
EXPECTED_JOINTS = 12
RAW_STEP_MIN = 0
RAW_STEP_MAX = 4095
REFERENCE_MODEL_ZERO = "model-zero"
REFERENCE_POLICY_DEFAULT = "policy-default"
REFERENCE_CHOICES = (REFERENCE_MODEL_ZERO, REFERENCE_POLICY_DEFAULT)


@dataclass(frozen=True)
class JointCalibrationConfig:
    name: str
    policy_index: int
    servo_id: int
    servo_sign: int
    joint_zero_rad: float
    training_default_rad: float
    center_step: int
    min_rad: float
    max_rad: float
    min_step: int
    max_step: int


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_servo_map(path: Path) -> tuple[dict[str, Any], list[JointCalibrationConfig]]:
    root = yaml.safe_load(path.read_text())
    if not isinstance(root, dict):
        raise ValueError(f"Servo map root is not a mapping: {path}")
    joints_node = root.get("joints")
    if not isinstance(joints_node, list):
        raise ValueError(f"Servo map does not contain a joints list: {path}")

    joints: list[JointCalibrationConfig] = []
    for ordinal, node in enumerate(joints_node):
        if not isinstance(node, dict):
            raise ValueError(f"Joint entry {ordinal} is not a mapping")
        joint = JointCalibrationConfig(
            name=str(node["name"]),
            policy_index=int(node.get("policy_index", node.get("policy_action_index", ordinal))),
            servo_id=int(node["servo_id"]),
            servo_sign=int(node.get("servo_sign", 1)),
            joint_zero_rad=float(node.get("joint_zero_rad", 0.0)),
            training_default_rad=float(
                node.get("training_default_rad", node.get("default_joint_rad", 0.0))
            ),
            center_step=int(node.get("center_step", node.get("servo_center_step", 2048))),
            min_rad=float(node.get("min_rad", node.get("limit_lower_rad", -math.pi))),
            max_rad=float(node.get("max_rad", node.get("limit_upper_rad", math.pi))),
            min_step=int(node.get("min_step", node.get("servo_min_step", 0))),
            max_step=int(node.get("max_step", node.get("servo_max_step", 4095))),
        )
        if joint.servo_sign not in (-1, 1):
            raise ValueError(f"servo_sign must be +/-1 for {joint.name}")
        if joint.min_rad >= joint.max_rad:
            raise ValueError(f"min_rad must be below max_rad for {joint.name}")
        joints.append(joint)

    joints.sort(key=lambda item: item.policy_index)
    if len(joints) != EXPECTED_JOINTS:
        raise ValueError(f"Expected {EXPECTED_JOINTS} joints, found {len(joints)}")
    if [joint.policy_index for joint in joints] != list(range(EXPECTED_JOINTS)):
        raise ValueError("policy_index values must be contiguous 0..11")
    if len({joint.servo_id for joint in joints}) != EXPECTED_JOINTS:
        raise ValueError("Servo IDs are not unique")
    return root, joints


def reference_angle_rad(joint: JointCalibrationConfig, reference: str) -> float:
    if reference == REFERENCE_MODEL_ZERO:
        return joint.joint_zero_rad
    if reference == REFERENCE_POLICY_DEFAULT:
        return joint.training_default_rad
    raise ValueError(f"Unsupported calibration reference: {reference}")


def radians_to_steps(
    joint: JointCalibrationConfig,
    q_rad: float,
    center_step: int | None = None,
) -> float:
    center = joint.center_step if center_step is None else int(center_step)
    return center + joint.servo_sign * (
        float(q_rad) - joint.joint_zero_rad
    ) * STEPS_PER_RADIAN


def steps_to_radians(
    joint: JointCalibrationConfig,
    raw_step: float,
    center_step: int | None = None,
) -> float:
    center = joint.center_step if center_step is None else int(center_step)
    return joint.joint_zero_rad + (
        (float(raw_step) - float(center)) / float(joint.servo_sign)
    ) * RADIANS_PER_STEP


def expected_step_for_reference(
    joint: JointCalibrationConfig,
    reference: str,
    center_step: int | None = None,
) -> int:
    return int(round(radians_to_steps(joint, reference_angle_rad(joint, reference), center_step)))


def expected_step_for_pose(
    joint: JointCalibrationConfig,
    center_step: int | None = None,
) -> int:
    """Compatibility helper: the historical 'pose' is the policy-default pose."""
    return expected_step_for_reference(joint, REFERENCE_POLICY_DEFAULT, center_step)


def proposed_center_step_for_reference(
    joint: JointCalibrationConfig,
    measured_step: float,
    reference: str,
) -> int:
    reference_rad = reference_angle_rad(joint, reference)
    center = float(measured_step) - joint.servo_sign * (
        reference_rad - joint.joint_zero_rad
    ) * STEPS_PER_RADIAN
    return int(round(center))


def proposed_center_step(
    joint: JointCalibrationConfig,
    measured_default_step: float,
) -> int:
    """Compatibility helper for historical policy-default capture."""
    return proposed_center_step_for_reference(
        joint, measured_default_step, REFERENCE_POLICY_DEFAULT
    )


def derived_raw_limits(
    joint: JointCalibrationConfig,
    center_step: int,
) -> tuple[int, int]:
    """Derive raw deployment endpoints while preserving min_rad/max_rad."""
    a = int(round(radians_to_steps(joint, joint.min_rad, center_step)))
    b = int(round(radians_to_steps(joint, joint.max_rad, center_step)))
    return min(a, b), max(a, b)


def raw_limits_valid(min_step: int, max_step: int) -> bool:
    return (
        RAW_STEP_MIN <= int(min_step) < int(max_step) <= RAW_STEP_MAX
    )


def mapped_range_steps(
    joint: JointCalibrationConfig,
    center_step: int,
) -> tuple[float, float]:
    """Compatibility helper returning floating-point derived raw endpoints."""
    a = radians_to_steps(joint, joint.min_rad, center_step)
    b = radians_to_steps(joint, joint.max_rad, center_step)
    return min(a, b), max(a, b)


def classify_correction(
    correction_steps: int,
    fine_threshold_steps: int,
    inspect_threshold_steps: int,
    raw_range_ok: bool = True,
) -> str:
    if not raw_range_ok:
        return "RAW_RANGE_OUT_OF_BOUNDS"
    magnitude = abs(int(correction_steps))
    if magnitude == 0:
        return "NO_CHANGE"
    if magnitude <= fine_threshold_steps:
        return "FINE_SOFTWARE_CORRECTION"
    if magnitude <= inspect_threshold_steps:
        return "INSPECT_MECHANICAL_ALIGNMENT"
    return "MECHANICAL_REINDEX_RECOMMENDED"


def _patch_named_integer_fields(
    original_text: str,
    updates_by_name: dict[str, dict[str, int]],
) -> str:
    """Patch integer fields in YAML joint entries while preserving formatting/comments."""
    lines = original_text.splitlines(keepends=True)
    current_joint: str | None = None
    replaced: dict[str, set[str]] = {name: set() for name in updates_by_name}
    output: list[str] = []

    name_pattern = re.compile(r"^\s*-?\s*name:\s*([^#\n]+?)\s*(?:#.*)?$")

    for line in lines:
        name_match = name_pattern.match(line.rstrip("\n"))
        if name_match:
            current_joint = name_match.group(1).strip().strip('"\'')

        if current_joint in updates_by_name:
            for field, value in updates_by_name[current_joint].items():
                pattern = re.compile(
                    rf"^(\s*{re.escape(field)}:\s*)([-+]?\d+)(\s*(?:#.*)?(?:\n)?)$"
                )
                match = pattern.match(line)
                if match:
                    line = f"{match.group(1)}{int(value)}{match.group(3)}"
                    replaced[current_joint].add(field)
                    break
        output.append(line)

    missing: list[str] = []
    for name, fields in updates_by_name.items():
        not_found = set(fields) - replaced.get(name, set())
        for field in sorted(not_found):
            missing.append(f"{name}.{field}")
    if missing:
        raise ValueError("Could not find YAML field(s): " + ", ".join(missing))
    return "".join(output)


def patch_servo_map_calibration_text(
    original_text: str,
    updates_by_name: dict[str, dict[str, int]],
) -> str:
    allowed = {"center_step", "min_step", "max_step"}
    for name, fields in updates_by_name.items():
        unknown = set(fields) - allowed
        if unknown:
            raise ValueError(f"Unsupported servo-map fields for {name}: {sorted(unknown)}")
    return _patch_named_integer_fields(original_text, updates_by_name)


def patch_joint_map_mirror_text(
    original_text: str,
    updates_by_name: dict[str, dict[str, int]],
) -> str:
    translated: dict[str, dict[str, int]] = {}
    for name, fields in updates_by_name.items():
        translated[name] = {
            "servo_center_step": int(fields["center_step"]),
            "servo_min_step": int(fields["min_step"]),
            "servo_max_step": int(fields["max_step"]),
        }
    return _patch_named_integer_fields(original_text, translated)


def patch_center_steps_text(
    original_text: str,
    centers_by_name: dict[str, int],
) -> str:
    """Compatibility helper that patches only center_step."""
    return _patch_named_integer_fields(
        original_text,
        {name: {"center_step": value} for name, value in centers_by_name.items()},
    )


def validate_proposal_against_map(
    proposal: dict[str, Any],
    joints: list[JointCalibrationConfig],
) -> None:
    proposal_joints = proposal.get("joints")
    if not isinstance(proposal_joints, list) or not proposal_joints:
        raise ValueError("Proposal joint list is missing or empty")

    joints_by_name = {joint.name: joint for joint in joints}
    seen: set[str] = set()
    for item in proposal_joints:
        if not isinstance(item, dict) or "name" not in item:
            raise ValueError("Proposal contains an invalid joint entry")
        name = str(item["name"])
        if name in seen:
            raise ValueError(f"Proposal contains duplicate joint {name}")
        seen.add(name)
        if name not in joints_by_name:
            raise ValueError(f"Proposal contains unknown joint {name}")
        joint = joints_by_name[name]
        checks = {
            "policy_index": joint.policy_index,
            "servo_id": joint.servo_id,
            "servo_sign": joint.servo_sign,
        }
        for key, expected in checks.items():
            if int(item[key]) != int(expected):
                raise ValueError(
                    f"Proposal mismatch for {joint.name}: {key}={item[key]} expected {expected}"
                )
        for key, expected in {
            "joint_zero_rad": joint.joint_zero_rad,
            "training_default_rad": joint.training_default_rad,
            "min_rad": joint.min_rad,
            "max_rad": joint.max_rad,
        }.items():
            if key in item and not math.isclose(
                float(item[key]), expected, rel_tol=0.0, abs_tol=1e-9
            ):
                raise ValueError(
                    f"Proposal mismatch for {joint.name}: {key}={item[key]} expected {expected}"
                )


def calibration_updates_from_proposal(
    proposal: dict[str, Any],
) -> dict[str, dict[str, int]]:
    updates: dict[str, dict[str, int]] = {}
    for item in proposal["joints"]:
        updates[str(item["name"])] = {
            "center_step": int(item["proposed_center_step"]),
            "min_step": int(item["derived_min_step"]),
            "max_step": int(item["derived_max_step"]),
        }
    return updates


def centers_from_proposal(proposal: dict[str, Any]) -> dict[str, int]:
    return {
        name: fields["center_step"]
        for name, fields in calibration_updates_from_proposal(proposal).items()
    }


def format_pose_reference(
    joints: Iterable[JointCalibrationConfig],
    reference: str = REFERENCE_POLICY_DEFAULT,
) -> str:
    label = "zero_rad" if reference == REFERENCE_MODEL_ZERO else "policy_default_rad"
    header = (
        f"idx  id  joint                               sign  {label:<18} "
        "angle_deg  center  expected_step"
    )
    rows = [header, "-" * len(header)]
    for joint in joints:
        q = reference_angle_rad(joint, reference)
        rows.append(
            f"{joint.policy_index:>3}  {joint.servo_id:>2}  "
            f"{joint.name:<36} {joint.servo_sign:>+4}  "
            f"{q:>18.4f}  {math.degrees(q):>9.2f}  "
            f"{joint.center_step:>6}  "
            f"{expected_step_for_reference(joint, reference):>13}"
        )
    return "\n".join(rows)
