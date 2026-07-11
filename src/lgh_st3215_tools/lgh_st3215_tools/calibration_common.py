#!/usr/bin/env python3
"""Shared calibration helpers for the LittleGreen ST3215 tools.

This module intentionally contains no ROS node logic.  It is used by the
capture, apply, verification, and pose-reference tools installed with the
native driver package.
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
        joints.append(joint)

    joints.sort(key=lambda item: item.policy_index)
    if len(joints) != EXPECTED_JOINTS:
        raise ValueError(f"Expected {EXPECTED_JOINTS} joints, found {len(joints)}")
    if [joint.policy_index for joint in joints] != list(range(EXPECTED_JOINTS)):
        raise ValueError("policy_index values must be contiguous 0..11")
    if len({joint.servo_id for joint in joints}) != EXPECTED_JOINTS:
        raise ValueError("Servo IDs are not unique")
    return root, joints


def expected_step_for_pose(joint: JointCalibrationConfig, center_step: int | None = None) -> int:
    center = joint.center_step if center_step is None else int(center_step)
    step = center + joint.servo_sign * (
        joint.training_default_rad - joint.joint_zero_rad
    ) * STEPS_PER_RADIAN
    return int(round(step))


def steps_to_radians(joint: JointCalibrationConfig, raw_step: float, center_step: int | None = None) -> float:
    center = joint.center_step if center_step is None else int(center_step)
    return joint.joint_zero_rad + (
        (float(raw_step) - float(center)) / float(joint.servo_sign)
    ) * RADIANS_PER_STEP


def proposed_center_step(joint: JointCalibrationConfig, measured_default_step: float) -> int:
    center = float(measured_default_step) - joint.servo_sign * (
        joint.training_default_rad - joint.joint_zero_rad
    ) * STEPS_PER_RADIAN
    return int(round(center))


def mapped_range_steps(joint: JointCalibrationConfig, center_step: int) -> tuple[float, float]:
    a = center_step + joint.servo_sign * (
        joint.min_rad - joint.joint_zero_rad
    ) * STEPS_PER_RADIAN
    b = center_step + joint.servo_sign * (
        joint.max_rad - joint.joint_zero_rad
    ) * STEPS_PER_RADIAN
    return min(a, b), max(a, b)


def classify_correction(
    correction_steps: int,
    fine_threshold_steps: int,
    inspect_threshold_steps: int,
    range_ok: bool,
) -> str:
    if not range_ok:
        return "RANGE_CONFLICT"
    magnitude = abs(correction_steps)
    if magnitude <= fine_threshold_steps:
        return "FINE_SOFTWARE_CORRECTION"
    if magnitude <= inspect_threshold_steps:
        return "INSPECT_MECHANICAL_ALIGNMENT"
    return "MECHANICAL_REINDEX_RECOMMENDED"


def patch_center_steps_text(
    original_text: str,
    centers_by_name: dict[str, int],
) -> str:
    """Replace center_step values while preserving comments and formatting."""
    lines = original_text.splitlines(keepends=True)
    current_joint: str | None = None
    replaced: set[str] = set()
    output: list[str] = []

    name_pattern = re.compile(r"^\s*-?\s*name:\s*([^#\n]+?)\s*(?:#.*)?$")
    center_pattern = re.compile(r"^(\s*center_step:\s*)([-+]?\d+)(\s*(?:#.*)?(?:\n)?)$")

    for line in lines:
        name_match = name_pattern.match(line.rstrip("\n"))
        if name_match:
            current_joint = name_match.group(1).strip().strip('"\'')

        center_match = center_pattern.match(line)
        if center_match and current_joint in centers_by_name:
            new_value = centers_by_name[current_joint]
            line = f"{center_match.group(1)}{new_value}{center_match.group(3)}"
            replaced.add(current_joint)
        output.append(line)

    missing = set(centers_by_name) - replaced
    if missing:
        raise ValueError(
            "Could not find center_step field for: " + ", ".join(sorted(missing))
        )
    return "".join(output)


def validate_proposal_against_map(
    proposal: dict[str, Any],
    joints: list[JointCalibrationConfig],
) -> None:
    proposal_joints = proposal.get("joints")
    if not isinstance(proposal_joints, list) or len(proposal_joints) != len(joints):
        raise ValueError("Proposal joint list is missing or has the wrong size")

    by_name = {str(item["name"]): item for item in proposal_joints}
    for joint in joints:
        if joint.name not in by_name:
            raise ValueError(f"Proposal is missing joint {joint.name}")
        item = by_name[joint.name]
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
        }.items():
            if not math.isclose(float(item[key]), expected, rel_tol=0.0, abs_tol=1e-9):
                raise ValueError(
                    f"Proposal mismatch for {joint.name}: {key}={item[key]} expected {expected}"
                )


def centers_from_proposal(proposal: dict[str, Any]) -> dict[str, int]:
    return {
        str(item["name"]): int(item["proposed_center_step"])
        for item in proposal["joints"]
    }


def format_pose_reference(joints: Iterable[JointCalibrationConfig]) -> str:
    rows = []
    header = (
        "idx  id  joint                               sign  default_rad  default_deg  "
        "center  expected_step"
    )
    rows.append(header)
    rows.append("-" * len(header))
    for joint in joints:
        rows.append(
            f"{joint.policy_index:>3}  {joint.servo_id:>2}  "
            f"{joint.name:<36} {joint.servo_sign:>+4}  "
            f"{joint.training_default_rad:>11.4f}  "
            f"{math.degrees(joint.training_default_rad):>11.2f}  "
            f"{joint.center_step:>6}  {expected_step_for_pose(joint):>13}"
        )
    return "\n".join(rows)
