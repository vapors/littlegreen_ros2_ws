#!/usr/bin/env python3
"""Review and explicitly apply a center-step calibration proposal.

The durable physical contract is min_rad/max_rad. Applying a new center_step also
updates the derived raw min_step/max_step values so the same model-space limits are
preserved. The source joint-map mirror is synchronized automatically when it can be
found in the same LittleGreen workspace.
"""

from __future__ import annotations

import argparse
import difflib
import os
import sys
from datetime import datetime
from pathlib import Path
from typing import Optional

import yaml

from lgh_st3215_tools.calibration_common import (
    calibration_updates_from_proposal,
    load_servo_map,
    patch_joint_map_mirror_text,
    patch_servo_map_calibration_text,
    sha256_file,
    validate_proposal_against_map,
)


def _auto_joint_map_path(servo_map_path: Path) -> Optional[Path]:
    """Find the canonical source joint_map.yaml from a source-tree servo map."""
    try:
        workspace = servo_map_path.parents[3]
    except IndexError:
        return None
    candidate = (
        workspace
        / "src"
        / "littlegreen_biped_pkg"
        / "src"
        / "configs"
        / "joint_map.yaml"
    )
    return candidate if candidate.is_file() else None


def _unified_diff(old_text: str, new_text: str, path: Path, suffix: str) -> str:
    return "".join(
        difflib.unified_diff(
            old_text.splitlines(keepends=True),
            new_text.splitlines(keepends=True),
            fromfile=str(path),
            tofile=str(path) + suffix,
        )
    )


def _write_atomic(path: Path, text: str, label: str) -> Path:
    timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    backup_path = path.with_name(path.name + f".backup-{timestamp}")
    backup_path.write_text(path.read_text())
    temp_path = path.with_name(path.name + f".tmp-{label}")
    temp_path.write_text(text)
    yaml.safe_load(temp_path.read_text())
    os.replace(temp_path, path)
    return backup_path


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Validate and apply a LittleGreen ST3215 center-step calibration proposal. "
            "Model-space limits are preserved and raw limits are derived."
        )
    )
    parser.add_argument("proposal", type=Path)
    parser.add_argument(
        "--source-servo-map",
        dest="servo_map",
        type=Path,
        required=True,
        help="Explicit source-tree lgh_st3215_driver/config/servo_map.yaml",
    )
    parser.add_argument(
        "--source-joint-map",
        dest="joint_map",
        type=Path,
        default=None,
        help=(
            "Source littlegreen_biped_pkg joint_map.yaml mirror. If omitted, the "
            "tool auto-detects it from the workspace layout."
        ),
    )
    parser.add_argument(
        "--no-joint-map-sync",
        action="store_true",
        help="Do not update servo_center_step/servo_min_step/servo_max_step mirrors.",
    )
    parser.add_argument("--apply", action="store_true", help="Actually modify source YAML")
    parser.add_argument("--allow-map-mismatch", action="store_true")
    parser.add_argument("--allow-large-corrections", action="store_true")
    args = parser.parse_args()

    proposal_path = args.proposal.expanduser().resolve()
    map_path = args.servo_map.expanduser().resolve()

    if not proposal_path.is_file():
        print(f"Proposal not found: {proposal_path}", file=sys.stderr)
        return 5
    if not map_path.is_file():
        print(f"Source servo map not found: {map_path}", file=sys.stderr)
        return 5

    proposal = yaml.safe_load(proposal_path.read_text())
    if not isinstance(proposal, dict):
        print("Proposal YAML root is invalid", file=sys.stderr)
        return 5

    _, joints = load_servo_map(map_path)
    try:
        validate_proposal_against_map(proposal, joints)
    except Exception as exc:
        print(f"REFUSED: proposal/map validation failed: {exc}", file=sys.stderr)
        return 3

    current_hash = sha256_file(map_path)
    expected_hash = str(proposal.get("source_servo_map_sha256", ""))
    if current_hash != expected_hash and not args.allow_map_mismatch:
        print("REFUSED: source servo_map SHA-256 does not match the captured map.", file=sys.stderr)
        print(f"  proposal expected: {expected_hash}", file=sys.stderr)
        print(f"  current map:       {current_hash}", file=sys.stderr)
        print("Re-capture calibration or review --allow-map-mismatch carefully.", file=sys.stderr)
        return 3

    blocking: list[str] = []
    review: list[str] = []
    for item in proposal.get("joints", []):
        status = str(item.get("status", ""))
        name = str(item.get("name", "unknown"))
        if status in ("RAW_RANGE_OUT_OF_BOUNDS", "UNSTABLE_CAPTURE", "RANGE_CONFLICT"):
            blocking.append(f"{name}: {status}")
        elif status == "MECHANICAL_REINDEX_RECOMMENDED" and not args.allow_large_corrections:
            blocking.append(f"{name}: {status}")
        elif status == "INSPECT_MECHANICAL_ALIGNMENT":
            review.append(f"{name}: {status}")

    if blocking:
        print("REFUSED: proposal contains blocking flags:", file=sys.stderr)
        for item in blocking:
            print(f"  {item}", file=sys.stderr)
        print(
            "Resolve/re-capture these entries. --allow-large-corrections only overrides "
            "reviewed MECHANICAL_REINDEX_RECOMMENDED entries.",
            file=sys.stderr,
        )
        return 3

    updates = calibration_updates_from_proposal(proposal)
    old_servo_text = map_path.read_text()
    new_servo_text = patch_servo_map_calibration_text(old_servo_text, updates)
    # Parse with the full semantic loader before presenting/applying it.
    temp_check = map_path.with_name(map_path.name + ".tmp-validate-calibration")
    try:
        temp_check.write_text(new_servo_text)
        load_servo_map(temp_check)
    finally:
        temp_check.unlink(missing_ok=True)

    joint_map_path: Optional[Path] = None
    old_joint_text: Optional[str] = None
    new_joint_text: Optional[str] = None
    if not args.no_joint_map_sync:
        if args.joint_map is not None:
            joint_map_path = args.joint_map.expanduser().resolve()
        else:
            joint_map_path = _auto_joint_map_path(map_path)
        if joint_map_path is None:
            print(
                "WARN: source joint_map.yaml was not auto-detected; only servo_map.yaml "
                "will be updated. Pass --source-joint-map to synchronize the audit mirror."
            )
        elif not joint_map_path.is_file():
            print(f"Source joint map not found: {joint_map_path}", file=sys.stderr)
            return 5
        else:
            old_joint_text = joint_map_path.read_text()
            new_joint_text = patch_joint_map_mirror_text(old_joint_text, updates)
            yaml.safe_load(new_joint_text)

    servo_diff = _unified_diff(old_servo_text, new_servo_text, map_path, " (proposed)")
    print("\nValidated calibration diff: servo map")
    print("======================================")
    print(servo_diff if servo_diff else "No servo-map changes are required.")

    joint_diff = ""
    if joint_map_path is not None and old_joint_text is not None and new_joint_text is not None:
        joint_diff = _unified_diff(
            old_joint_text, new_joint_text, joint_map_path, " (proposed mirror)"
        )
        print("\nValidated calibration diff: joint-map mirror")
        print("=============================================")
        print(joint_diff if joint_diff else "No joint-map mirror changes are required.")

    print("\nCalibration semantics")
    print("=====================")
    print("  center_step: calibrated at model zero")
    print("  min_rad/max_rad: preserved physical model-space limits")
    print("  min_step/max_step: re-derived from center_step + model-space limits")
    if review:
        print("\nReview flags:")
        for item in review:
            print(f"  {item}")

    if not args.apply:
        print("\nDry-run only. Re-run with --apply after reviewing the diff.")
        return 0

    if not servo_diff and not joint_diff:
        print("No changes to apply.")
        return 0

    servo_backup = _write_atomic(map_path, new_servo_text, "calibration")
    joint_backup: Optional[Path] = None
    if joint_map_path is not None and new_joint_text is not None and joint_diff:
        joint_backup = _write_atomic(joint_map_path, new_joint_text, "calibration-mirror")

    print(f"\nApplied calibration to: {map_path}")
    print(f"Servo-map backup:       {servo_backup}")
    print(f"New map SHA-256:        {sha256_file(map_path)}")
    if joint_map_path is not None and joint_backup is not None:
        print(f"Synchronized mirror:    {joint_map_path}")
        print(f"Joint-map backup:       {joint_backup}")
    print("\nRebuild/relaunch the affected packages before verification:")
    print("  lgh_st3215_driver")
    if joint_map_path is not None:
        print("  littlegreen_biped_pkg")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
