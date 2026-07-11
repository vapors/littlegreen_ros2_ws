#!/usr/bin/env python3
"""Review and explicitly apply a center_step proposal to servo_map.yaml.

Default behavior is dry-run: validate proposal/map identity and print a unified
diff.  The source map is modified only when --apply is supplied.  A timestamped
backup is always created before an applied edit.
"""

from __future__ import annotations

import argparse
import difflib
import os
import sys
from datetime import datetime
from pathlib import Path

import yaml

from lgh_st3215_tools.calibration_common import (
    centers_from_proposal,
    load_servo_map,
    patch_center_steps_text,
    sha256_file,
    validate_proposal_against_map,
)



def main() -> int:
    parser = argparse.ArgumentParser(
        description="Validate and apply a captured ST3215 center-step calibration proposal."
    )
    parser.add_argument("proposal", type=Path)
    parser.add_argument("--source-servo-map", dest="servo_map", type=Path, required=True, help="Explicit source-tree lgh_st3215_driver/config/servo_map.yaml to review or modify.")
    parser.add_argument("--apply", action="store_true", help="Actually modify servo_map.yaml")
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
        print("Pass --source-servo-map explicitly.", file=sys.stderr)
        return 5

    proposal = yaml.safe_load(proposal_path.read_text())
    if not isinstance(proposal, dict):
        print("Proposal YAML root is invalid", file=sys.stderr)
        return 5

    _, joints = load_servo_map(map_path)
    validate_proposal_against_map(proposal, joints)

    current_hash = sha256_file(map_path)
    expected_hash = str(proposal.get("source_servo_map_sha256", ""))
    if current_hash != expected_hash and not args.allow_map_mismatch:
        print("REFUSED: source servo_map SHA-256 does not match the captured map.", file=sys.stderr)
        print(f"  proposal expected: {expected_hash}", file=sys.stderr)
        print(f"  current map:       {current_hash}", file=sys.stderr)
        print("Re-capture calibration or use --allow-map-mismatch only after careful review.", file=sys.stderr)
        return 3

    blocking = []
    for item in proposal.get("joints", []):
        status = str(item.get("status", ""))
        if status in ("RANGE_CONFLICT", "UNSTABLE_CAPTURE"):
            blocking.append(f"{item['name']}: {status}")
        if status == "MECHANICAL_REINDEX_RECOMMENDED" and not args.allow_large_corrections:
            blocking.append(f"{item['name']}: {status}")

    if blocking:
        print("REFUSED: proposal contains blocking flags:", file=sys.stderr)
        for item in blocking:
            print(f"  {item}", file=sys.stderr)
        print(
            "Re-index/recapture, or use --allow-large-corrections only for reviewed "
            "MECHANICAL_REINDEX_RECOMMENDED entries.",
            file=sys.stderr,
        )
        return 3

    centers = centers_from_proposal(proposal)
    old_text = map_path.read_text()
    new_text = patch_center_steps_text(old_text, centers)

    diff = "".join(
        difflib.unified_diff(
            old_text.splitlines(keepends=True),
            new_text.splitlines(keepends=True),
            fromfile=str(map_path),
            tofile=str(map_path) + " (proposed)",
        )
    )

    print("\nValidated calibration diff")
    print("==========================")
    print(diff if diff else "No center_step changes are required.")

    if not args.apply:
        print("\nDry-run only. Re-run with --apply after reviewing the diff.")
        return 0

    if not diff:
        print("No changes to apply.")
        return 0

    timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    backup_path = map_path.with_name(map_path.name + f".backup-{timestamp}")
    backup_path.write_text(old_text)

    temp_path = map_path.with_name(map_path.name + ".tmp-calibration")
    temp_path.write_text(new_text)
    # Parse the candidate before replacing the live source map.
    load_servo_map(temp_path)
    os.replace(temp_path, map_path)

    print(f"\nApplied calibration to: {map_path}")
    print(f"Backup written to:      {backup_path}")
    print(f"New map SHA-256:        {sha256_file(map_path)}")
    print("\nRebuild/relaunch the driver before verification.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
