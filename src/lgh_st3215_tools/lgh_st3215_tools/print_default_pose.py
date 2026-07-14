#!/usr/bin/env python3
"""Print LittleGreen model-zero or policy-default pose references."""

from __future__ import annotations

import argparse
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from lgh_st3215_tools.calibration_common import (
    REFERENCE_CHOICES,
    REFERENCE_MODEL_ZERO,
    REFERENCE_POLICY_DEFAULT,
    format_pose_reference,
    load_servo_map,
)


def default_map_path() -> Path:
    return Path(get_package_share_directory("lgh_st3215_driver")) / "config" / "servo_map.yaml"


def run(default_reference: str | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Print model-zero or policy-default joint/servo references."
    )
    parser.add_argument("--servo-map", type=Path, default=None)
    parser.add_argument(
        "--reference",
        choices=REFERENCE_CHOICES,
        default=default_reference or REFERENCE_POLICY_DEFAULT,
    )
    args = parser.parse_args()

    map_path = args.servo_map.expanduser().resolve() if args.servo_map else default_map_path()
    _, joints = load_servo_map(map_path)

    print(f"Servo map: {map_path}")
    print(f"Reference: {args.reference}")
    print()
    print(format_pose_reference(joints, args.reference))
    print()
    if args.reference == REFERENCE_MODEL_ZERO:
        print(
            "MODEL ZERO is the physical calibration fixture pose. center_step is the "
            "raw position at joint_zero_rad; it is not required to equal 2048."
        )
    else:
        print(
            "POLICY DEFAULT is the Track 1 stance commanded by pose_console / "
            "assume_policy_default. It is derived from model-zero calibration."
        )
    return 0


def main() -> int:
    """Historical command name; now explicitly prints policy-default."""
    return run(REFERENCE_POLICY_DEFAULT)


def main_model_zero() -> int:
    return run(REFERENCE_MODEL_ZERO)


if __name__ == "__main__":
    raise SystemExit(main())
