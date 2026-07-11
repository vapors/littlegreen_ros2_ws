#!/usr/bin/env python3
"""Print the training-default joint angles and expected servo steps."""

from __future__ import annotations

import argparse
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from lgh_st3215_tools.calibration_common import format_pose_reference, load_servo_map


def default_map_path() -> Path:
    return Path(get_package_share_directory("lgh_st3215_driver")) / "config" / "servo_map.yaml"


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Print the physical training-default pose reference from servo_map.yaml."
    )
    parser.add_argument("--servo-map", type=Path, default=None)
    args = parser.parse_args()

    map_path = args.servo_map.expanduser().resolve() if args.servo_map else default_map_path()
    _, joints = load_servo_map(map_path)

    print(f"Servo map: {map_path}")
    print()
    print(format_pose_reference(joints))
    print()
    print(
        "Use these expected steps only for coarse mechanical horn indexing. "
        "Fine calibration should be captured from the physically aligned robot."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
