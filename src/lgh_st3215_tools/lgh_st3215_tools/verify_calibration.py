#!/usr/bin/env python3
"""Compatibility entry point for model-zero calibration verification."""

from lgh_st3215_tools.verify_reference_pose import main_model_zero


def main() -> int:
    return main_model_zero()


if __name__ == "__main__":
    raise SystemExit(main())
