#!/usr/bin/env python3
from __future__ import annotations

import argparse
import statistics
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from lgh_imu_tools.exit_codes import ExitCode
from lgh_imu_tools.imu_common import (
    euler_deg,
    load_contract,
    mat_vec,
    projected_gravity_sensor,
)


class Collector(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("imu_orientation_audit")
        self.quaternions: list[tuple[float, float, float, float]] = []
        self.create_subscription(Imu, topic, self._callback, qos_profile_sensor_data)

    def _callback(self, msg: Imu) -> None:
        self.quaternions.append(
            (
                msg.orientation.w,
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
            )
        )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Capture a known robot orientation and audit projected-gravity axis/sign."
    )
    parser.add_argument("--pose", required=True)
    parser.add_argument("--contract", type=Path)
    parser.add_argument("--duration-sec", type=float, default=3.0)
    parser.add_argument("--expected-axis", choices=["x", "y", "z"])
    parser.add_argument("--expected-sign", choices=["positive", "negative"])
    parser.add_argument("--minimum-magnitude", type=float, default=0.5)
    parser.add_argument(
        "--output-root", type=Path, default=Path.home() / ".ros" / "lgh_imu_audits"
    )
    args = parser.parse_args()

    if args.duration_sec <= 0.0 or not 0.0 <= args.minimum_magnitude <= 1.0:
        print("duration must be positive and minimum magnitude must be in [0, 1]", file=sys.stderr)
        return int(ExitCode.CONFIG_ERROR)
    if (args.expected_axis is None) != (args.expected_sign is None):
        print("--expected-axis and --expected-sign must be supplied together", file=sys.stderr)
        return int(ExitCode.CONFIG_ERROR)

    try:
        config = load_contract(args.contract)
    except Exception as exc:
        print(f"IMU CONFIG ERROR: {exc}", file=sys.stderr)
        return int(ExitCode.CONFIG_ERROR)

    axis = args.expected_axis
    sign = args.expected_sign
    minimum_magnitude = args.minimum_magnitude
    if args.pose == "neutral" and axis is None:
        axis = "z"
        sign = "negative"
        minimum_magnitude = max(minimum_magnitude, 0.8)

    output_dir = (
        args.output_root.expanduser()
        / f"{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}_{args.pose}"
    )
    output_dir.mkdir(parents=True, exist_ok=False)

    rclpy.init()
    node = Collector(str(config["topic"]))
    deadline = time.monotonic() + args.duration_sec
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()
        return int(ExitCode.INTERRUPTED_BY_SIGINT)

    if not node.quaternions:
        node.destroy_node()
        rclpy.shutdown()
        return int(ExitCode.TIMEOUT_OR_UNAVAILABLE)

    matrix = [float(value) for value in config["imu_to_base_matrix"]]
    gravity_samples = [
        mat_vec(matrix, projected_gravity_sensor(quaternion))
        for quaternion in node.quaternions
    ]
    gravity = tuple(
        statistics.median(sample[index] for sample in gravity_samples) for index in range(3)
    )
    rpy = tuple(
        statistics.median(euler_deg(quaternion)[index] for quaternion in node.quaternions)
        for index in range(3)
    )

    passed = True
    message = "capture only; no expected axis/sign requested"
    if axis is not None and sign is not None:
        value = gravity["xyz".index(axis)]
        passed = (
            value >= minimum_magnitude
            if sign == "positive"
            else value <= -minimum_magnitude
        )
        message = (
            f"{axis}={value:.4f}, expected {sign} magnitude >= {minimum_magnitude}"
        )

    payload = {
        "schema_version": 1,
        "pose": args.pose,
        "samples": len(node.quaternions),
        "median_rpy_deg": list(rpy),
        "median_projected_gravity_base": list(gravity),
        "expectation": {
            "axis": axis,
            "sign": sign,
            "minimum_magnitude": minimum_magnitude,
            "result": "PASS" if passed else "FAIL",
            "message": message,
        },
        "contract": config,
    }
    report_path = output_dir / "orientation_audit.yaml"
    report_path.write_text(yaml.safe_dump(payload, sort_keys=False, width=140))

    print(f"IMU ORIENTATION AUDIT: {'PASS' if passed else 'FAIL'}")
    print(f"projected_gravity_base={gravity}")
    print(report_path)

    node.destroy_node()
    rclpy.shutdown()
    return int(ExitCode.PASS if passed else ExitCode.TEST_FAIL)


if __name__ == "__main__":
    raise SystemExit(main())
