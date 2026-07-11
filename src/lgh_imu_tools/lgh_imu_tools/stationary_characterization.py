#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
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
from lgh_imu_tools.imu_common import euler_deg, percentile, q_norm


class Collector(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("imu_stationary_characterization")
        self.rows: list[dict] = []
        self.create_subscription(Imu, topic, self._callback, qos_profile_sensor_data)

    def _callback(self, msg: Imu) -> None:
        quaternion = (
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
        )
        roll, pitch, yaw = euler_deg(quaternion)
        self.rows.append(
            {
                "monotonic_ns": time.monotonic_ns(),
                "stamp_ns": int(msg.header.stamp.sec) * 1_000_000_000
                + int(msg.header.stamp.nanosec),
                "qw": quaternion[0],
                "qx": quaternion[1],
                "qy": quaternion[2],
                "qz": quaternion[3],
                "roll_deg": roll,
                "pitch_deg": pitch,
                "yaw_deg": yaw,
                "gx": msg.angular_velocity.x,
                "gy": msg.angular_velocity.y,
                "gz": msg.angular_velocity.z,
                "ax": msg.linear_acceleration.x,
                "ay": msg.linear_acceleration.y,
                "az": msg.linear_acceleration.z,
                "q_norm": q_norm(quaternion),
            }
        )


def statistics_for(values: list[float]) -> dict:
    return {
        "mean": statistics.fmean(values),
        "std": statistics.stdev(values) if len(values) > 1 else 0.0,
        "p95_abs": percentile([abs(value) for value in values], 0.95),
        "p99_abs": percentile([abs(value) for value in values], 0.99),
        "min": min(values),
        "max": max(values),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Record stationary IMU bias, noise, and drift statistics.")
    parser.add_argument("--topic", default="/imu/data")
    parser.add_argument("--duration-sec", type=float, default=20.0)
    parser.add_argument(
        "--output-root", type=Path, default=Path.home() / ".ros" / "lgh_imu_datasets"
    )
    args = parser.parse_args()

    if args.duration_sec <= 0.0:
        print("duration must be positive", file=sys.stderr)
        return int(ExitCode.CONFIG_ERROR)

    output_dir = (
        args.output_root.expanduser()
        / f"{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}_imu_stationary"
    )
    output_dir.mkdir(parents=True, exist_ok=False)

    rclpy.init()
    node = Collector(args.topic)
    deadline = time.monotonic() + args.duration_sec
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()
        return int(ExitCode.INTERRUPTED_BY_SIGINT)

    if not node.rows:
        node.destroy_node()
        rclpy.shutdown()
        return int(ExitCode.TIMEOUT_OR_UNAVAILABLE)

    with (output_dir / "timeseries.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(node.rows[0].keys()))
        writer.writeheader()
        writer.writerows(node.rows)

    summary = {
        "schema_version": 1,
        "samples": len(node.rows),
        "requested_duration_sec": args.duration_sec,
        "gyro": {
            key: statistics_for([float(row[key]) for row in node.rows])
            for key in ("gx", "gy", "gz")
        },
        "accel": {
            key: statistics_for([float(row[key]) for row in node.rows])
            for key in ("ax", "ay", "az")
        },
        "orientation_deg": {
            key: statistics_for([float(row[key]) for row in node.rows])
            for key in ("roll_deg", "pitch_deg", "yaw_deg")
        },
        "quaternion_norm": statistics_for([float(row["q_norm"]) for row in node.rows]),
    }
    (output_dir / "summary.yaml").write_text(
        yaml.safe_dump(summary, sort_keys=False, width=140)
    )
    print(output_dir)

    node.destroy_node()
    rclpy.shutdown()
    return int(ExitCode.PASS)


if __name__ == "__main__":
    raise SystemExit(main())
