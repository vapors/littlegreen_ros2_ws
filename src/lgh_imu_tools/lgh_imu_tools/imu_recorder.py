#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from lgh_imu_tools.exit_codes import ExitCode


class Recorder(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("imu_recorder")
        self.rows: list[list] = []
        self.create_subscription(Imu, topic, self._callback, qos_profile_sensor_data)

    def _callback(self, msg: Imu) -> None:
        self.rows.append(
            [
                time.time_ns(),
                time.monotonic_ns(),
                msg.header.stamp.sec,
                msg.header.stamp.nanosec,
                msg.header.frame_id,
                msg.orientation.w,
                msg.orientation.x,
                msg.orientation.y,
                msg.orientation.z,
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z,
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
            ]
        )


def main() -> int:
    parser = argparse.ArgumentParser(description="Record sensor_msgs/Imu samples to CSV.")
    parser.add_argument("--topic", default="/imu/data")
    parser.add_argument("--duration-sec", type=float, default=10.0)
    parser.add_argument(
        "--output-root", type=Path, default=Path.home() / ".ros" / "lgh_imu_datasets"
    )
    args = parser.parse_args()

    if args.duration_sec <= 0.0:
        print("duration must be positive", file=sys.stderr)
        return int(ExitCode.CONFIG_ERROR)

    output_dir = (
        args.output_root.expanduser()
        / f"{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}_imu"
    )
    output_dir.mkdir(parents=True, exist_ok=False)

    rclpy.init()
    node = Recorder(args.topic)
    deadline = time.monotonic() + args.duration_sec
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()
        return int(ExitCode.INTERRUPTED_BY_SIGINT)

    output_path = output_dir / "imu.csv"
    with output_path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "wall_time_ns",
                "monotonic_ns",
                "stamp_sec",
                "stamp_nanosec",
                "frame_id",
                "qw",
                "qx",
                "qy",
                "qz",
                "gx",
                "gy",
                "gz",
                "ax",
                "ay",
                "az",
            ]
        )
        writer.writerows(node.rows)

    print(output_path)
    code = ExitCode.PASS if node.rows else ExitCode.TIMEOUT_OR_UNAVAILABLE
    node.destroy_node()
    rclpy.shutdown()
    return int(code)


if __name__ == "__main__":
    raise SystemExit(main())
