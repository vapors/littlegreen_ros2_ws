#!/usr/bin/env python3
from __future__ import annotations

import argparse
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
from lgh_imu_tools.imu_common import finite, load_contract, percentile, q_norm


class Collector(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("imu_preflight")
        self.rows: list[dict] = []
        self.create_subscription(Imu, topic, self._callback, qos_profile_sensor_data)

    def _callback(self, msg: Imu) -> None:
        arrival_ns = self.get_clock().now().nanoseconds
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        self.rows.append(
            {
                "arrival_ns": arrival_ns,
                "stamp_ns": stamp_ns,
                "frame_id": msg.header.frame_id,
                "q": [msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z],
                "gyro": [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
                "accel": [
                    msg.linear_acceleration.x,
                    msg.linear_acceleration.y,
                    msg.linear_acceleration.z,
                ],
                "orientation_covariance": list(msg.orientation_covariance),
            }
        )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Validate the canonical /imu/data contract without knowing the sensor transport."
    )
    parser.add_argument("--contract", type=Path)
    parser.add_argument("--duration-sec", type=float, default=3.0)
    parser.add_argument("--timeout-sec", type=float, default=8.0)
    parser.add_argument(
        "--output-root", type=Path, default=Path.home() / ".ros" / "lgh_reports"
    )
    args = parser.parse_args()

    if args.duration_sec <= 0.0 or args.timeout_sec <= 0.0:
        print("duration and timeout must be positive", file=sys.stderr)
        return int(ExitCode.CONFIG_ERROR)
    if args.timeout_sec < args.duration_sec:
        print("timeout must be greater than or equal to duration", file=sys.stderr)
        return int(ExitCode.CONFIG_ERROR)

    try:
        config = load_contract(args.contract)
    except Exception as exc:
        print(f"IMU CONFIG ERROR: {exc}", file=sys.stderr)
        return int(ExitCode.CONFIG_ERROR)

    output_dir = (
        args.output_root.expanduser()
        / f"{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ')}_imu_preflight"
    )
    output_dir.mkdir(parents=True, exist_ok=False)

    rclpy.init()
    node = Collector(str(config["topic"]))
    deadline = time.monotonic() + args.timeout_sec
    capture_start: float | None = None

    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.rows and capture_start is None:
                capture_start = time.monotonic()
            if capture_start is not None and time.monotonic() - capture_start >= args.duration_sec:
                break
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()
        return int(ExitCode.INTERRUPTED_BY_SIGINT)

    rows = node.rows
    checks: list[dict] = []

    def check(name: str, passed: bool, message: str) -> None:
        checks.append(
            {"name": name, "status": "PASS" if passed else "FAIL", "message": message}
        )

    if len(rows) < 2:
        check("messages", False, f"only {len(rows)} messages received")
        code = ExitCode.TIMEOUT_OR_UNAVAILABLE
    else:
        duration = (rows[-1]["arrival_ns"] - rows[0]["arrival_ns"]) / 1e9
        rate_hz = (len(rows) - 1) / duration if duration > 0.0 else 0.0
        ages_ms = [
            (row["arrival_ns"] - row["stamp_ns"]) / 1e6
            for row in rows
            if row["stamp_ns"] > 0
        ]
        q_norms = [q_norm(row["q"]) for row in rows]
        stamp_deltas = [
            rows[index]["stamp_ns"] - rows[index - 1]["stamp_ns"]
            for index in range(1, len(rows))
        ]

        check("message_count", True, f"{len(rows)} messages")
        check(
            "rate",
            rate_hz >= float(config["minimum_rate_hz"]),
            f"{rate_hz:.3f} Hz; minimum {config['minimum_rate_hz']}",
        )
        expected_frame = str(config["expected_frame_id"])
        bad_frames = sorted(
            {row["frame_id"] for row in rows if row["frame_id"] != expected_frame}
        )
        check("frame_id", not bad_frames, f"expected {expected_frame}; unexpected={bad_frames}")

        if bool(config.get("require_orientation", True)):
            check(
                "finite_quaternion",
                all(finite(row["q"]) for row in rows),
                "all quaternion fields finite",
            )
            check(
                "quaternion_norm",
                max(abs(value - 1.0) for value in q_norms)
                <= float(config["quaternion_norm_tolerance"]),
                f"range {min(q_norms):.6f}..{max(q_norms):.6f}",
            )
            orientation_available = all(
                not row["orientation_covariance"]
                or row["orientation_covariance"][0] >= 0.0
                for row in rows
            )
            check(
                "orientation_available",
                orientation_available,
                "orientation covariance does not mark orientation unavailable",
            )

        if bool(config.get("require_angular_velocity", True)):
            check(
                "finite_angular_velocity",
                all(finite(row["gyro"]) for row in rows),
                "all gyro fields finite",
            )
        if bool(config.get("require_linear_acceleration", True)):
            check(
                "finite_linear_acceleration",
                all(finite(row["accel"]) for row in rows),
                "all acceleration fields finite",
            )

        check(
            "timestamp_progression",
            all(delta > 0 for delta in stamp_deltas),
            f"nonpositive deltas={sum(delta <= 0 for delta in stamp_deltas)}",
        )

        if ages_ms:
            maximum_age = max(ages_ms)
            minimum_age = min(ages_ms)
            age_ok = (
                minimum_age >= -5.0
                and maximum_age <= float(config["maximum_transport_age_ms"])
            )
            check(
                "transport_age",
                age_ok,
                f"min={minimum_age:.3f} ms, max={maximum_age:.3f} ms, "
                f"p99={percentile(ages_ms, 0.99):.3f} ms",
            )
        else:
            check("transport_age", False, "messages contained zero header stamps")

        code = ExitCode.PASS if all(c["status"] == "PASS" for c in checks) else ExitCode.TEST_FAIL

    payload = {
        "schema_version": 1,
        "tool": "imu_preflight",
        "status": "PASS" if code == ExitCode.PASS else "FAIL",
        "exit_code": int(code),
        "contract": config,
        "checks": checks,
        "sample_count": len(rows),
    }
    report_path = output_dir / "report.yaml"
    report_path.write_text(yaml.safe_dump(payload, sort_keys=False, width=140))
    (output_dir / "summary.txt").write_text(
        "\n".join(f"[{item['status']}] {item['name']}: {item['message']}" for item in checks)
        + "\n"
    )

    print(f"IMU PREFLIGHT: {payload['status']}")
    print(report_path)
    print(f"exit_code: {int(code)}")

    node.destroy_node()
    rclpy.shutdown()
    return int(code)


if __name__ == "__main__":
    raise SystemExit(main())
