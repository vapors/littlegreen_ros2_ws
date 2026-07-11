#!/usr/bin/env python3
"""Interactive console for the guarded ST3215 default-pose move.

This helper starts /st3215_driver/move_to_default_pose and watches stdin in
cbreak mode. SPACE, q/Q, a/A, or ESC requests /st3215_driver/abort_pose_move.
Ctrl+C also requests an abort before exiting.

The abort service is a software motion abort/hold. It is NOT a hardware
power-cut or torque-disable emergency stop.
"""

from __future__ import annotations

import argparse
import select
import sys
import termios
import time
import tty
from typing import Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from std_srvs.srv import Trigger


class DefaultPoseMoveConsole(Node):
    def __init__(
        self,
        move_service: str,
        abort_service: str,
        diagnostics_topic: str,
    ) -> None:
        super().__init__("st3215_default_pose_move_console")
        self.move_client = self.create_client(Trigger, move_service)
        self.abort_client = self.create_client(Trigger, abort_service)
        self.create_subscription(
            DiagnosticArray,
            diagnostics_topic,
            self._diagnostics_callback,
            10,
        )

        self.pose_move_running: Optional[bool] = None
        self.pose_override_active: Optional[bool] = None
        self.last_diag_monotonic = 0.0

    def _diagnostics_callback(self, msg: DiagnosticArray) -> None:
        for status in msg.status:
            if status.name != "ST3215 native single bus":
                continue
            values = {item.key: item.value for item in status.values}
            if "pose_move_running" in values:
                self.pose_move_running = values["pose_move_running"].lower() == "true"
            if "pose_override_active" in values:
                self.pose_override_active = values["pose_override_active"].lower() == "true"
            self.last_diag_monotonic = time.monotonic()
            break

    def wait_for_services(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if self.move_client.service_is_ready() and self.abort_client.service_is_ready():
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def call_trigger(self, client, timeout_sec: float = 3.0):
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

        if not future.done():
            return None
        try:
            return future.result()
        except Exception as exc:  # pragma: no cover - runtime transport error path
            self.get_logger().error(f"Service call failed: {exc}")
            return None


def read_key_nonblocking() -> Optional[str]:
    readable, _, _ = select.select([sys.stdin], [], [], 0.0)
    if not readable:
        return None
    return sys.stdin.read(1)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Start the guarded default-pose move with keyboard abort support."
    )
    parser.add_argument(
        "--move-service",
        default="/st3215_driver/move_to_default_pose",
    )
    parser.add_argument(
        "--abort-service",
        default="/st3215_driver/abort_pose_move",
    )
    parser.add_argument(
        "--diagnostics-topic",
        default="/st3215_driver/diagnostics",
    )
    parser.add_argument(
        "--service-wait-sec",
        type=float,
        default=5.0,
    )

    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)

    node = DefaultPoseMoveConsole(
        move_service=args.move_service,
        abort_service=args.abort_service,
        diagnostics_topic=args.diagnostics_topic,
    )

    old_termios = None
    abort_requested = False
    move_started = False

    try:
        print("\nST3215 guarded default-pose move console")
        print("========================================")
        print("This console sends REAL servo commands through the driver.")
        print("The robot must be securely supported and a hardware power disconnect kept ready.")
        print("\nKeyboard controls during the ramp:")
        print("  SPACE   abort ramp and hold latest measured pose")
        print("  q / Q   abort ramp and hold latest measured pose")
        print("  a / A   abort ramp and hold latest measured pose")
        print("  ESC     abort ramp and hold latest measured pose")
        print("  Ctrl+C  request abort, then exit")
        print("\nIMPORTANT: this is a software motion abort/hold, NOT a hardware torque-off E-stop.\n")

        if not node.wait_for_services(args.service_wait_sec):
            print("Required move/abort services are not available.", file=sys.stderr)
            return 2

        confirmation = input("Type MOVE exactly to start the default-pose ramp: ").strip()
        if confirmation != "MOVE":
            print("Cancelled. No move request was sent.")
            return 0

        response = node.call_trigger(node.move_client)
        if response is None:
            print("Move service timed out or failed.", file=sys.stderr)
            return 3
        if not response.success:
            print(f"Move rejected: {response.message}", file=sys.stderr)
            return 4

        move_started = True
        print(f"\nMove started: {response.message}")
        print("\n>>> PRESS SPACE, q, a, OR ESC TO ABORT <<<\n")

        if not sys.stdin.isatty():
            print("stdin is not a TTY; keyboard abort monitoring is unavailable.", file=sys.stderr)
            print(f"Use: ros2 service call {args.abort_service} std_srvs/srv/Trigger '{{}}'", file=sys.stderr)
        else:
            old_termios = termios.tcgetattr(sys.stdin.fileno())
            tty.setcbreak(sys.stdin.fileno())

        saw_post_start_diagnostics = False
        start_monotonic = time.monotonic()

        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)

            key = read_key_nonblocking() if old_termios is not None else None
            if key in (" ", "q", "Q", "a", "A", "\x1b"):
                print("\nAbort key detected. Requesting software pose abort...")
                abort_requested = True
                abort_response = node.call_trigger(node.abort_client)
                if abort_response is None:
                    print("ABORT SERVICE DID NOT RESPOND. USE HARDWARE POWER DISCONNECT IF NEEDED.", file=sys.stderr)
                    return 5
                print(f"Abort response: success={abort_response.success} message={abort_response.message}")
                return 0 if abort_response.success else 6

            if node.last_diag_monotonic >= start_monotonic:
                saw_post_start_diagnostics = True

            if saw_post_start_diagnostics and node.pose_move_running is False:
                if node.pose_override_active:
                    print("\nDefault-pose ramp complete. Pose override remains active and is holding the pose.")
                else:
                    print("\nDefault-pose ramp complete. Pose override is not active.")
                return 0

    except KeyboardInterrupt:
        print("\nCtrl+C detected.")
        if move_started and not abort_requested:
            print("Requesting software pose abort before exit...")
            response = node.call_trigger(node.abort_client)
            if response is None:
                print("ABORT SERVICE DID NOT RESPOND. USE HARDWARE POWER DISCONNECT IF NEEDED.", file=sys.stderr)
                return 7
            print(f"Abort response: success={response.success} message={response.message}")
        return 130

    finally:
        if old_termios is not None:
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old_termios)
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
