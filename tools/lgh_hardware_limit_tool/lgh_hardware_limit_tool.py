#!/usr/bin/env python3
"""Standalone physical joint-limit capture and contract generation for LittleGreen.

Version 1.1.0: LittleGreen paths, model-space limit authority, and center-independent rendering.

This tool intentionally has no ROS 2 dependency.  It talks directly to the ST3215
single bus, captures physical raw-step endpoints with torque disabled, applies a
configurable inward safety margin, and generates synchronized contract artifacts for:

  1. LittleGreen Track 1 training: track1_hardware_contract.generated.py
  2. littlegreen_ros2_ws deployment: servo_map.measured_limits.generated.yaml

The neutral source of truth is authoritative_hardware_contract.yaml.

Dependencies:
  - Python 3.10+
  - PyYAML (python3-yaml)

IMPORTANT:
  Stop the ROS 2 ST3215 driver before using `capture`.  This script must be the only
  process communicating with the servo UART.
"""

from __future__ import annotations

import argparse
import ast
import csv
import fcntl
import hashlib
import json
import math
import os
import select
import statistics
import sys
import termios
import time
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable

try:
    import yaml
except ImportError as exc:  # pragma: no cover - environment dependent
    raise SystemExit(
        "PyYAML is required. Install it with: sudo apt install python3-yaml"
    ) from exc


STEPS_PER_REVOLUTION = 4096.0
RADIANS_PER_STEP = 2.0 * math.pi / STEPS_PER_REVOLUTION
STEPS_PER_RADIAN = STEPS_PER_REVOLUTION / (2.0 * math.pi)
EXPECTED_JOINTS = 12

HEADER = 0xFF
BROADCAST_ID = 0xFE
INSTRUCTION_READ = 0x02
INSTRUCTION_WRITE = 0x03
TORQUE_ENABLE_ADDRESS = 0x28
PRESENT_POSITION_ADDRESS = 0x38

_DEFAULT_HOME = Path(os.environ["HOME"]) if os.environ.get("HOME") else Path(".")
DEFAULT_WORKSPACE = Path(os.environ.get("LITTLEGREEN_ROS2_WS", str(_DEFAULT_HOME / "littlegreen_ros2_ws")))
DEFAULT_SERVO_MAP = DEFAULT_WORKSPACE / "src" / "lgh_st3215_driver" / "config" / "servo_map.yaml"
DEFAULT_TRACK2_CONTRACT = DEFAULT_WORKSPACE / "src" / "lgh_st3215_tools" / "config" / "track1_action_contract_v4.yaml"


def resolve_cli_path(raw: str) -> Path:
    """Resolve a CLI path without unconditionally asking pathlib for a home directory."""
    text = os.path.expandvars(str(raw))
    if text == "~" or text.startswith("~/"):
        home = os.environ.get("HOME")
        if home:
            text = str(Path(home) / text[2:]) if text != "~" else home
        else:
            raise RuntimeError(
                f"Cannot expand path {raw!r}: HOME is not set. Use an absolute path."
            )
    return Path(os.path.abspath(text))


def resolve_optional_training_contract(raw: str | None) -> Path | None:
    if not raw:
        return None
    try:
        path = resolve_cli_path(raw)
    except Exception as exc:
        print(f"WARNING: deployment-contract path could not be resolved: {exc}")
        print("Continuing without deployment-contract comparison. Run `render` later on the Track 1/Track 2 host or with an absolute local path.")
        return None
    if not path.exists():
        print(f"WARNING: deployment-contract file not found: {path}")
        print("Continuing without deployment-contract comparison. Run `render` later with a reachable local file.")
        return None
    return path


@dataclass(frozen=True)
class JointMapEntry:
    name: str
    policy_index: int
    servo_id: int
    servo_sign: int
    joint_zero_rad: float
    training_default_rad: float
    center_step: int
    old_min_rad: float
    old_max_rad: float
    old_min_step: int
    old_max_step: int


@dataclass
class EndpointSample:
    label: str
    median_step: float
    rounded_step: int
    min_step: int
    max_step: int
    span_steps: int
    sample_count: int
    median_rad: float


class St3215Serial:
    """Minimal Linux termios transport for direct ST3215 access."""

    def __init__(self, path: str, baud: int = 1_000_000, timeout_s: float = 0.03):
        self.path = path
        self.baud = baud
        self.timeout_s = timeout_s
        self.fd: int | None = None
        self._rx = bytearray()

    def __enter__(self) -> "St3215Serial":
        self.open()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def open(self) -> None:
        if self.fd is not None:
            return
        flags = os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK
        self.fd = os.open(self.path, flags)
        try:
            # Advisory lock protects against a second instance of this tool.
            fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            os.close(self.fd)
            self.fd = None
            raise RuntimeError(f"Could not lock {self.path}; another limit tool may be using it")

        attrs = termios.tcgetattr(self.fd)
        # iflag, oflag, cflag, lflag
        attrs[0] = 0
        attrs[1] = 0
        attrs[2] = termios.CS8 | termios.CLOCAL | termios.CREAD
        attrs[3] = 0
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0

        baud_const_name = f"B{self.baud}"
        if not hasattr(termios, baud_const_name):
            self.close()
            raise RuntimeError(f"Python termios does not expose {baud_const_name} on this system")
        baud_const = getattr(termios, baud_const_name)
        attrs[4] = baud_const  # ispeed
        attrs[5] = baud_const  # ospeed
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)
        termios.tcflush(self.fd, termios.TCIOFLUSH)
        self._rx.clear()

    def close(self) -> None:
        if self.fd is not None:
            try:
                fcntl.flock(self.fd, fcntl.LOCK_UN)
            except OSError:
                pass
            os.close(self.fd)
            self.fd = None
        self._rx.clear()

    @staticmethod
    def _checksum(body: Iterable[int]) -> int:
        return (~sum(body)) & 0xFF

    @classmethod
    def build_packet(cls, servo_id: int, instruction: int, params: list[int]) -> bytes:
        length = len(params) + 2
        body = [servo_id, length, instruction, *params]
        return bytes([HEADER, HEADER, *body, cls._checksum(body)])

    def _write_all(self, data: bytes, timeout_s: float | None = None) -> None:
        if self.fd is None:
            raise RuntimeError("UART is not open")
        deadline = time.monotonic() + (self.timeout_s if timeout_s is None else timeout_s)
        offset = 0
        while offset < len(data):
            try:
                n = os.write(self.fd, data[offset:])
                if n > 0:
                    offset += n
                    continue
            except BlockingIOError:
                pass
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("UART write timeout")
            _, writable, _ = select.select([], [self.fd], [], remaining)
            if not writable:
                raise TimeoutError("UART write timeout")

    @staticmethod
    def _valid_checksum(frame: bytes) -> bool:
        return len(frame) >= 6 and (sum(frame[2:]) & 0xFF) == 0xFF

    def _extract_frame(self) -> bytes | None:
        while len(self._rx) >= 2 and not (self._rx[0] == HEADER and self._rx[1] == HEADER):
            del self._rx[0]
        if len(self._rx) < 4:
            return None
        frame_size = 4 + self._rx[3]
        if frame_size < 6 or frame_size > 260:
            del self._rx[0]
            return None
        if len(self._rx) < frame_size:
            return None
        frame = bytes(self._rx[:frame_size])
        del self._rx[:frame_size]
        return frame

    def _read_frame(self, expected_id: int, timeout_s: float | None = None) -> bytes:
        if self.fd is None:
            raise RuntimeError("UART is not open")
        deadline = time.monotonic() + (self.timeout_s if timeout_s is None else timeout_s)
        while time.monotonic() < deadline:
            candidate = self._extract_frame()
            if candidate is not None:
                if not self._valid_checksum(candidate):
                    raise RuntimeError("ST3215 reply checksum mismatch")
                if candidate[2] != expected_id:
                    raise RuntimeError(
                        f"Unexpected servo reply ID {candidate[2]}, expected {expected_id}"
                    )
                return candidate
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            readable, _, _ = select.select([self.fd], [], [], remaining)
            if not readable:
                break
            try:
                chunk = os.read(self.fd, 256)
            except BlockingIOError:
                continue
            if chunk:
                self._rx.extend(chunk)
        raise TimeoutError(f"Timed out waiting for servo {expected_id} reply")

    def flush_input(self) -> None:
        if self.fd is None:
            return
        self._rx.clear()
        termios.tcflush(self.fd, termios.TCIFLUSH)

    def read_bytes(self, servo_id: int, address: int, length: int) -> bytes:
        self.flush_input()
        packet = self.build_packet(servo_id, INSTRUCTION_READ, [address, length])
        self._write_all(packet)
        frame = self._read_frame(servo_id)
        expected = 6 + length
        if len(frame) != expected:
            raise RuntimeError(
                f"Servo {servo_id} reply length {len(frame)} != expected {expected}"
            )
        status = frame[4]
        if status != 0:
            raise RuntimeError(f"Servo {servo_id} returned status error 0x{status:02X}")
        return frame[5 : 5 + length]

    def read_position(self, servo_id: int) -> int:
        data = self.read_bytes(servo_id, PRESENT_POSITION_ADDRESS, 2)
        raw = data[0] | (data[1] << 8)
        magnitude = raw & 0x7FFF
        return -magnitude if raw & 0x8000 else magnitude

    def read_torque_enabled(self, servo_id: int) -> int:
        return int(self.read_bytes(servo_id, TORQUE_ENABLE_ADDRESS, 1)[0])

    def broadcast_torque(self, enabled: bool) -> None:
        packet = self.build_packet(
            BROADCAST_ID,
            INSTRUCTION_WRITE,
            [TORQUE_ENABLE_ADDRESS, 1 if enabled else 0],
        )
        self._write_all(packet)
        time.sleep(0.05)


# ------------------------------ map / conversion ------------------------------


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def load_servo_map(path: Path) -> tuple[dict[str, Any], list[JointMapEntry]]:
    root = yaml.safe_load(path.read_text())
    if not isinstance(root, dict) or not isinstance(root.get("joints"), list):
        raise ValueError(f"Invalid servo map: {path}")
    joints: list[JointMapEntry] = []
    for ordinal, item in enumerate(root["joints"]):
        joint = JointMapEntry(
            name=str(item["name"]),
            policy_index=int(item.get("policy_index", ordinal)),
            servo_id=int(item["servo_id"]),
            servo_sign=int(item.get("servo_sign", 1)),
            joint_zero_rad=float(item.get("joint_zero_rad", 0.0)),
            training_default_rad=float(item.get("training_default_rad", 0.0)),
            center_step=int(item.get("center_step", 2048)),
            old_min_rad=float(item.get("min_rad", -math.pi)),
            old_max_rad=float(item.get("max_rad", math.pi)),
            old_min_step=int(item.get("min_step", 0)),
            old_max_step=int(item.get("max_step", 4095)),
        )
        if joint.servo_sign not in (-1, 1):
            raise ValueError(f"servo_sign must be +/-1 for {joint.name}")
        joints.append(joint)
    joints.sort(key=lambda j: j.policy_index)
    if len(joints) != EXPECTED_JOINTS:
        raise ValueError(f"Expected {EXPECTED_JOINTS} joints, found {len(joints)}")
    if [j.policy_index for j in joints] != list(range(EXPECTED_JOINTS)):
        raise ValueError("policy_index must be contiguous 0..11")
    return root, joints


def steps_to_rad(joint: JointMapEntry, step: float) -> float:
    return joint.joint_zero_rad + (
        (float(step) - float(joint.center_step)) / float(joint.servo_sign)
    ) * RADIANS_PER_STEP


def rad_to_step(joint: JointMapEntry, q_rad: float) -> float:
    return joint.center_step + joint.servo_sign * (
        float(q_rad) - joint.joint_zero_rad
    ) * STEPS_PER_RADIAN


def expected_default_step(joint: JointMapEntry) -> int:
    return int(round(rad_to_step(joint, joint.training_default_rad)))


# ------------------------------ training contract ------------------------------


def _literal_assignment(tree: ast.Module, name: str) -> Any:
    for node in tree.body:
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name) and target.id == name:
                    return ast.literal_eval(node.value)
        elif isinstance(node, ast.AnnAssign):
            if isinstance(node.target, ast.Name) and node.target.id == name:
                return ast.literal_eval(node.value)
    raise KeyError(f"Could not find literal assignment {name}")


def _literal_assignment_any(tree: ast.Module, names: list[str]) -> tuple[str, Any]:
    for name in names:
        try:
            return name, _literal_assignment(tree, name)
        except KeyError:
            continue
    raise KeyError(f"Could not find any literal assignment from: {', '.join(names)}")


def load_deployment_contract(path: Path) -> dict[str, Any]:
    """Load the current LittleGreen YAML contract or a legacy Python contract."""
    if path.suffix.lower() in (".yaml", ".yml"):
        root = yaml.safe_load(path.read_text())
        if not isinstance(root, dict):
            raise ValueError(f"Invalid deployment contract YAML: {path}")
        names = root.get("joint_order")
        defaults = root.get("training_default_rad", root.get("action_default_rad"))
        lower = root.get("lower_limit_rad", root.get("action_target_lower_rad"))
        upper = root.get("upper_limit_rad", root.get("action_target_upper_rad"))
        if not all(isinstance(value, list) for value in (names, defaults, lower, upper)):
            raise ValueError(
                "Deployment contract YAML must provide joint_order, training_default_rad, "
                "lower_limit_rad, and upper_limit_rad"
            )
        return {
            "joint_order_symbol": "joint_order",
            "joint_order": list(names),
            "training_default_rad": list(defaults),
            "lower_limit_rad": list(lower),
            "upper_limit_rad": list(upper),
        }

    tree = ast.parse(path.read_text(), filename=str(path))
    joint_name_symbol, joint_order = _literal_assignment_any(
        tree, ["ACTIONABLE_JOINTS_V1_2_3", "ACTIONABLE_JOINTS_V1_2"]
    )
    return {
        "joint_order_symbol": joint_name_symbol,
        "joint_order": list(joint_order),
        "training_default_rad": list(_literal_assignment(tree, "TRAINING_DEFAULT_RAD")),
        "lower_limit_rad": list(_literal_assignment(tree, "HARDWARE_LOWER_LIMIT_RAD")),
        "upper_limit_rad": list(_literal_assignment(tree, "HARDWARE_UPPER_LIMIT_RAD")),
    }


# ------------------------------ capture helpers ------------------------------


def capture_endpoint(
    bus: St3215Serial,
    joint: JointMapEntry,
    label: str,
    sample_count: int,
    sample_rate_hz: float,
) -> EndpointSample:
    values: list[int] = []
    period = 1.0 / sample_rate_hz
    next_time = time.monotonic()
    for _ in range(sample_count):
        values.append(bus.read_position(joint.servo_id))
        next_time += period
        delay = next_time - time.monotonic()
        if delay > 0:
            time.sleep(delay)
    med = statistics.median(values)
    rounded = int(round(med))
    return EndpointSample(
        label=label,
        median_step=float(med),
        rounded_step=rounded,
        min_step=min(values),
        max_step=max(values),
        span_steps=max(values) - min(values),
        sample_count=len(values),
        median_rad=steps_to_rad(joint, med),
    )


def prompt_endpoint(joint: JointMapEntry, label: str) -> str:
    print()
    print(f"[{joint.policy_index + 1:02d}/12] {joint.name}  ID={joint.servo_id}  endpoint {label}")
    print("Move this joint slowly by hand to the physical endpoint. Do not force the mechanism.")
    while True:
        response = input("Press ENTER to capture, 's' to skip this joint, or 'q' to save and quit: ").strip().lower()
        if response in ("", "s", "q"):
            return response


def capture_record_from_endpoints(
    joint: JointMapEntry,
    endpoint_a: EndpointSample,
    endpoint_b: EndpointSample,
) -> dict[str, Any]:
    step_lo = min(endpoint_a.rounded_step, endpoint_b.rounded_step)
    step_hi = max(endpoint_a.rounded_step, endpoint_b.rounded_step)
    qa = steps_to_rad(joint, endpoint_a.median_step)
    qb = steps_to_rad(joint, endpoint_b.median_step)
    q_lo = min(qa, qb)
    q_hi = max(qa, qb)
    return {
        "name": joint.name,
        "policy_index": joint.policy_index,
        "servo_id": joint.servo_id,
        "servo_sign": joint.servo_sign,
        "joint_zero_rad": joint.joint_zero_rad,
        "training_default_rad": joint.training_default_rad,
        "captured_center_step": joint.center_step,
        "center_step": joint.center_step,
        "expected_policy_default_step": expected_default_step(joint),
        "expected_default_step": expected_default_step(joint),
        "endpoint_a": asdict(endpoint_a),
        "endpoint_b": asdict(endpoint_b),
        "physical_min_step": step_lo,
        "physical_max_step": step_hi,
        "physical_lower_rad": q_lo,
        "physical_upper_rad": q_hi,
        "previous_lower_rad": joint.old_min_rad,
        "previous_upper_rad": joint.old_max_rad,
    }


def save_capture_yaml(
    path: Path,
    servo_map_path: Path,
    joints: list[JointMapEntry],
    records: dict[str, dict[str, Any]],
    device: str,
    baud: int,
) -> None:
    payload = {
        "schema_version": 1,
        "robot": "LittleGreen",
        "tool_version": "1.1.0",
        "capture_type": "physical_joint_endpoints_with_model_space_limits",
        "captured_at_utc": datetime.now(timezone.utc).isoformat(),
        "source_servo_map": str(servo_map_path),
        "source_servo_map_sha256": sha256_file(servo_map_path),
        "uart_device": device,
        "baud": baud,
        "joint_order": [j.name for j in joints],
        "joints": [records[j.name] for j in joints if j.name in records],
    }
    path.write_text(yaml.safe_dump(payload, sort_keys=False))


def load_capture_yaml(path: Path) -> dict[str, Any]:
    root = yaml.safe_load(path.read_text())
    if not isinstance(root, dict) or not isinstance(root.get("joints"), list):
        raise ValueError(f"Invalid capture YAML: {path}")
    return root


# ------------------------------ capture audit ------------------------------


def audit_capture_geometry(capture: dict[str, Any], margin_steps: int) -> list[dict[str, Any]]:
    """Audit policy-default containment in model-space, independent of current centers."""
    margin_rad = margin_steps * RADIANS_PER_STEP
    rows: list[dict[str, Any]] = []
    for item in capture["joints"]:
        physical_lower = float(item["physical_lower_rad"])
        physical_upper = float(item["physical_upper_rad"])
        default_rad = float(item["training_default_rad"])
        safe_lower = physical_lower + margin_rad
        safe_upper = physical_upper - margin_rad
        default_bracketed = physical_lower <= default_rad <= physical_upper
        clearance_rad = (
            min(default_rad - physical_lower, physical_upper - default_rad)
            if default_bracketed
            else -1.0
        )
        margin_compatible = safe_lower <= default_rad <= safe_upper
        rows.append({
            "policy_index": int(item["policy_index"]),
            "name": str(item["name"]),
            "physical_lower_rad": physical_lower,
            "policy_default_rad": default_rad,
            "physical_upper_rad": physical_upper,
            "default_bracketed": default_bracketed,
            "nearest_endpoint_clearance_rad": clearance_rad,
            "requested_margin_steps": int(margin_steps),
            "requested_margin_rad": margin_rad,
            "margin_keeps_default_inside": margin_compatible,
        })
    rows.sort(key=lambda row: row["policy_index"])
    return rows


def write_capture_audit(output_dir: Path, rows: list[dict[str, Any]]) -> None:
    csv_path = output_dir / "capture_audit.csv"
    with csv_path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    lines = [
        "LittleGreen physical-limit capture audit",
        "=" * 72,
        "",
        "idx  joint                               physical lower  policy default  physical upper  bracket  margin_ok",
        "-" * 126,
    ]
    for row in rows:
        lines.append(
            f"{row['policy_index']:>3}  {row['name']:<36} "
            f"{row['physical_lower_rad']:>+13.5f}  {row['policy_default_rad']:>+14.5f}  "
            f"{row['physical_upper_rad']:>+13.5f}  "
            f"{str(row['default_bracketed']):>7}  {str(row['margin_keeps_default_inside']):>9}"
        )
    lines.extend([
        "",
        "Audit authority: physical/model-space radians. Raw endpoints are deployment values derived from the current center_step.",
    ])
    (output_dir / "capture_audit.txt").write_text("\n".join(lines) + "\n")


# ------------------------------ render contracts ------------------------------


def build_authoritative_contract(
    capture: dict[str, Any],
    joints: list[JointMapEntry],
    margin_steps: int,
) -> dict[str, Any]:
    """Build center-independent radian limits and center-dependent raw adapters."""
    if margin_steps < 0:
        raise ValueError("margin_steps must be >= 0")
    capture_by_name = {str(item["name"]): item for item in capture["joints"]}
    missing = [j.name for j in joints if j.name not in capture_by_name]
    if missing:
        raise ValueError("Capture is incomplete; missing: " + ", ".join(missing))

    margin_rad = margin_steps * RADIANS_PER_STEP
    output_joints: list[dict[str, Any]] = []
    for joint in joints:
        item = capture_by_name[joint.name]
        physical_lower = float(item["physical_lower_rad"])
        physical_upper = float(item["physical_upper_rad"])
        safe_lower = physical_lower + margin_rad
        safe_upper = physical_upper - margin_rad
        if safe_lower >= safe_upper:
            raise ValueError(
                f"Margin {margin_steps} steps collapses model-space range for {joint.name}: "
                f"[{physical_lower}, {physical_upper}]"
            )
        if not safe_lower <= joint.training_default_rad <= safe_upper:
            raise ValueError(
                f"Policy default for {joint.name} is outside the safety-margined physical range"
            )

        raw_a = int(round(rad_to_step(joint, safe_lower)))
        raw_b = int(round(rad_to_step(joint, safe_upper)))
        safe_min_step = min(raw_a, raw_b)
        safe_max_step = max(raw_a, raw_b)
        if not (0 <= safe_min_step < safe_max_step <= 4095):
            raise ValueError(
                f"Current center_step maps safe limits outside 0..4095 for {joint.name}: "
                f"[{safe_min_step}, {safe_max_step}]"
            )

        output_joints.append({
            "name": joint.name,
            "policy_index": joint.policy_index,
            "servo_id": joint.servo_id,
            "servo_sign": joint.servo_sign,
            "joint_zero_rad": joint.joint_zero_rad,
            "training_default_rad": joint.training_default_rad,
            "center_step": joint.center_step,
            "capture_center_step": int(item.get("captured_center_step", item.get("center_step", joint.center_step))),
            "capture_physical_min_step": int(item["physical_min_step"]),
            "capture_physical_max_step": int(item["physical_max_step"]),
            "physical_lower_rad": physical_lower,
            "physical_upper_rad": physical_upper,
            "safety_margin_steps_each_end": margin_steps,
            "safety_margin_rad_equivalent": margin_rad,
            "safe_lower_rad": safe_lower,
            "safe_upper_rad": safe_upper,
            "derived_safe_min_step": safe_min_step,
            "derived_safe_max_step": safe_max_step,
            "safe_min_step": safe_min_step,
            "safe_max_step": safe_max_step,
            "training_default_inside_safe_range": True,
            "previous_lower_rad": joint.old_min_rad,
            "previous_upper_rad": joint.old_max_rad,
        })

    return {
        "schema_version": 2,
        "robot": "LittleGreen",
        "contract_role": "authoritative_measured_model_space_hardware_limits",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "capture_source": capture.get("captured_at_utc", "unknown"),
        "authority_model": {
            "durable": "physical_lower_rad/physical_upper_rad and safe_lower_rad/safe_upper_rad",
            "center_calibration": "center_step at model zero",
            "derived": "safe_min_step/safe_max_step from current center_step",
        },
        "margin_policy": {
            "type": "fixed_inward_angular_margin_equivalent_to_capture_steps",
            "margin_steps": margin_steps,
            "margin_rad_equivalent": margin_rad,
            "margin_deg_equivalent": math.degrees(margin_rad),
        },
        "joint_order": [j.name for j in joints],
        "training_default_rad": [j.training_default_rad for j in joints],
        "safe_lower_limit_rad": [item["safe_lower_rad"] for item in output_joints],
        "safe_upper_limit_rad": [item["safe_upper_rad"] for item in output_joints],
        "joints": output_joints,
    }


def write_contract_csv(path: Path, contract: dict[str, Any]) -> None:
    fields = [
        "policy_index",
        "servo_id",
        "name",
        "servo_sign",
        "center_step",
        "training_default_rad",
        "capture_physical_min_step",
        "capture_physical_max_step",
        "physical_lower_rad",
        "physical_upper_rad",
        "safety_margin_steps_each_end",
        "derived_safe_min_step",
        "derived_safe_max_step",
        "safe_lower_rad",
        "safe_upper_rad",
        "previous_lower_rad",
        "previous_upper_rad",
        "training_default_inside_safe_range",
    ]
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for item in contract["joints"]:
            writer.writerow({field: item.get(field) for field in fields})


def py_list(name: str, values: list[Any], indent: int = 4) -> str:
    lines = [f"{name} = ["]
    for value in values:
        if isinstance(value, str):
            lines.append(" " * indent + repr(value) + ",")
        elif isinstance(value, float):
            lines.append(" " * indent + f"{value:.12g},")
        else:
            lines.append(" " * indent + repr(value) + ",")
    lines.append("]")
    return "\n".join(lines)


def write_training_hardware_contract_py(path: Path, contract: dict[str, Any]) -> None:
    names = contract["joint_order"]
    defaults = contract["training_default_rad"]
    lower = contract["safe_lower_limit_rad"]
    upper = contract["safe_upper_limit_rad"]
    text = f'''"""Generated measured hardware contract for LittleGreen.

Generated by lgh_hardware_limit_tool.py.
Do not hand-edit numeric limits; regenerate from authoritative_hardware_contract.yaml.
"""

from __future__ import annotations

{py_list("ACTIONABLE_JOINTS_V1_2_3", names)}

{py_list("TRAINING_DEFAULT_RAD", defaults)}

{py_list("HARDWARE_LOWER_LIMIT_RAD", lower)}

{py_list("HARDWARE_UPPER_LIMIT_RAD", upper)}


def map_bounded_action_scalar(action: float, default: float, lower: float, upper: float) -> float:
    """Map normalized action [-1, 1] asymmetrically around the training default."""
    a = max(-1.0, min(1.0, float(action)))
    if a >= 0.0:
        return default + a * (upper - default)
    return default + (-a) * (lower - default)


def validate_contract() -> None:
    n = len(ACTIONABLE_JOINTS_V1_2_3)
    if not (
        len(TRAINING_DEFAULT_RAD)
        == len(HARDWARE_LOWER_LIMIT_RAD)
        == len(HARDWARE_UPPER_LIMIT_RAD)
        == n
    ):
        raise ValueError("hardware-contract arrays must have identical lengths")
    for name, default, lower, upper in zip(
        ACTIONABLE_JOINTS_V1_2_3,
        TRAINING_DEFAULT_RAD,
        HARDWARE_LOWER_LIMIT_RAD,
        HARDWARE_UPPER_LIMIT_RAD,
    ):
        if not lower <= default <= upper:
            raise ValueError(
                f"Training default for {{name}} ({{default}}) is outside [{{lower}}, {{upper}}]"
            )


validate_contract()
'''
    path.write_text(text)


def write_ros_servo_map(
    path: Path,
    servo_map_root: dict[str, Any],
    contract: dict[str, Any],
) -> None:
    root = json.loads(json.dumps(servo_map_root))  # simple deep copy
    contract_by_name = {item["name"]: item for item in contract["joints"]}
    for joint in root["joints"]:
        item = contract_by_name[str(joint["name"])]
        joint["min_rad"] = float(item["safe_lower_rad"])
        joint["max_rad"] = float(item["safe_upper_rad"])
        joint["min_step"] = int(item["safe_min_step"])
        joint["max_step"] = int(item["safe_max_step"])
    root["hardware_limit_source"] = "authoritative_hardware_contract.yaml"
    root["hardware_limit_margin_steps_each_end"] = int(
        contract["margin_policy"]["margin_steps"]
    )
    path.write_text(yaml.safe_dump(root, sort_keys=False))


def classify_comparison(
    measured_lo: float,
    measured_hi: float,
    training_lo: float,
    training_hi: float,
    tolerance: float,
) -> str:
    lo_match = abs(measured_lo - training_lo) <= tolerance
    hi_match = abs(measured_hi - training_hi) <= tolerance
    if lo_match and hi_match:
        return "MATCH"
    training_inside = training_lo >= measured_lo - tolerance and training_hi <= measured_hi + tolerance
    measured_inside = measured_lo >= training_lo - tolerance and measured_hi <= training_hi + tolerance
    if training_inside:
        return "TRAINING_TRIMMED_INSIDE_HARDWARE"
    if measured_inside:
        return "TRAINING_EXCEEDS_MEASURED_SAFE_RANGE"
    return "SHIFTED_OR_MIXED"


def compare_to_training_contract(
    contract: dict[str, Any],
    training: dict[str, Any],
    tolerance: float,
) -> list[dict[str, Any]]:
    t_names = training["joint_order"]
    if len(t_names) != EXPECTED_JOINTS:
        raise ValueError("Training hardware contract does not contain 12 joints")
    t_by_name = {
        name: {
            "default": training["training_default_rad"][i],
            "lower": training["lower_limit_rad"][i],
            "upper": training["upper_limit_rad"][i],
        }
        for i, name in enumerate(t_names)
    }
    rows: list[dict[str, Any]] = []
    for item in contract["joints"]:
        name = item["name"]
        if name not in t_by_name:
            raise ValueError(f"Training contract missing joint {name}")
        t = t_by_name[name]
        rows.append(
            {
                "policy_index": item["policy_index"],
                "name": name,
                "measured_physical_lower_rad": item["physical_lower_rad"],
                "measured_physical_upper_rad": item["physical_upper_rad"],
                "measured_safe_lower_rad": item["safe_lower_rad"],
                "measured_safe_upper_rad": item["safe_upper_rad"],
                "training_lower_rad": t["lower"],
                "training_upper_rad": t["upper"],
                "safe_lower_delta_rad": item["safe_lower_rad"] - t["lower"],
                "safe_upper_delta_rad": item["safe_upper_rad"] - t["upper"],
                "measured_default_rad": item["training_default_rad"],
                "training_default_rad": t["default"],
                "default_delta_rad": item["training_default_rad"] - t["default"],
                "classification_vs_physical": classify_comparison(
                    item["physical_lower_rad"],
                    item["physical_upper_rad"],
                    t["lower"],
                    t["upper"],
                    tolerance,
                ),
                "classification_vs_safe": classify_comparison(
                    item["safe_lower_rad"],
                    item["safe_upper_rad"],
                    t["lower"],
                    t["upper"],
                    tolerance,
                ),
            }
        )
    return rows


def write_comparison_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        return
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def write_comparison_report(path: Path, rows: list[dict[str, Any]]) -> None:
    lines = [
        "LittleGreen measured-vs-deployment hardware limit comparison",
        "=" * 76,
        "",
        "Physical endpoint comparison",
        "----------------------------",
        "idx  joint                               measured physical rad      training rad               class",
        "-" * 132,
    ]
    for row in rows:
        lines.append(
            f"{row['policy_index']:>3}  {row['name']:<36} "
            f"[{row['measured_physical_lower_rad']:+.4f}, {row['measured_physical_upper_rad']:+.4f}]  "
            f"[{row['training_lower_rad']:+.4f}, {row['training_upper_rad']:+.4f}]  "
            f"{row['classification_vs_physical']}"
        )
    lines.extend([
        "",
        "Safety-margined authoritative comparison",
        "-----------------------------------------",
        "idx  joint                               measured safe rad          training rad               class",
        "-" * 132,
    ])
    for row in rows:
        lines.append(
            f"{row['policy_index']:>3}  {row['name']:<36} "
            f"[{row['measured_safe_lower_rad']:+.4f}, {row['measured_safe_upper_rad']:+.4f}]  "
            f"[{row['training_lower_rad']:+.4f}, {row['training_upper_rad']:+.4f}]  "
            f"{row['classification_vs_safe']}"
        )
    lines.extend(["", "Delta convention in CSV: measured_safe - training_limit.", ""])
    path.write_text("\n".join(lines))


def render_outputs(
    capture_path: Path,
    servo_map_path: Path,
    output_dir: Path,
    margin_steps: int,
    training_contract_path: Path | None,
    compare_tolerance_rad: float,
) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    servo_root, joints = load_servo_map(servo_map_path)
    capture = load_capture_yaml(capture_path)
    audit_rows = audit_capture_geometry(capture, margin_steps)
    write_capture_audit(output_dir, audit_rows)
    unbracketed = [row["name"] for row in audit_rows if not row["default_bracketed"]]
    if unbracketed:
        raise ValueError(
            "Captured physical endpoints do not bracket the policy-default angle for: "
            + ", ".join(unbracketed)
            + ". Re-capture those joints before generating an authoritative contract."
        )
    margin_conflicts = [row["name"] for row in audit_rows if not row["margin_keeps_default_inside"]]
    if margin_conflicts:
        raise ValueError(
            f"Requested {margin_steps}-step-equivalent inward margin excludes the policy default for: "
            + ", ".join(margin_conflicts)
            + ". Re-capture the endpoint if incomplete, reduce the margin, or adopt an explicit per-joint/asymmetric margin policy."
        )
    contract = build_authoritative_contract(capture, joints, margin_steps)
    contract["source_capture_file"] = str(capture_path)
    contract["source_capture_sha256"] = sha256_file(capture_path)
    contract["source_servo_map"] = str(servo_map_path)
    contract["source_servo_map_sha256"] = sha256_file(servo_map_path)

    authoritative = output_dir / "authoritative_hardware_contract.yaml"
    authoritative.write_text(yaml.safe_dump(contract, sort_keys=False))
    write_contract_csv(output_dir / "authoritative_hardware_contract.csv", contract)
    write_training_hardware_contract_py(
        output_dir / "track1_hardware_contract.generated.py", contract
    )
    write_ros_servo_map(
        output_dir / "servo_map.measured_limits.generated.yaml", servo_root, contract
    )

    comparison_rows: list[dict[str, Any]] = []
    if training_contract_path is not None:
        training = load_deployment_contract(training_contract_path)
        comparison_rows = compare_to_training_contract(
            contract, training, compare_tolerance_rad
        )
        write_comparison_csv(output_dir / "comparison_to_deployment_contract.csv", comparison_rows)
        write_comparison_report(output_dir / "comparison_report.txt", comparison_rows)

    # Cross-check generated adapters against the neutral contract.
    generated_training = load_deployment_contract(
        output_dir / "track1_hardware_contract.generated.py"
    )
    if generated_training["joint_order"] != contract["joint_order"]:
        raise RuntimeError("Generated training contract joint order mismatch")
    for a, b in zip(
        generated_training["lower_limit_rad"], contract["safe_lower_limit_rad"]
    ):
        if not math.isclose(a, b, abs_tol=1e-10, rel_tol=0.0):
            raise RuntimeError("Generated training lower limits do not match authoritative YAML")
    for a, b in zip(
        generated_training["upper_limit_rad"], contract["safe_upper_limit_rad"]
    ):
        if not math.isclose(a, b, abs_tol=1e-10, rel_tol=0.0):
            raise RuntimeError("Generated training upper limits do not match authoritative YAML")

    manifest = {
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "margin_steps": margin_steps,
        "files": {},
    }
    for filename in [
        "authoritative_hardware_contract.yaml",
        "authoritative_hardware_contract.csv",
        "track1_hardware_contract.generated.py",
        "servo_map.measured_limits.generated.yaml",
        "comparison_to_deployment_contract.csv",
        "comparison_report.txt",
    ]:
        file_path = output_dir / filename
        if file_path.exists():
            manifest["files"][filename] = sha256_file(file_path)
    (output_dir / "generation_manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n"
    )

    print()
    print("Generated synchronized hardware-contract artifacts")
    print("================================================")
    print(f"Neutral source:   {authoritative}")
    print(f"Track 1 adapter: {output_dir / 'track1_hardware_contract.generated.py'}")
    print(f"ROS 2 adapter:    {output_dir / 'servo_map.measured_limits.generated.yaml'}")
    if comparison_rows:
        physical_counts: dict[str, int] = {}
        safe_counts: dict[str, int] = {}
        for row in comparison_rows:
            pc = row["classification_vs_physical"]
            sc = row["classification_vs_safe"]
            physical_counts[pc] = physical_counts.get(pc, 0) + 1
            safe_counts[sc] = safe_counts.get(sc, 0) + 1
        print(f"Comparison:       {output_dir / 'comparison_report.txt'}")
        print("Physical compare: " + ", ".join(f"{k}={v}" for k, v in sorted(physical_counts.items())))
        print("Safe compare:     " + ", ".join(f"{k}={v}" for k, v in sorted(safe_counts.items())))


# ------------------------------ commands ------------------------------


def cmd_capture(args: argparse.Namespace) -> int:
    servo_map_path = resolve_cli_path(args.servo_map)
    output_dir = resolve_cli_path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    capture_path = output_dir / "physical_limit_capture.yaml"

    servo_root, joints = load_servo_map(servo_map_path)
    del servo_root

    records: dict[str, dict[str, Any]] = {}
    if args.resume and capture_path.exists():
        existing = load_capture_yaml(capture_path)
        records = {str(item["name"]): item for item in existing["joints"]}
        print(f"Resuming existing capture with {len(records)}/12 joints: {capture_path}")
    elif capture_path.exists() and not args.overwrite:
        raise SystemExit(
            f"Capture already exists: {capture_path}\nUse --resume or --overwrite."
        )

    print("LittleGreen standalone physical-limit capture")
    print("========================================================")
    print(f"Servo map: {servo_map_path}")
    print(f"UART:      {args.device}@{args.baud}")
    print(f"Output:    {output_dir}")
    print()
    print("PRECONDITION: stop the ROS 2 ST3215 driver before continuing.")
    print("The robot must be securely supported. Torque will be disabled for all 12 servos.")
    print("Move joints slowly by hand. Do not force a joint into its hard stop.")
    confirm = input("Type DISABLE TORQUE exactly to continue: ").strip()
    if confirm != "DISABLE TORQUE":
        print("Aborted; torque command was not sent.")
        return 2

    with St3215Serial(args.device, args.baud, args.uart_timeout_s) as bus:
        print("\nChecking servo communication...")
        initial_steps: dict[str, int] = {}
        for joint in joints:
            step = bus.read_position(joint.servo_id)
            initial_steps[joint.name] = step
            print(
                f"  ID {joint.servo_id:>2} {joint.name:<36} "
                f"step={step:>5} q={steps_to_rad(joint, step):+.4f} rad"
            )

        print("\nDisabling torque on all servos...")
        bus.broadcast_torque(False)
        torque_states = {}
        for joint in joints:
            state = bus.read_torque_enabled(joint.servo_id)
            torque_states[joint.name] = state
        bad = [name for name, state in torque_states.items() if state != 0]
        if bad:
            raise RuntimeError("Torque-disable verification failed for: " + ", ".join(bad))
        print("Torque-disable verification: PASS (12/12)")

        quit_requested = False
        recapture_names = set(args.recapture_joint or [])
        unknown_recaptures = recapture_names - {j.name for j in joints}
        if unknown_recaptures:
            raise ValueError("Unknown --recapture-joint name(s): " + ", ".join(sorted(unknown_recaptures)))
        for joint in joints:
            if joint.name in records and joint.name not in recapture_names:
                print(f"\nSkipping already captured joint: {joint.name}")
                continue
            if joint.name in recapture_names and joint.name in records:
                print(f"\nRe-capturing joint and replacing prior record: {joint.name}")

            response = prompt_endpoint(joint, "A")
            if response == "q":
                quit_requested = True
                break
            if response == "s":
                print("Skipped.")
                continue
            endpoint_a = capture_endpoint(
                bus, joint, "A", args.samples, args.sample_rate_hz
            )
            print(
                f"  A: median={endpoint_a.median_step:.1f} step  "
                f"q={endpoint_a.median_rad:+.4f} rad  span={endpoint_a.span_steps} steps"
            )

            response = prompt_endpoint(joint, "B")
            if response == "q":
                quit_requested = True
                break
            if response == "s":
                print("Joint incomplete; endpoint A was not saved.")
                continue
            endpoint_b = capture_endpoint(
                bus, joint, "B", args.samples, args.sample_rate_hz
            )
            print(
                f"  B: median={endpoint_b.median_step:.1f} step  "
                f"q={endpoint_b.median_rad:+.4f} rad  span={endpoint_b.span_steps} steps"
            )

            record = capture_record_from_endpoints(joint, endpoint_a, endpoint_b)
            records[joint.name] = record
            print(
                f"  physical raw range: [{record['physical_min_step']}, {record['physical_max_step']}]"
            )
            print(
                f"  physical q range:   [{record['physical_lower_rad']:+.4f}, "
                f"{record['physical_upper_rad']:+.4f}] rad"
            )
            save_capture_yaml(
                capture_path, servo_map_path, joints, records, args.device, args.baud
            )
            print(f"  saved: {capture_path}")

        print("\nCapture session ended. Torque remains DISABLED.")
        print("Support the robot before exiting or re-enabling torque through your normal controlled workflow.")

    save_capture_yaml(
        capture_path, servo_map_path, joints, records, args.device, args.baud
    )

    if len(records) == EXPECTED_JOINTS:
        print("\nAll 12 joints captured. Rendering synchronized contracts...")
        training_path = resolve_optional_training_contract(args.training_contract)
        render_outputs(
            capture_path,
            servo_map_path,
            output_dir,
            args.margin_steps,
            training_path,
            args.compare_tolerance_rad,
        )
    else:
        print(f"\nPartial capture saved: {len(records)}/12 joints.")
        print("Resume later with the same command plus --resume.")
        if quit_requested:
            return 0
    return 0


def cmd_render(args: argparse.Namespace) -> int:
    capture_path = resolve_cli_path(args.capture)
    servo_map_path = resolve_cli_path(args.servo_map)
    output_dir = resolve_cli_path(args.output_dir)
    training_path = resolve_optional_training_contract(args.training_contract)
    render_outputs(
        capture_path,
        servo_map_path,
        output_dir,
        args.margin_steps,
        training_path,
        args.compare_tolerance_rad,
    )
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Standalone ST3215 physical joint-limit capture and contract generator"
    )
    sub = parser.add_subparsers(dest="command", required=True)

    capture = sub.add_parser("capture", help="Capture all physical joint endpoints over direct UART")
    capture.add_argument("--servo-map", default=str(DEFAULT_SERVO_MAP), help="Current calibrated LittleGreen servo_map.yaml")
    capture.add_argument("--device", default="/dev/ttyS3")
    capture.add_argument("--baud", type=int, default=1_000_000)
    capture.add_argument("--uart-timeout-s", type=float, default=0.03)
    capture.add_argument("--samples", type=int, default=80, help="Samples per physical endpoint")
    capture.add_argument("--sample-rate-hz", type=float, default=40.0)
    capture.add_argument(
        "--margin-steps",
        type=int,
        default=10,
        help="Inward margin equivalent to 10 raw steps at capture scale (default: 10)",
    )
    capture.add_argument("--training-contract", "--deployment-contract", dest="training_contract", default=str(DEFAULT_TRACK2_CONTRACT), help="Path to current LittleGreen YAML/Python deployment hardware contract")
    capture.add_argument("--compare-tolerance-rad", type=float, default=0.02)
    capture.add_argument("--output-dir", required=True)
    capture.add_argument("--resume", action="store_true")
    capture.add_argument(
        "--recapture-joint",
        action="append",
        default=[],
        help="With --resume, force re-capture of this joint name; may be repeated",
    )
    capture.add_argument("--overwrite", action="store_true")
    capture.set_defaults(func=cmd_capture)

    render = sub.add_parser(
        "render",
        help="Regenerate synchronized contracts from an existing physical capture",
    )
    render.add_argument("--capture", required=True, help="physical_limit_capture.yaml")
    render.add_argument("--servo-map", default=str(DEFAULT_SERVO_MAP), help="Current calibrated LittleGreen servo_map.yaml")
    render.add_argument(
        "--margin-steps",
        type=int,
        default=10,
        help="Inward margin equivalent to 10 raw steps at capture scale (default: 10)",
    )
    render.add_argument("--training-contract", "--deployment-contract", dest="training_contract", default=str(DEFAULT_TRACK2_CONTRACT), help="Path to current LittleGreen YAML/Python deployment hardware contract")
    render.add_argument("--compare-tolerance-rad", type=float, default=0.02)
    render.add_argument("--output-dir", required=True)
    render.set_defaults(func=cmd_render)

    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    try:
        return int(args.func(args))
    except KeyboardInterrupt:
        print("\nInterrupted. Torque state is not changed automatically on interrupt.", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
