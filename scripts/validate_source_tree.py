#!/usr/bin/env python3
"""ROS-independent validation for the current LittleGreen source workspace."""
from __future__ import annotations

import ast
import hashlib
import math
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

try:
    import yaml
except ImportError:
    yaml = None

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
TOOLS = ROOT / "tools"
EXPECTED = {
    "lgh_st3215_driver",
    "lgh_st3215_tools",
    "lgh_st3215_maintenance",
    "lgh_imu_tools",
    "littlegreen_biped_pkg",
    "littlegreen_description",
    "pd_controller_pkg",
    "joystick_bridge",
    "teleop_twist_joy",
}
FORBIDDEN_PATH_TOKENS = ("bhl_", "berkeley_biped", "lilgreen_description")
FORBIDDEN_TEXT_TOKENS = ("bhl_", "berkeley_biped", "berkeley_ros2_ws", "lilgreen_description")
TEXT_SUFFIXES = {
    ".py", ".cpp", ".hpp", ".h", ".xml", ".yaml", ".yml", ".md",
    ".txt", ".cfg", ".cmake", ".xacro", ".gazebo", ".trans",
}
ALLOWED_PROVENANCE_TEXT = {
    Path("src/lgh_st3215_driver/config/servo_map.yaml"): [
        "bhl_st3215_microros_pio_v6_5_8/include/servo_map.h"
    ],
}

errors: list[str] = []
warnings: list[str] = []


def fail(message: str) -> None:
    errors.append(message)


def require_close(actual: float, expected: float, label: str, tolerance: float = 1.0e-5) -> None:
    if not (math.isfinite(actual) and math.isfinite(expected)):
        fail(f"non-finite value for {label}: actual={actual}, expected={expected}")
    elif abs(actual - expected) > tolerance:
        fail(f"mismatch for {label}: actual={actual}, expected={expected}, tolerance={tolerance}")


for token in FORBIDDEN_PATH_TOKENS:
    for path in SRC.rglob(f"*{token}*"):
        fail(f"forbidden old-name path: {path.relative_to(ROOT)}")

package_names: set[str] = set()
for package_xml in SRC.rglob("package.xml"):
    try:
        tree = ET.parse(package_xml)
        name = tree.findtext("name") or ""
    except Exception as exc:
        fail(f"invalid package.xml {package_xml.relative_to(ROOT)}: {exc}")
        continue
    package_names.add(name)
    if name in {
        "bhl_st3215_driver", "bhl_st3215_tools", "bhl_st3215_maintenance",
        "bhl_imu_tools", "berkeley_biped_pkg", "lilgreen_description",
    }:
        fail(f"old package name still active: {name}")

missing = EXPECTED - package_names
if missing:
    fail(f"missing expected packages: {sorted(missing)}")

for scan_root in (SRC, TOOLS):
  if not scan_root.exists():
    continue
  for path in scan_root.rglob("*"):
    if not path.is_file() or path.suffix.lower() not in TEXT_SUFFIXES:
        continue
    try:
        text = path.read_text(encoding="utf-8")
    except UnicodeDecodeError:
        continue
    relative = path.relative_to(ROOT)
    scan_text = text
    for allowed in ALLOWED_PROVENANCE_TEXT.get(relative, []):
        if allowed in scan_text:
            scan_text = scan_text.replace(allowed, "<allowed-historical-provenance>")
            warnings.append(f"historical provenance reference retained in {relative}: {allowed}")
    for token in FORBIDDEN_TEXT_TOKENS:
        if token in scan_text:
            fail(f"forbidden old identifier {token!r} in {relative}")
    if path.suffix == ".py":
        try:
            ast.parse(text, filename=str(path))
        except SyntaxError as exc:
            fail(f"Python syntax error in {relative}: {exc}")
    if path.suffix in {".yaml", ".yml"} and yaml is not None:
        try:
            yaml.safe_load(text)
        except Exception as exc:
            fail(f"YAML parse error in {relative}: {exc}")

# Empty source directories are not reliably preserved by ZIP extraction.
for cmake_file in SRC.rglob("CMakeLists.txt"):
    cmake_text = cmake_file.read_text(encoding="utf-8")
    for match in re.finditer(r"install\s*\(\s*DIRECTORY\s+([^\s\)]+)", cmake_text, re.MULTILINE):
        token = match.group(1).strip('"')
        if token.startswith("$"):
            continue
        directory = cmake_file.parent / token.rstrip("/")
        if not directory.is_dir():
            fail(
                "install(DIRECTORY) source does not exist: "
                f"{directory.relative_to(ROOT)} referenced by {cmake_file.relative_to(ROOT)}"
            )
        elif not any(directory.iterdir()):
            fail(
                "install(DIRECTORY) source is empty: "
                f"{directory.relative_to(ROOT)} referenced by {cmake_file.relative_to(ROOT)}"
            )

required_files = [
    SRC / "lgh_st3215_driver/launch/lgh_st3215_driver.launch.py",
    SRC / "lgh_st3215_driver/msg/ServoTelemetry.msg",
    SRC / "lgh_st3215_tools/lgh_st3215_tools/diagnostic_compat.py",
    SRC / "lgh_st3215_tools/lgh_st3215_tools/verify_reference_pose.py",
    SRC / "lgh_st3215_tools/test/test_calibration_semantics.py",
    SRC / "lgh_st3215_tools/config/track1_action_contract_v4.yaml",
    SRC / "littlegreen_biped_pkg/src/littlegreen_biped_node.cpp",
    SRC / "littlegreen_biped_pkg/scripts/policy_bundle_audit.py",
    SRC / "littlegreen_biped_pkg/scripts/policy_runtime_metrics.py",
    SRC / "littlegreen_biped_pkg/launch/littlegreen_biped_launch.py",
    SRC / "littlegreen_biped_pkg/launch/policy_shadow.launch.py",
    SRC / "littlegreen_biped_pkg/launch/policy_live.launch.py",
    SRC / "littlegreen_biped_pkg/src/configs/policy_latest.yaml",
    SRC / "littlegreen_biped_pkg/src/configs/policy.onnx",
    SRC / "littlegreen_biped_pkg/src/configs/joint_map.yaml",
    SRC / "littlegreen_description/urdf/littlegreen.xacro",
    ROOT / "scripts/install_ubuntu_x86_64.sh",
    ROOT / "scripts/install_onnxruntime_x86_64.sh",
    ROOT / "scripts/apply_v2_7_2_hotfix.sh",
    ROOT / "docs/LIVE_POLICY_DEPLOYMENT.md",
    ROOT / "docs/TRACK1_TRACK2_POLICY_METRICS.md",
    ROOT / "docs/INSTALL_UBUNTU_X86_64.md",
    ROOT / "docs/CALIBRATION_WORKFLOW.md",
    ROOT / "docs/SERVO_REPLACEMENT_CHECKLIST.md",
    ROOT / "tools/lgh_hardware_limit_tool/lgh_hardware_limit_tool.py",
    ROOT / "tools/lgh_hardware_limit_tool/README.md",
]
for path in required_files:
    if not path.is_file():
        fail(f"missing required file: {path.relative_to(ROOT)}")

policy_node = SRC / "littlegreen_biped_pkg/src/littlegreen_biped_node.cpp"
if policy_node.is_file():
    policy_text = policy_node.read_text(encoding="utf-8")
    required_contract_tokens = [
        "action_contract_version",
        "action_residual_scale_rad",
        "action_default_rad",
        "action_target_lower_rad",
        "action_target_upper_rad",
        "action_nominal_residual_lower_rad",
        "action_nominal_residual_upper_rad",
        "deployment_contract_profile",
        "bounded_default_centered_vector_residual",
        "previous_action_observation",
        "Action contract v%d validated against joint_map.yaml",
    ]
    for token in required_contract_tokens:
        if token not in policy_text:
            fail(f"policy node is missing action-contract-v3/v4 token: {token}")

# Validate the packaged Track 1 deployment bundle and hardware map.
if yaml is not None:
    policy_path = SRC / "littlegreen_biped_pkg/src/configs/policy_latest.yaml"
    onnx_path = SRC / "littlegreen_biped_pkg/src/configs/policy.onnx"
    joint_map_path = SRC / "littlegreen_biped_pkg/src/configs/joint_map.yaml"
    servo_map_path = SRC / "lgh_st3215_driver/config/servo_map.yaml"
    contract_path = SRC / "lgh_st3215_tools/config/track1_action_contract_v4.yaml"
    try:
        policy = yaml.safe_load(policy_path.read_text(encoding="utf-8"))
        joint_map = yaml.safe_load(joint_map_path.read_text(encoding="utf-8"))
        servo_map = yaml.safe_load(servo_map_path.read_text(encoding="utf-8"))
        contract = yaml.safe_load(contract_path.read_text(encoding="utf-8"))

        if int(policy.get("action_contract_version", 0)) != 4:
            fail("packaged policy_latest.yaml must use action_contract_version: 4")
        if policy.get("action_transform") != "bounded_default_centered_vector_residual":
            fail("packaged policy has unexpected action_transform")
        if not policy.get("deployment_requires_action_contract_v4_transform", False):
            fail("packaged policy must require the v4 deployment transform")
        if policy.get("previous_action_observation") != "bounded_normalized_action":
            fail("packaged policy has unexpected previous_action_observation")
        if int(policy.get("num_observations", 0)) != 45 or int(policy.get("num_actions", 0)) != 12:
            fail("packaged policy must expose obs[45] -> actions[12]")

        scale = [float(v) for v in policy["action_residual_scale_rad"]]
        defaults = [float(v) for v in policy["action_default_rad"]]
        lower = [float(v) for v in policy["action_target_lower_rad"]]
        upper = [float(v) for v in policy["action_target_upper_rad"]]
        nominal_lower = [float(v) for v in policy["action_nominal_residual_lower_rad"]]
        nominal_upper = [float(v) for v in policy["action_nominal_residual_upper_rad"]]
        if not all(len(v) == 12 for v in [scale, defaults, lower, upper, nominal_lower, nominal_upper]):
            fail("packaged policy action vectors must all contain 12 values")
        if max(scale) - min(scale) <= 1.0e-5:
            fail("action contract v4 requires a non-uniform residual scale vector")
        for index in range(min(12, len(scale))):
            require_close(nominal_lower[index], max(lower[index], defaults[index] - scale[index]), f"nominal lower[{index}]")
            require_close(nominal_upper[index], min(upper[index], defaults[index] + scale[index]), f"nominal upper[{index}]")

        joints = sorted(joint_map["joints"], key=lambda item: int(item["policy_action_index"]))
        if len(joints) != 12:
            fail("joint_map.yaml must contain 12 policy joints")
        else:
            for index, joint in enumerate(joints):
                require_close(defaults[index], float(joint["default_joint_rad"]), f"joint-map default[{index}]")
                require_close(lower[index], float(joint["limit_lower_rad"]), f"joint-map lower[{index}]")
                require_close(upper[index], float(joint["limit_upper_rad"]), f"joint-map upper[{index}]")

        servo_joints = sorted(servo_map["joints"], key=lambda item: int(item["policy_index"]))
        if len(servo_joints) != 12:
            fail("servo_map.yaml must contain 12 joints")
        else:
            steps_per_radian = 4096.0 / (2.0 * math.pi)
            for index, joint in enumerate(servo_joints):
                require_close(defaults[index], float(joint["training_default_rad"]), f"servo-map training default[{index}]")
                center = int(joint["center_step"])
                sign = int(joint["servo_sign"])
                zero = float(joint.get("joint_zero_rad", 0.0))
                raw_a = int(round(center + sign * (float(joint["min_rad"]) - zero) * steps_per_radian))
                raw_b = int(round(center + sign * (float(joint["max_rad"]) - zero) * steps_per_radian))
                expected_min_step, expected_max_step = min(raw_a, raw_b), max(raw_a, raw_b)
                if int(joint["min_step"]) != expected_min_step or int(joint["max_step"]) != expected_max_step:
                    fail(
                        f"servo-map raw limits are not derived from center/radian limits for {joint['name']}: "
                        f"stored=[{joint['min_step']}, {joint['max_step']}], "
                        f"derived=[{expected_min_step}, {expected_max_step}]"
                    )

        contract_policy_fields = {
            "action_contract_version": "action_contract_version",
            "action_transform": "action_transform",
            "action_contract_name": "action_contract_name",
            "deployment_contract_profile": "deployment_contract_profile",
            "residual_scale_rad": "action_residual_scale_rad",
            "training_default_rad": "action_default_rad",
            "lower_limit_rad": "action_target_lower_rad",
            "upper_limit_rad": "action_target_upper_rad",
            "nominal_residual_lower_rad": "action_nominal_residual_lower_rad",
            "nominal_residual_upper_rad": "action_nominal_residual_upper_rad",
        }
        for contract_key, policy_key in contract_policy_fields.items():
            if contract.get(contract_key) != policy.get(policy_key):
                fail(
                    "Track 2 contract snapshot differs from policy_latest.yaml for "
                    f"{contract_key}/{policy_key}"
                )

        expected_sha = str(policy.get("policy_sha256") or policy.get("metadata", {}).get("policy_sha256") or "")
        actual_sha = hashlib.sha256(onnx_path.read_bytes()).hexdigest()
        if expected_sha != actual_sha:
            fail(f"policy ONNX checksum mismatch: YAML={expected_sha}, actual={actual_sha}")
        checkpoint_sha = hashlib.sha256(
            (SRC / "littlegreen_biped_pkg/src/checkpoints/policy.onnx").read_bytes()
        ).hexdigest()
        if checkpoint_sha != actual_sha:
            fail("configs/policy.onnx and checkpoints/policy.onnx differ")
    except Exception as exc:
        fail(f"policy bundle validation raised: {exc}")
else:
    warnings.append("PyYAML unavailable; YAML parsing and policy-bundle checks were skipped")

if errors:
    print("SOURCE VALIDATION: FAIL")
    for item in errors:
        print(f"ERROR: {item}")
    for item in warnings:
        print(f"WARN: {item}")
    sys.exit(2)

print("SOURCE VALIDATION: PASS")
print(f"packages discovered: {len(package_names)}")
for item in warnings:
    print(f"WARN: {item}")
sys.exit(0)
