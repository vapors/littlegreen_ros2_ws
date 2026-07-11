#!/usr/bin/env python3
"""ROS-independent validation for the LittleGreen v2.6.0 source workspace."""
from __future__ import annotations

import ast
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

try:
    import yaml
except ImportError:
    yaml = None

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "src"
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
TEXT_SUFFIXES = {".py", ".cpp", ".hpp", ".h", ".xml", ".yaml", ".yml", ".md", ".txt", ".cfg", ".cmake", ".xacro", ".gazebo", ".trans"}
ALLOWED_PROVENANCE_TEXT = {
    Path("src/lgh_st3215_driver/config/servo_map.yaml"): [
        "bhl_st3215_microros_pio_v6_5_8/include/servo_map.h"
    ],
}

errors: list[str] = []
warnings: list[str] = []

for token in FORBIDDEN_PATH_TOKENS:
    for p in SRC.rglob(f"*{token}*"):
        errors.append(f"forbidden old-name path: {p.relative_to(ROOT)}")

package_names: set[str] = set()
for package_xml in SRC.rglob("package.xml"):
    try:
        tree = ET.parse(package_xml)
        name = tree.findtext("name") or ""
    except Exception as exc:
        errors.append(f"invalid package.xml {package_xml.relative_to(ROOT)}: {exc}")
        continue
    package_names.add(name)
    if name in {"bhl_st3215_driver", "bhl_st3215_tools", "bhl_st3215_maintenance", "bhl_imu_tools", "berkeley_biped_pkg", "lilgreen_description"}:
        errors.append(f"old package name still active: {name}")

missing = EXPECTED - package_names
if missing:
    errors.append(f"missing expected packages: {sorted(missing)}")

for p in SRC.rglob("*"):
    if not p.is_file() or p.suffix.lower() not in TEXT_SUFFIXES:
        continue
    try:
        text = p.read_text(encoding="utf-8")
    except UnicodeDecodeError:
        continue
    relative = p.relative_to(ROOT)
    scan_text = text
    for allowed in ALLOWED_PROVENANCE_TEXT.get(relative, []):
        if allowed in scan_text:
            scan_text = scan_text.replace(allowed, "<allowed-historical-provenance>")
            warnings.append(f"historical provenance reference retained in {relative}: {allowed}")
    for token in FORBIDDEN_TEXT_TOKENS:
        if token in scan_text:
            errors.append(f"forbidden old identifier {token!r} in {relative}")
    if p.suffix == ".py":
        try:
            ast.parse(text, filename=str(p))
        except SyntaxError as exc:
            errors.append(f"Python syntax error in {p.relative_to(ROOT)}: {exc}")
    if p.suffix in {".yaml", ".yml"} and yaml is not None:
        try:
            yaml.safe_load(text)
        except Exception as exc:
            errors.append(f"YAML parse error in {p.relative_to(ROOT)}: {exc}")

required_files = [
    SRC / "lgh_st3215_driver/launch/lgh_st3215_driver.launch.py",
    SRC / "lgh_st3215_driver/msg/ServoTelemetry.msg",
    SRC / "littlegreen_biped_pkg/src/littlegreen_biped_node.cpp",
    SRC / "littlegreen_biped_pkg/launch/littlegreen_biped_launch.py",
    SRC / "littlegreen_description/urdf/littlegreen.xacro",
]
for p in required_files:
    if not p.is_file():
        errors.append(f"missing required file: {p.relative_to(ROOT)}")

# The current Track 1 policy snapshot retains its historical task identifier by design.
legacy_task = "Velocity-Lilgreen-Humanoid-v0"
legacy_locations = []
for p in (SRC / "littlegreen_biped_pkg/src/configs").glob("*.yaml"):
    if legacy_task in p.read_text(encoding="utf-8"):
        legacy_locations.append(str(p.relative_to(ROOT)))
if legacy_locations:
    warnings.append(
        "legacy Track 1 task identifier retained until the new deployment bundle arrives: "
        + ", ".join(legacy_locations)
    )

if yaml is None:
    warnings.append("PyYAML unavailable; YAML parsing checks were skipped")

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
