#!/usr/bin/env bash
set -uo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "$SCRIPT_DIR/.." && pwd)"
SOFTWARE_ONLY=0
[[ "${1:-}" == "--software-only" ]] && SOFTWARE_ONLY=1

fail=0
warn=0
pass() { printf 'PASS  %s\n' "$*"; }
warning() { printf 'WARN  %s\n' "$*"; warn=$((warn+1)); }
error() { printf 'FAIL  %s\n' "$*"; fail=$((fail+1)); }

[[ -f /opt/ros/humble/setup.bash ]] && pass "ROS 2 Humble base installed" || error "missing /opt/ros/humble/setup.bash"
if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi

ORT_DIR="${ONNXRUNTIME_DIR:-$HOME/libs/onnxruntime-linux-aarch64-1.22.0}"
[[ -f "$ORT_DIR/include/onnxruntime_cxx_api.h" ]] && pass "ONNX Runtime headers found" || error "missing ONNX Runtime headers at $ORT_DIR"
[[ -f "$ORT_DIR/lib/libonnxruntime.so" ]] && pass "ONNX Runtime shared library found" || error "missing ONNX Runtime library at $ORT_DIR"
export LD_LIBRARY_PATH="$ORT_DIR/lib:${LD_LIBRARY_PATH:-}"

if [[ -f "$WORKSPACE/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$WORKSPACE/install/setup.bash"
  pass "workspace overlay exists"
else
  error "missing workspace overlay; run scripts/build_workspace.sh"
fi

expected=(lgh_st3215_driver lgh_st3215_tools lgh_st3215_maintenance lgh_imu_tools littlegreen_biped_pkg littlegreen_description pd_controller_pkg joystick_bridge teleop_twist_joy)
if command -v ros2 >/dev/null 2>&1; then
  installed="$(ros2 pkg list 2>/dev/null || true)"
  for p in "${expected[@]}"; do
    grep -qx "$p" <<<"$installed" && pass "ROS package $p" || error "ROS package missing: $p"
  done
  for old in bhl_st3215_driver bhl_st3215_tools bhl_st3215_maintenance bhl_imu_tools berkeley_biped_pkg lilgreen_description; do
    if grep -qx "$old" <<<"$installed"; then
      warning "old package is still visible in the environment: $old (check sourced overlays)"
    fi
  done
else
  error "ros2 command unavailable"
fi

if "$WORKSPACE/scripts/validate_source_tree.py" >/tmp/lgh_validate_source.log 2>&1; then
  pass "source-tree rename and syntax audit"
else
  error "source-tree validation failed; see /tmp/lgh_validate_source.log"
fi

if [[ $SOFTWARE_ONLY -eq 0 ]]; then
  [[ -e /dev/ttyS3 ]] && pass "/dev/ttyS3 exists" || warning "/dev/ttyS3 is not present"
  if id -nG "$USER" | tr ' ' '\n' | grep -qx dialout; then
    pass "user belongs to dialout"
  else
    warning "current login does not include dialout; log out/in after installation"
  fi
fi

echo
if [[ $fail -gt 0 ]]; then
  echo "INSTALL VERIFICATION: FAIL ($fail failures, $warn warnings)"
  exit 3
elif [[ $warn -gt 0 ]]; then
  echo "INSTALL VERIFICATION: PASS WITH WARNINGS ($warn)"
  exit 2
else
  echo "INSTALL VERIFICATION: PASS"
  exit 0
fi
