#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "$SCRIPT_DIR/.." && pwd)"
CLEAN=0
RUN_ROSDEP=1
BUILD_TYPE="RelWithDebInfo"

usage() {
  cat <<EOF
Usage: $0 [--clean] [--skip-rosdep] [--release|--debug]

Builds the complete LittleGreen ROS 2 workspace.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --clean) CLEAN=1 ;;
    --skip-rosdep) RUN_ROSDEP=0 ;;
    --release) BUILD_TYPE="Release" ;;
    --debug) BUILD_TYPE="Debug" ;;
    -h|--help) usage; exit 0 ;;
    *) echo "ERROR: Unknown option: $1" >&2; usage; exit 5 ;;
  esac
  shift
done

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ERROR: ROS 2 Humble is not installed at /opt/ros/humble." >&2
  exit 4
fi
source /opt/ros/humble/setup.bash

ORT_DIR="${ONNXRUNTIME_DIR:-$HOME/libs/onnxruntime-linux-aarch64-1.22.0}"
if [[ ! -f "$ORT_DIR/include/onnxruntime_cxx_api.h" || ! -f "$ORT_DIR/lib/libonnxruntime.so" ]]; then
  echo "ERROR: ONNX Runtime C/C++ package not found at $ORT_DIR" >&2
  echo "Run: $WORKSPACE/scripts/install_onnxruntime_aarch64.sh" >&2
  exit 4
fi
export ONNXRUNTIME_DIR="$ORT_DIR"
export LD_LIBRARY_PATH="$ORT_DIR/lib:${LD_LIBRARY_PATH:-}"

cd "$WORKSPACE"
if [[ $CLEAN -eq 1 ]]; then
  rm -rf build install log
fi

if [[ $RUN_ROSDEP -eq 1 ]]; then
  rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
fi

colcon build \
  --symlink-install \
  --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE="$BUILD_TYPE"

source "$WORKSPACE/install/setup.bash"
echo
printf 'Build completed. Overlay: %s\n' "$WORKSPACE/install/setup.bash"
