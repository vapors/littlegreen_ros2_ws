#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "$SCRIPT_DIR/.." && pwd)"
VERSION="$(tr -d '\r\n' < "$WORKSPACE/VERSION")"

if [[ "$VERSION" != "2.7.2" ]]; then
  echo "ERROR: expected VERSION 2.7.2 after hotfix extraction; found $VERSION" >&2
  exit 5
fi

"$SCRIPT_DIR/validate_source_tree.py"

cat <<EOF

LittleGreen v2.7.2 calibration hotfix is present and source validation passed.

Rebuild the affected packages:
  cd $WORKSPACE
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  rm -rf build/lgh_st3215_driver install/lgh_st3215_driver \\
         build/lgh_st3215_tools install/lgh_st3215_tools
  colcon build --symlink-install --packages-select \\
    lgh_st3215_driver lgh_st3215_tools
  source install/setup.bash

If apply_calibration later changes the joint_map.yaml mirror, also rebuild:
  littlegreen_biped_pkg
EOF
