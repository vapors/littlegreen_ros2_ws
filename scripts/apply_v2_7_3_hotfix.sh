#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

if [[ ! -f VERSION ]]; then
  echo "ERROR: run this script from an extracted LittleGreen workspace hotfix." >&2
  exit 5
fi

version="$(tr -d '[:space:]' < VERSION)"
if [[ "$version" != "2.7.3" ]]; then
  echo "ERROR: expected VERSION=2.7.3 after overlay, found '$version'." >&2
  exit 5
fi

required=(
  docs/COMMAND_CHEATSHEET.md
  docs/COMMAND_REFERENCE.md
  docs/ROS_GRAPH_AND_AUTHORITY.md
  docs/IMU_CALIBRATION.md
  docs/V2_7_3_RELEASE.md
)

for path in "${required[@]}"; do
  if [[ ! -f "$path" ]]; then
    echo "ERROR: missing documentation file: $path" >&2
    exit 5
  fi
done

./scripts/validate_source_tree.py

cat <<'MSG'
LittleGreen v2.7.3 documentation refresh applied.

No ROS package rebuild is required because this release changes documentation,
README files, VERSION, and the hotfix helper only. Rebuild only if you have also
changed source code, YAML configuration, calibration maps, or policy artifacts.

Start here:
  docs/COMMAND_CHEATSHEET.md
  docs/COMMAND_REFERENCE.md
  docs/ROS_GRAPH_AND_AUTHORITY.md
MSG
