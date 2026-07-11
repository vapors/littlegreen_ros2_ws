#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd "$SCRIPT_DIR/.." && pwd)"
SKIP_ROS=0
SKIP_ONNX=0
SKIP_BUILD=0
NO_BASHRC=0

usage() {
  cat <<EOF
Usage: $0 [options]

Complete LittleGreen v2.6.0 install for Orange Pi 5 Max running Ubuntu 22.04 aarch64.

Options:
  --skip-ros       Do not install ROS 2/system dependencies.
  --skip-onnx      Do not download ONNX Runtime 1.22.0.
  --skip-build     Do not run rosdep or colcon build.
  --no-bashrc      Do not add the LittleGreen environment file to ~/.bashrc.
  -h, --help       Show this help.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --skip-ros) SKIP_ROS=1 ;;
    --skip-onnx) SKIP_ONNX=1 ;;
    --skip-build) SKIP_BUILD=1 ;;
    --no-bashrc) NO_BASHRC=1 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "ERROR: Unknown option: $1" >&2; usage; exit 5 ;;
  esac
  shift
done

if [[ $EUID -eq 0 ]]; then
  echo "ERROR: Run this script as your normal user, not as root." >&2
  exit 3
fi

if [[ ! -r /etc/os-release ]]; then
  echo "ERROR: Cannot determine operating system." >&2
  exit 5
fi
# shellcheck disable=SC1091
source /etc/os-release
if [[ "${ID:-}" != "ubuntu" || "${VERSION_ID:-}" != "22.04" ]]; then
  echo "ERROR: Expected Ubuntu 22.04; found ${PRETTY_NAME:-unknown}." >&2
  exit 5
fi
case "$(uname -m)" in
  aarch64|arm64) ;;
  *) echo "ERROR: Expected aarch64/arm64 Orange Pi userspace; found $(uname -m)." >&2; exit 5 ;;
esac

if [[ "$WORKSPACE" != "$HOME/littlegreen_ros2_ws" ]]; then
  echo "WARN: Workspace is at $WORKSPACE"
  echo "      The canonical deployment path is $HOME/littlegreen_ros2_ws"
fi

echo "==> Validating the v2.6.0 source tree"
"$SCRIPT_DIR/validate_source_tree.py"

if [[ $SKIP_ROS -eq 0 ]]; then
  echo "==> Installing ROS 2 Humble and build prerequisites"
  sudo apt-get update
  sudo apt-get install -y locales software-properties-common curl ca-certificates gnupg lsb-release \
    build-essential cmake ninja-build git wget unzip tar pkg-config python3-pip python3-venv
  sudo locale-gen en_US en_US.UTF-8
  sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
  export LANG=en_US.UTF-8
  sudo add-apt-repository universe -y

  if ! dpkg-query -W -f='${Status}' ros2-apt-source 2>/dev/null | grep -q 'install ok installed'; then
    echo "==> Configuring the official ROS 2 apt source"
    ROS_APT_SOURCE_VERSION="${ROS_APT_SOURCE_VERSION:-1.2.0}"
    CODENAME="${UBUNTU_CODENAME:-${VERSION_CODENAME:-jammy}}"
    ROS_APT_DEB="ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${CODENAME}_all.deb"
    ROS_APT_URL="https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/${ROS_APT_DEB}"
    ROS_APT_OK=0
    if curl -fL --retry 3 --retry-delay 2 --connect-timeout 20 \
        -o /tmp/ros2-apt-source.deb "$ROS_APT_URL"; then
      if sudo dpkg -i /tmp/ros2-apt-source.deb; then
        ROS_APT_OK=1
      fi
    fi
    rm -f /tmp/ros2-apt-source.deb
    if [[ $ROS_APT_OK -ne 1 ]]; then
      echo "WARN: ros2-apt-source bootstrap failed; using the official keyring/repository fallback."
      sudo curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
      echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${CODENAME} main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
    fi
  fi

  sudo apt-get update
  # ros-base is intentional on the robot. rosdep installs the additional workspace dependencies.
  sudo apt-get install -y \
    ros-humble-ros-base ros-dev-tools \
    python3-colcon-common-extensions python3-rosdep python3-vcstool python3-yaml \
    libyaml-cpp-dev libeigen3-dev

  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    sudo rosdep init
  fi
  rosdep update

  if ! getent group dialout >/dev/null; then
    sudo groupadd dialout
  fi
  sudo usermod -aG dialout "$USER"
fi

if [[ $SKIP_ONNX -eq 0 ]]; then
  "$SCRIPT_DIR/install_onnxruntime_aarch64.sh"
fi

if [[ $SKIP_BUILD -eq 0 ]]; then
  "$SCRIPT_DIR/build_workspace.sh" --clean
fi

if [[ $NO_BASHRC -eq 0 ]]; then
  LITTLEGREEN_ROS2_WS="$WORKSPACE" "$SCRIPT_DIR/configure_environment.sh"
fi

cat <<EOF

LittleGreen ROS 2 v2.6.0 installation finished.

Next steps:
  1. Log out and back in so the dialout group is active.
  2. Run: source ~/.config/littlegreen/ros2_env.sh
  3. Run: $WORKSPACE/scripts/verify_install.sh
  4. Follow: $WORKSPACE/docs/FRESH_INSTALL_CHECKLIST.md

Do not enable servo writes until feedback-only commissioning passes.
EOF
