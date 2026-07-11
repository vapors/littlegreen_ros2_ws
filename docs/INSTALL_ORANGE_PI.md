# Orange Pi Installation

This guide installs a clean `~/littlegreen_ros2_ws` deployment on an Orange Pi 5 Max running Ubuntu 22.04 aarch64.

The installer handles the software environment. It deliberately does **not** modify Orange Pi boot overlays, UART pinmux, servo power wiring, micro-ROS firmware, direct IMU configuration, systemd units, or external shell scripts.

## 1. Place the workspace

Clone or extract the repository so the canonical path is:

```text
~/littlegreen_ros2_ws
```

Then:

```bash
cd ~/littlegreen_ros2_ws
cat VERSION
chmod +x scripts/*.sh scripts/*.py
./scripts/validate_source_tree.py
```

Expected:

```text
SOURCE VALIDATION: PASS
```

Warnings about retained source provenance or the current Track 1 task identifier are informational.

## 2. Confirm the host baseline

```bash
source /etc/os-release
printf 'OS: %s %s\n' "$ID" "$VERSION_ID"
uname -m
```

Expected:

```text
ubuntu 22.04
aarch64
```

The installer refuses unsupported distributions, Ubuntu releases, and architectures rather than attempting a partial installation.

## 3. Run the installer

Run as the normal login user, not root:

```bash
cd ~/littlegreen_ros2_ws
./scripts/install_orange_pi.sh
```

The installer:

1. validates Ubuntu 22.04 and aarch64;
2. installs build and locale prerequisites;
3. configures the official ROS 2 apt source;
4. installs ROS 2 Humble `ros-base`, development tools, `rosdep`, and `colcon`;
5. initializes or updates `rosdep`;
6. adds the current user to `dialout`;
7. installs ONNX Runtime 1.22.0 for Linux aarch64;
8. resolves workspace dependencies;
9. performs a clean `colcon build --symlink-install`;
10. creates `~/.config/littlegreen/ros2_env.sh`;
11. adds one managed LittleGreen source block to `~/.bashrc`.

The robot image intentionally uses `ros-base`; Gazebo is not required.

### Installer options

```text
--skip-ros       keep an existing ROS/system dependency installation
--skip-onnx      reuse an existing ONNX Runtime installation
--skip-build     install/configure dependencies but do not run rosdep or colcon
--no-bashrc      do not add the environment source block to ~/.bashrc
```

Examples:

```bash
./scripts/install_orange_pi.sh --skip-ros
./scripts/install_orange_pi.sh --skip-ros --skip-onnx
```

`--skip-onnx` expects ONNX Runtime to already exist at:

```text
~/libs/onnxruntime-linux-aarch64-1.22.0
```

or at the path supplied in `ONNXRUNTIME_DIR`.

## 4. Offline ONNX Runtime installation

The policy node requires the ONNX Runtime C/C++ archive, not only the Python package.

For an offline host, copy this archive to the Orange Pi:

```text
onnxruntime-linux-aarch64-1.22.0.tgz
```

Then:

```bash
ONNXRUNTIME_ARCHIVE=/path/to/onnxruntime-linux-aarch64-1.22.0.tgz \
  ./scripts/install_onnxruntime_aarch64.sh
```

## 5. Activate group membership and shell environment

The `dialout` group change requires a new login session. Log out and back in, or reboot:

```bash
sudo reboot
```

The installer already adds this managed block to `~/.bashrc`:

```bash
# >>> LittleGreen ROS 2 environment >>>
source "/home/<user>/.config/littlegreen/ros2_env.sh"
# <<< LittleGreen ROS 2 environment <<<
```

Do not add a duplicate source line.

New interactive Bash terminals load the environment automatically. In the terminal that ran the installer, either open a new terminal or run:

```bash
source ~/.bashrc
```

Directly sourcing the generated environment file is equivalent but optional:

```bash
source ~/.config/littlegreen/ros2_env.sh
```

Scripts and systemd launch wrappers that do not read `~/.bashrc` should source the generated environment file explicitly.

## 6. Verify the software installation

```bash
cd ~/littlegreen_ros2_ws
./scripts/verify_install.sh --software-only
```

Exit codes:

| Code | Meaning |
|---:|---|
| `0` | pass |
| `2` | pass with warnings |
| `3` | failure |

Confirm the active packages:

```bash
ros2 pkg list | grep -E '^(lgh_|littlegreen_)'
```

Confirm the driver resolves inside this workspace:

```bash
ros2 pkg prefix lgh_st3215_driver
```

Expected prefix:

```text
/home/<user>/littlegreen_ros2_ws/install/lgh_st3215_driver
```

## 7. Rebuild after source changes

Normal rebuild:

```bash
cd ~/littlegreen_ros2_ws
./scripts/build_workspace.sh
```

Clean rebuild:

```bash
./scripts/build_workspace.sh --clean
```

Useful options:

```text
--skip-rosdep
--release
--debug
```

## 8. Confirm UART access

The installer does not enable UART overlays. Verify the host configuration established for `/dev/ttyS3` is still present:

```bash
ls -l /dev/ttyS3
id -nG | tr ' ' '\n' | grep '^dialout$'
test -r /dev/ttyS3 && test -w /dev/ttyS3 && echo UART_ACCESS_OK
```

Do not apply servo power or enable writes until the staged checklist reaches that gate.

## 9. Confirm the IMU boundary

The workspace includes source-independent IMU tools, not a direct Orange Pi I2C/SPI IMU driver. The current micro-ROS source or a future direct driver must publish `/imu/data`.

```bash
ros2 run lgh_imu_tools imu_preflight
ros2 run lgh_imu_tools stationary_characterization --duration-sec 20
```

`override_imu:=true` is a software test option, not a hardware commissioning substitute.

## 10. Troubleshooting

### APT reports conflicting `Signed-By` values

The installer normally disables a duplicate legacy ROS source automatically. If APT still reports a conflict, inspect the active ROS entries:

```bash
grep -RnsE \
  'packages\.ros\.org/ros2/ubuntu|repo\.ros2\.org' \
  /etc/apt/sources.list \
  /etc/apt/sources.list.d \
  2>/dev/null
```

When both `ros2.sources` and an older `ros2.list` define the same repository, keep `ros2.sources` and disable the older file:

```bash
sudo mv /etc/apt/sources.list.d/ros2.list \
  /etc/apt/sources.list.d/ros2.list.disabled

sudo apt-get clean
sudo apt-get update
sudo dpkg --configure -a
sudo apt-get --fix-broken install
```

### `rosdep` fails

First repair APT, then rerun:

```bash
source /opt/ros/humble/setup.bash
cd ~/littlegreen_ros2_ws
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
```

The hardware workspace does not require `gazebo_ros`.

### Current shell cannot see the workspace

```bash
source ~/.bashrc
ros2 pkg prefix lgh_st3215_driver
```

Do not add another LittleGreen environment line to `~/.bashrc`.

## 11. Continue with staged commissioning

Do not enable servo writes immediately after installation.

Continue with [`FRESH_INSTALL_CHECKLIST.md`](FRESH_INSTALL_CHECKLIST.md).
