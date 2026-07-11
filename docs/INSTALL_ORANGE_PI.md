# Complete Orange Pi Installation — LittleGreen ROS 2 v2.6.3

This guide installs a clean `~/littlegreen_ros2_ws` deployment on an Orange Pi 5 Max running Ubuntu 22.04 aarch64.

The automated installer performs the software setup, but it deliberately does **not** modify Orange Pi boot overlays, UART pinmux, servo power wiring, micro-ROS firmware, direct IMU configuration, systemd units, or external shell scripts.

## 1. Before replacing the old workspace

Archive the previous source and any local data that are not already backed up:

```bash
mkdir -p ~/workspace_archives

tar -czf ~/workspace_archives/berkeley_ros2_ws_v2_5_1_local_$(date -u +%Y%m%dT%H%M%SZ).tar.gz \
  -C ~ berkeley_ros2_ws
```

Also preserve any locally modified:

```text
servo_map.yaml
policy bundles
systemd units
udev rules
shell aliases
micro-ROS launch scripts
calibration reports
identification and standing datasets
```

Do not source the old workspace while installing the new one.

## 2. Place the v2.6.3 workspace

The distributed ZIP contains one top-level directory named `littlegreen_ros2_ws`.

Verify the downloaded archive when the `.sha256` file is beside it:

```bash
sha256sum -c littlegreen_ros2_ws_v2_6_0.sha256
```

Then extract:

```bash
cd ~
unzip littlegreen_ros2_ws_v2_6_0.zip
cd ~/littlegreen_ros2_ws
```

Confirm the source-tree release:

```bash
cat VERSION
# expected: 2.6.0
```

Make scripts executable if the archive tool did not preserve permissions:

```bash
chmod +x scripts/*.sh scripts/*.py
```

Run the ROS-independent source audit before installing anything:

```bash
./scripts/validate_source_tree.py
```

Expected result:

```text
SOURCE VALIDATION: PASS
```

A warning about the historical Track 1 task identifier `Velocity-Lilgreen-Humanoid-v0` is expected until Track 1 supplies the new deployment bundle.

## 3. Confirm the operating-system baseline

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

The install script refuses another distribution, Ubuntu release, or architecture rather than attempting a partial deployment.

## 4. Run the complete installer

Run as your normal login user, not root:

```bash
cd ~/littlegreen_ros2_ws
./scripts/install_orange_pi.sh
```

The installer:

1. validates Ubuntu 22.04 and aarch64;
2. installs locale and build prerequisites;
3. configures the official ROS 2 apt source;
4. installs ROS 2 Humble `ros-base`, ROS development tools, `rosdep`, and `colcon`;
5. initializes or updates `rosdep`;
6. adds the current user to `dialout`;
7. downloads and extracts ONNX Runtime 1.22.0 for Linux aarch64;
8. resolves workspace dependencies with `rosdep`;
9. performs a clean `colcon build --symlink-install`;
10. creates `~/.config/littlegreen/ros2_env.sh` and an idempotent `.bashrc` source block.

The installer uses `apt-get update` and explicit package installation. It does not perform a broad distribution upgrade, preserving the host’s Orange Pi BSP/kernel management boundary.

### Installer options

```text
--skip-ros       keep an existing ROS/system dependency installation
--skip-onnx      keep an existing ONNX Runtime installation
--skip-build     configure dependencies but do not build

`--skip-onnx` reuses an existing ONNX Runtime installation; it does not remove the ONNX Runtime build dependency. The default expected location is `~/libs/onnxruntime-linux-aarch64-1.22.0`.
--no-bashrc      create no automatic shell-source block
```

Example for an already configured ROS host:

```bash
./scripts/install_orange_pi.sh --skip-ros
```

## 5. Offline ONNX Runtime installation

The policy node is a C++ executable and requires the ONNX Runtime C/C++ archive, not only the Python package.

The default expected path is:

```text
~/libs/onnxruntime-linux-aarch64-1.22.0
```

For an offline host, copy this archive onto the Orange Pi:

```text
onnxruntime-linux-aarch64-1.22.0.tgz
```

Then run:

```bash
ONNXRUNTIME_ARCHIVE=/path/to/onnxruntime-linux-aarch64-1.22.0.tgz \
  ./scripts/install_onnxruntime_aarch64.sh
```

The installer records the downloaded archive checksum and source URL inside the extracted ONNX Runtime directory.

## 6. Log out and back in

The `dialout` group change does not affect the current login session. Log out and back in, or reboot cleanly:

```bash
sudo reboot
```

After reconnecting:

```bash
id -nG
```

Confirm `dialout` appears.

## 7. Load the new environment

The installer adds the LittleGreen environment file to `~/.bashrc`, so every new interactive Bash terminal loads it automatically. You do not need to add another source line.

For the terminal that was already open during installation, either open a new terminal or run:

```bash
source ~/.bashrc
```

Directly sourcing the generated file is equivalent but optional:

```bash
source ~/.config/littlegreen/ros2_env.sh
```

The generated file defines:

```text
LITTLEGREEN_ROS2_WS=~/littlegreen_ros2_ws
ONNXRUNTIME_DIR=~/libs/onnxruntime-linux-aarch64-1.22.0
LD_LIBRARY_PATH=$ONNXRUNTIME_DIR/lib:...
```

Inspect the active workspace:

```bash
printf '%s\n' "$LITTLEGREEN_ROS2_WS"
printf '%s\n' "$ONNXRUNTIME_DIR"
```

## 8. Verify the software installation

```bash
cd ~/littlegreen_ros2_ws
./scripts/verify_install.sh --software-only
```

The check verifies:

- ROS 2 Humble base;
- ONNX Runtime headers and shared library;
- the built workspace overlay;
- discovery of all LittleGreen packages;
- absence of old package names in the active overlay;
- the source-tree rename/syntax audit.

Exit codes:

```text
0  pass
2  pass with warnings
3  failure
```

List the renamed packages:

```bash
ros2 pkg list | grep -E '^(lgh_|littlegreen_)'
```

Confirm old package names are not sourced:

```bash
ros2 pkg list | grep -E '^(bhl_|berkeley_biped|lilgreen_)'
```

The second command should return no output. If old packages remain visible, open a clean shell and inspect `AMENT_PREFIX_PATH` for an old overlay.

## 9. Manual rebuild workflow

For later source changes:

```bash
cd ~/littlegreen_ros2_ws
./scripts/build_workspace.sh
```

Clean rebuild:

```bash
./scripts/build_workspace.sh --clean
```

The build helper runs:

```bash
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
colcon build --symlink-install
```

## 10. Confirm UART prerequisites

The installer does not enable UART overlays. Verify the UART established during Track 2 testing still exists:

```bash
ls -l /dev/ttyS3
```

Check access:

```bash
test -r /dev/ttyS3 && test -w /dev/ttyS3 && echo UART_ACCESS_OK
```

The default driver launch uses `/dev/ttyS3` at 1,000,000 baud. Override the port only when intentionally testing another configured UART:

```bash
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  port:=/dev/ttyS3 \
  profile:=commissioning \
  enable_writes:=false
```

## 11. IMU boundary

v2.6.3 contains source-independent IMU tools, not a new direct Orange Pi I2C/SPI driver. The current micro-ROS source or a future direct driver must publish the canonical `/imu/data` topic before these pass:

```bash
ros2 run lgh_imu_tools imu_preflight
ros2 run lgh_imu_tools stationary_characterization --duration-sec 20
```

Do not treat `override_imu:=true` as a substitute for hardware IMU commissioning.

## 12. Continue with staged commissioning

Do not enable servo writes immediately after installation. Continue with:

[`FRESH_INSTALL_CHECKLIST.md`](FRESH_INSTALL_CHECKLIST.md)

The first hardware launch is feedback-only and uses the `commissioning` profile.


## Troubleshooting strict-shell setup errors

v2.6.3 guards ROS environment sourcing against Bash `set -u`. If an older workspace reports:

```text
/opt/ros/humble/setup.bash: line 8: AMENT_TRACE_SETUP_FILES: unbound variable
```

apply the v2.6.3 shell hotfix or temporarily resume manually with:

```bash
set +u
source /opt/ros/humble/setup.bash
set -u
```

The manual form is only a temporary workaround; the v2.6.3 scripts preserve and restore the caller's original nounset state automatically.
