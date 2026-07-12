# LittleGreen ROS 2 Workspace

ROS 2 Humble source workspace for the LittleGreen biped hardware stack on the Orange Pi 5 Max.

The current source release is recorded in [`VERSION`](VERSION). This repository is organized around a strict hardware boundary:

| Package | Responsibility |
|---|---|
| `lgh_st3215_driver` | Sole normal runtime owner of the ST3215 UART bus |
| `lgh_st3215_tools` | Guarded calibration, characterization, preflight, auditing, and datasets |
| `lgh_st3215_maintenance` | Offline read-only direct-bus inspection; the driver must be stopped |
| `lgh_imu_tools` | Source-independent validation of the canonical `/imu/data` interface |
| `littlegreen_biped_pkg` | Observation construction, ONNX inference, and live/shadow/disabled policy output |
| `pd_controller_pkg` | Safety filtering and optional outer-loop command shaping |
| `littlegreen_description` | Robot description and visualization resources |

## Start here

### Fresh Orange Pi installation

```bash
cd ~/littlegreen_ros2_ws
./scripts/validate_source_tree.py
./scripts/install_orange_pi.sh
```

The installer adds the LittleGreen environment file to `~/.bashrc`. New interactive Bash terminals load it automatically. In the terminal that ran the installer, either open a new terminal or run:

```bash
source ~/.bashrc
```

Then verify the software installation:

```bash
~/littlegreen_ros2_ws/scripts/verify_install.sh
```

Full instructions: [`docs/INSTALL_ORANGE_PI.md`](docs/INSTALL_ORANGE_PI.md)

### First feedback-only launch

Keep the robot securely supported and servo writes disabled:

```bash
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=commissioning \
  enable_writes:=false
```

In another terminal:

```bash
ros2 run lgh_st3215_tools st3215_preflight \
  --mode feedback \
  --expect-writes false
```

Continue with [`docs/FRESH_INSTALL_CHECKLIST.md`](docs/FRESH_INSTALL_CHECKLIST.md).



### ShadowMode (for Isaac Sim ) LittleGreen Ubuntu 22.04 x86_64 installer


```bash
cd ~/littlegreen_ros2_ws
chmod +x scripts/install_ubuntu_x86_64.sh scripts/install_onnxruntime_x86_64.sh
./scripts/install_ubuntu_x86_64.sh
```

Supported options:

```text
--skip-ros
--skip-onnx
--skip-build
--no-bashrc
```

The default ONNX Runtime path is:

```text
~/libs/onnxruntime-linux-x64-1.22.0
```

[ add more about shadow mode and launcher here ]


This installer uses ROS 2 Humble `ros-base` and does not install Gazebo.

## Driver profiles

Profiles select the ROS publication surface. They do **not** enable writes or change bus timing, register reads, joint mapping, or the ST3215 motion profile.

| Profile | Intended use | High-rate laboratory topics |
|---|---|---|
| `commissioning` | Calibration, identification, hardware auditing | Enabled |
| `runtime_safe` | Policy shadow and initial deployment observation | Disabled |

`enable_writes` remains a separate, explicit launch argument.

## Safety boundary

- Servo writes are disabled by default.
- Maintenance commands are read-only and must not run while the runtime driver owns the UART.
- Policy shadow mode does not publish on `/desired_position`.
- Software pose holds and torque services are not electrical emergency stops.
- Initial commissioning must be performed with the robot mechanically supported and the physical servo-power disconnect immediately accessible.
- Aggressive outer-PD tuning is outside the current commissioning path.

## Documentation

The current operator documentation is indexed in [`docs/README.md`](docs/README.md).

Most-used pages:

- [`docs/INSTALL_ORANGE_PI.md`](docs/INSTALL_ORANGE_PI.md) — clean host installation
- [`docs/FRESH_INSTALL_CHECKLIST.md`](docs/FRESH_INSTALL_CHECKLIST.md) — staged acceptance gates
- [`docs/COMMAND_CHEATSHEET.md`](docs/COMMAND_CHEATSHEET.md) — common commands
- [`docs/INTERFACES_AND_PARAMETERS.md`](docs/INTERFACES_AND_PARAMETERS.md) — launch arguments, profiles, topics, services, and parameters
- [`docs/CALIBRATION_WORKFLOW.md`](docs/CALIBRATION_WORKFLOW.md) — guarded center calibration
- [`docs/HARDWARE_CONTRACT.md`](docs/HARDWARE_CONTRACT.md) — authoritative joint limits and runtime ownership
- [`docs/SAFETY_AND_LIMITATIONS.md`](docs/SAFETY_AND_LIMITATIONS.md) — current boundaries and deferred work
- [`docs/VALIDATION.md`](docs/VALIDATION.md) — current source/documentation validation record

Historical release and migration records remain available under `docs/archive/` and `docs/history/`, but they are not part of the active operating instructions.

## Supported deployment baseline

- Orange Pi 5 Max, aarch64
- Ubuntu 22.04 Jammy
- ROS 2 Humble
- ONNX Runtime C/C++ 1.22.0
- ST3215 bus on `/dev/ttyS3` at 1,000,000 baud by default

The installer does not modify Orange Pi boot overlays, UART pinmux, servo power wiring, micro-ROS firmware, direct IMU configuration, or systemd services.
