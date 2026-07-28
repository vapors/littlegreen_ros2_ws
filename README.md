# LittleGreen ROS 2 Workspace

ROS 2 Humble source workspace for the LittleGreen biped hardware stack. The active release is recorded in [`VERSION`](VERSION).

## Package boundaries

| Package | Responsibility |
|---|---|
| `lgh_st3215_driver` | Sole normal runtime owner of the ST3215 UART bus |
| `lgh_st3215_tools` | Guarded calibration, characterization, preflight, hardware auditing, and datasets |
| `lgh_st3215_maintenance` | Offline read-only direct-bus inspection; the runtime driver must be stopped |
| `lgh_imu_tools` | Source-independent validation of the canonical `/imu/data` interface |
| `littlegreen_biped_pkg` | Shared 47-D Stand/Walk observation construction, task-specific phase state, action-contract v3/v4 validation, ONNX inference, policy auditing, golden-vector hooks, runtime metrics, and live/shadow/disabled output |
| `pd_controller_pkg` | Safety filtering and optional outer-loop command shaping |
| `littlegreen_description` | Robot description and visualization resources |

## Current Track 1 deployment status

v2.9.0 packages the complete LittleGreen Humanoid Lite v2.3.1 canonical Stand bundle:

```text
Task:                 Velocity-Lilgreen-Stand-ST3215-Loaded-v23
Interface:            observation[47] -> action[12]
Rate:                 50 Hz
Observation contract: littlegreen_velocity_47d_phase_v1
Phase mode:           randomized_static_per_episode
Action contract:      v4 bounded default-centered vector residual
ONNX SHA-256:          66936666934ff02e75681b8ae5c2c6727021df598419016ce0479e8466282608
```

The Stand policy receives the exact shared actor observation exported by Track 1. Its phase is sampled uniformly once at the beginning of an intentional policy episode and the sine/cosine pair remains unchanged for the episode. It does not advance with time, command, contact, inference count, or transient readiness loss.

The former 45-D v2.8.0 policy is retained as a paired rollback artifact under `configs/legacy_v280_45d/`; it is not the active default. Future Walk construction is present for contract testing and shadow inspection, but live Walk remains blocked until the exported checkpoint explicitly pins its deployment stage or period. See [`docs/OBSERVATION_CONTRACT.md`](docs/OBSERVATION_CONTRACT.md).

## Install

### Orange Pi 5 Max

```bash
cd ~/littlegreen_ros2_ws
./scripts/validate_source_tree.py
./scripts/install_orange_pi.sh
```

### Ubuntu 22.04 x86_64 host

```bash
cd ~/littlegreen_ros2_ws
./scripts/validate_source_tree.py
./scripts/install_ubuntu_x86_64.sh
```

The installer adds the LittleGreen environment script to `~/.bashrc`. New interactive terminals load it automatically. In the installation terminal, open a new terminal or run:

```bash
source ~/.bashrc
```

Manual sourcing of `~/.config/littlegreen/ros2_env.sh` is optional for interactive shells and remains useful for non-interactive scripts or services.

## First feedback-only launch

Keep the robot mechanically supported and writes disabled:

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

For the current micro-ROS IMU source, use a dedicated terminal whenever `/imu/data` is required:

```bash
ros2 run micro_ros_agent micro_ros_agent serial \
  --dev /dev/ttyACM0 \
  -b 115200 \
  -v0
```

Continue with [`docs/FRESH_INSTALL_CHECKLIST.md`](docs/FRESH_INSTALL_CHECKLIST.md).

## Policy bundle audit and shadow

Audit the packaged YAML/ONNX pair before launch. The installed audit verifies SHA-256 and the actual ONNX input/output tensor shapes:

```bash
ros2 run littlegreen_biped_pkg policy_bundle_audit
```

Then launch shadow mode with a feedback-only driver and a validated IMU source:

```bash
ros2 launch littlegreen_biped_pkg policy_shadow.launch.py
```

For a short Track 1-aligned runtime metrics capture:

```bash
ros2 run littlegreen_biped_pkg policy_runtime_metrics \
  --duration-sec 30
```

See [`docs/LIVE_POLICY_DEPLOYMENT.md`](docs/LIVE_POLICY_DEPLOYMENT.md) and [`docs/TRACK1_TRACK2_POLICY_METRICS.md`](docs/TRACK1_TRACK2_POLICY_METRICS.md).

## Driver profiles

Profiles select the ROS publication surface. They do **not** enable writes or alter bus timing, joint mapping, register reads, or the ST3215 motion profile.

| Profile | Intended use | High-rate laboratory topics |
|---|---|---|
| `commissioning` | Calibration, identification, hardware auditing | Enabled |
| `runtime_safe` | Policy shadow and guarded live deployment | Disabled |

`enable_writes` remains a separate explicit launch argument.

## Safety boundary

- Servo writes are disabled by default.
- Maintenance commands are read-only and must not run while the runtime driver owns the UART.
- Policy shadow mode never publishes on `/desired_position`.
- Software pose holds and torque services are not electrical emergency stops.
- Commissioning and first live runs require mechanical support and immediate access to servo power disconnect.
- Initial live deployment uses `controller_mode:=safety_only`; aggressive outer-PD tuning remains outside this release.
- Do not edit ROS-side defaults to compensate for a Track 1 posture issue. Update and re-export the paired policy contract instead.

## Documentation

Start with [`docs/README.md`](docs/README.md). Common pages:

- [`docs/INSTALL_ORANGE_PI.md`](docs/INSTALL_ORANGE_PI.md)
- [`docs/FRESH_INSTALL_CHECKLIST.md`](docs/FRESH_INSTALL_CHECKLIST.md)
- [`docs/COMMAND_CHEATSHEET.md`](docs/COMMAND_CHEATSHEET.md)
- [`docs/COMMAND_REFERENCE.md`](docs/COMMAND_REFERENCE.md)
- [`docs/ROS_GRAPH_AND_AUTHORITY.md`](docs/ROS_GRAPH_AND_AUTHORITY.md)
- [`docs/INTERFACES_AND_PARAMETERS.md`](docs/INTERFACES_AND_PARAMETERS.md)
- [`docs/OBSERVATION_CONTRACT.md`](docs/OBSERVATION_CONTRACT.md)
- [`docs/TRACK1_V2_3_1_INTEGRATION_AUDIT.md`](docs/TRACK1_V2_3_1_INTEGRATION_AUDIT.md)
- [`docs/LIVE_POLICY_DEPLOYMENT.md`](docs/LIVE_POLICY_DEPLOYMENT.md)
- [`docs/TRACK1_TRACK2_POLICY_METRICS.md`](docs/TRACK1_TRACK2_POLICY_METRICS.md)
- [`docs/CALIBRATION_WORKFLOW.md`](docs/CALIBRATION_WORKFLOW.md)
- [`docs/SERVO_REPLACEMENT_CHECKLIST.md`](docs/SERVO_REPLACEMENT_CHECKLIST.md)
- [`docs/HARDWARE_CONTRACT.md`](docs/HARDWARE_CONTRACT.md)
- [`docs/SAFETY_AND_LIMITATIONS.md`](docs/SAFETY_AND_LIMITATIONS.md)
- [`docs/V2_9_0_RELEASE.md`](docs/V2_9_0_RELEASE.md)
- [`docs/V2_9_0_VALIDATION.md`](docs/V2_9_0_VALIDATION.md)
- [`docs/VALIDATION.md`](docs/VALIDATION.md)

Historical records are retained under `docs/archive/` and `docs/history/` and are not active operating instructions.

## Supported baseline

- Orange Pi 5 Max aarch64 for robot deployment
- Ubuntu 22.04 x86_64 for host-side build and inspection
- ROS 2 Humble
- ONNX Runtime C/C++ 1.22.0
- ST3215 bus on `/dev/ttyS3` at 1,000,000 baud by default

The installer does not modify Orange Pi boot overlays, UART pinmux, servo power wiring, micro-ROS firmware, direct IMU configuration, or systemd services.
