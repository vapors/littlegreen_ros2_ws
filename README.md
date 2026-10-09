# LittleGreen ROS 2 Workspace

ROS 2 Humble source workspace for the LittleGreen biped hardware stack. The active release is recorded in [`VERSION`](VERSION).

## Package boundaries

| Package | Responsibility |
|---|---|
| `lgh_st3215_driver` | Sole normal runtime owner of the ST3215 UART bus |
| `lgh_st3215_tools` | Guarded calibration, characterization, preflight, hardware auditing, and datasets |
| `lgh_st3215_maintenance` | Offline read-only direct-bus inspection; the runtime driver must be stopped |
| `lgh_imu_tools` | Source-independent validation of the canonical `/imu/data` interface |
| `littlegreen_biped_pkg` | Shared 47-D Stand/Walk/Locomotion observation construction, task-specific phase state including v10.2 neutral-static mode, action-contract v3/v4 validation, exported command-envelope clamping, ONNX inference, policy auditing, golden-vector hooks, runtime metrics, and live/shadow/disabled output |
| `pd_controller_pkg` | Safety filtering and optional outer-loop command shaping |
| `littlegreen_description` | Robot description and visualization resources |

## Current Track 1 deployment status

Workspace v2.9.4 packages the audited LittleGreen Humanoid Lite v10.2 `model_10000` locomotion bundle as the active source policy:

```text
Task:                 Velocity-Lilgreen-Locomotion-ST3215-Loaded-v102
Interface:            observation[47] -> action[12]
Rate:                 50 Hz
Observation contract: littlegreen_velocity_47d_phase_v1
Phase mode:           neutral_static -> [0.0, 1.0]
Action contract:      v4 bounded default-centered vector residual
ONNX SHA-256:          da7adcaf996809bfb7a413bfec712dd2f3bd2ed984f8f661af2f9cc9c4d7872e
```

The source tree, checkpoint mirror, deployment contract, manifest, and checksum files are synchronized to that same policy identity. Rebuilding the package therefore no longer risks silently reverting the installed runtime to the older Stand bundle.

### Learned zero-command live handoff

`q_default` remains the protected Track-1 observation/action reference. Live locomotion instead enters policy control from the Track-1 learned zero-command standing state plus an explicit robot-specific hardware trim. The handoff profile records three separate layers:

```text
Track-1 source median pose
        +
robot-specific hardware_trim_rad
        +
documented physical-limit clamp(s)
        =
audited effective Track-2 handoff pose
```

For the current robot, only bilateral hip-pitch (`-0.035 rad`) and ankle-pitch (`-0.100 rad`) hardware trims are applied. The resulting supported pose passed the orientation audit at approximately 1.7 degrees total tilt. `obs[33:45]` is seeded with the matched Track-1 bounded previous action before the first live policy inference. Power-on behavior remains hold-current-position; there is no automatic startup motion. See [`docs/V2_9_4_HANDOFF_CALIBRATION.md`](docs/V2_9_4_HANDOFF_CALIBRATION.md).

The former v2.3.1 Stand and legacy 45-D artifacts remain historical/rollback references; they are not the active v2.9.4 source policy.

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
- Do not edit `q_default`, action defaults, residual scales, or joint limits to tune physical handoff posture. Robot-specific policy-entry calibration belongs only in the audited `hardware_trim_rad` layer; Track-1 policy semantics remain unchanged.

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
- [`docs/V2_9_4_HANDOFF_CALIBRATION.md`](docs/V2_9_4_HANDOFF_CALIBRATION.md)
- [`docs/V2_9_4_VALIDATION.md`](docs/V2_9_4_VALIDATION.md)
- [`docs/V2_9_3_POLICY_HANDOFF.md`](docs/V2_9_3_POLICY_HANDOFF.md) — original handoff design record
- [`docs/VALIDATION.md`](docs/VALIDATION.md)

Historical records are retained under `docs/archive/` and `docs/history/` and are not active operating instructions.

## Supported baseline

- Orange Pi 5 Max aarch64 for robot deployment
- Ubuntu 22.04 x86_64 for host-side build and inspection
- ROS 2 Humble
- ONNX Runtime C/C++ 1.22.0
- ST3215 bus on `/dev/ttyS3` at 1,000,000 baud by default

The installer does not modify Orange Pi boot overlays, UART pinmux, servo power wiring, micro-ROS firmware, direct IMU configuration, or systemd services.
