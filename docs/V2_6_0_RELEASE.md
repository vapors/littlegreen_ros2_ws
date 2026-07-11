# LittleGreen ROS 2 Workspace v2.6.0 Release

## Release purpose

v2.6.0 establishes the permanent LittleGreen naming contract and packages the complete source workspace for a clean Orange Pi installation.

It follows the archived v2.5.1 laboratory-toolbox separation and intentionally avoids changing the validated actuator behavior.

## Breaking identity changes

```text
berkeley_ros2_ws             -> littlegreen_ros2_ws
bhl_*                        -> lgh_*
berkeley_biped_*             -> littlegreen_biped_*
lilgreen_description         -> littlegreen_description
```

The following are renamed throughout active source, manifests, CMake, Python resources, imports, launch files, executables, generated reports, local lock files, and documentation:

| v2.5.1 | v2.6.0 |
|---|---|
| `bhl_st3215_driver` | `lgh_st3215_driver` |
| `bhl_st3215_tools` | `lgh_st3215_tools` |
| `bhl_st3215_maintenance` | `lgh_st3215_maintenance` |
| `bhl_imu_tools` | `lgh_imu_tools` |
| `berkeley_biped_pkg` | `littlegreen_biped_pkg` |
| `berkeley_biped_node` | `littlegreen_biped_node` |
| `lilgreen_description` | `littlegreen_description` |

Generic ROS interfaces remain generic. Examples include `/joint_states`, `/imu/data`, `/desired_position`, `/servo_target_radians`, and `/st3215_driver/*`.

## Complete-install additions

- root `src/`, `docs/`, `scripts/`, and `VERSION` workspace layout;
- complete Ubuntu 22.04/aarch64 Orange Pi installer;
- official ROS 2 apt-source bootstrap with fallback;
- `rosdep` initialization/update and workspace dependency resolution;
- pinned ONNX Runtime 1.22.0 Linux aarch64 installer;
- idempotent LittleGreen shell environment file;
- clean build helper and post-install verifier;
- staged fresh-install commissioning checklist;
- source-tree validation independent of ROS installation;
- package inventory and build manifest;
- pre-v2.6 historical documents preserved under `docs/history/pre_v2_6/`.

## Runtime/tooling boundaries

### `lgh_st3215_driver`

Sole normal runtime ST3215 UART authority. Adds no new control algorithm in this release.

### `lgh_st3215_tools`

Owns guarded calibration, identification, standing characterization, preflight, hardware snapshots, and dataset manifests.

### `lgh_st3215_maintenance`

Offline read-only direct-bus utilities with exclusive UART ownership. No EEPROM write tools are included.

### `lgh_imu_tools`

Validates the canonical `/imu/data` boundary independent of micro-ROS, I2C, or SPI source.

### `littlegreen_biped_pkg`

Retains live, shadow, and disabled policy output modes. Shadow mode shares the live observation and inference path but does not publish to `/desired_position`.

## Driver profiles

Two launch-time publication profiles remain:

```text
commissioning   full characterization publications
runtime_safe    joint state, feedback age, and diagnostics only
```

Profiles do not enable writes and do not change UART timing, feedback reads, joint mapping, control-table settings, or the fixed ST3215 speed/acceleration baseline.

## Preserved behavior and artifacts

Critical servo calibration, limits, driver parameters, policy model bytes, policy joint map, policy runtime configuration, and PD configuration were preserved from v2.5.1 except for active project/package naming references.

The Track 1 task identifier `Velocity-Lilgreen-Humanoid-v0` remains intentionally unchanged inside the currently packaged policy metadata. Track 1 will replace that snapshot with the formal deployment bundle later.

## Explicit exclusions

v2.6.0 does not:

- integrate the future Track 1 policy deployment bundle;
- authorize current packaged policy artifacts for live hardware operation;
- add aggressive outer PD;
- change servo centers, directions, or physical limits;
- change the 50 Hz bus behavior or full feedback read;
- add a direct Orange Pi I2C/SPI IMU driver;
- modify Orange Pi boot overlays or systemd services;
- modify micro-ROS firmware;
- add EEPROM-writing maintenance commands.

## Required on-device acceptance

A static source release cannot prove the Orange Pi runtime. Complete the included fresh-install checklist to validate:

```text
ROS/ONNX installation
colcon build
UART ownership and permissions
read-only servo ID verification
50 Hz feedback and telemetry
ST3215 preflight
IMU contract
runtime-safe profile
policy shadow isolation
guarded startup hold
```
