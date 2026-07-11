# LittleGreen ROS 2 Workspace v2.6.0

Complete ROS 2 Humble source workspace for the LittleGreen biped hardware stack on the Orange Pi 5 Max.

v2.6.0 is the first workspace released under the LittleGreen naming contract:

```text
workspace                 littlegreen_ros2_ws
hardware package prefix   lgh_
policy package stem       littlegreen_biped
robot description         littlegreen_description
```

The control, calibration, joint-map, and servo-map behavior is inherited from the archived v2.5.1 baseline. The breaking change in v2.6.0 is package, executable, file, report-directory, and workspace identity.

## Package boundaries

| Package | Responsibility |
|---|---|
| `lgh_st3215_driver` | Sole normal runtime owner of the ST3215 UART bus |
| `lgh_st3215_tools` | Guarded calibration, characterization, preflight, auditing, and datasets |
| `lgh_st3215_maintenance` | Offline read-only direct-bus inspection; driver must be stopped |
| `lgh_imu_tools` | Source-independent `/imu/data` validation and recording |
| `littlegreen_biped_pkg` | Observation construction, ONNX inference, live/shadow/disabled policy output |
| `pd_controller_pkg` | Existing safety envelope and optional outer-loop command shaping |
| `littlegreen_description` | Robot description, visualization, and related launch resources |

Generic ROS packages and generic topic names retain their existing names.

## Supported deployment baseline

- Orange Pi 5 Max, aarch64
- Ubuntu 22.04 Jammy
- ROS 2 Humble
- ONNX Runtime C/C++ 1.22.0
- ST3215 bus on `/dev/ttyS3` at 1,000,000 baud by default

The installer does not modify Orange Pi boot overlays, UART pinmux, power configuration, micro-ROS firmware, or systemd services. Those remain host-specific commissioning steps.

## Fresh installation

Extract or clone this directory to:

```bash
~/littlegreen_ros2_ws
```

Then run as the normal user:

```bash
cd ~/littlegreen_ros2_ws
./scripts/validate_source_tree.py
./scripts/install_orange_pi.sh
```

Log out and back in after installation so the `dialout` membership is active, then:

```bash
source ~/.config/littlegreen/ros2_env.sh
~/littlegreen_ros2_ws/scripts/verify_install.sh
```

Full instructions: [`docs/INSTALL_ORANGE_PI.md`](docs/INSTALL_ORANGE_PI.md)

Hardware commissioning sequence: [`docs/FRESH_INSTALL_CHECKLIST.md`](docs/FRESH_INSTALL_CHECKLIST.md)

## Safety boundary

- Servo writes are disabled by default.
- Maintenance commands are read-only in v2.6.0.
- Policy shadow mode does not publish on `/desired_position`.
- Software pose holds and torque commands are not electrical emergency stops.
- The robot must remain securely supported during initial commissioning, with physical power disconnect immediately accessible.

## First feedback-only launch

```bash
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=commissioning \
  enable_writes:=false
```

In another shell:

```bash
ros2 run lgh_st3215_tools st3215_preflight \
  --mode feedback \
  --expect-writes false
```

## Documentation map

- [`docs/V2_6_0_RELEASE.md`](docs/V2_6_0_RELEASE.md) — release scope and exclusions
- [`docs/INSTALL_ORANGE_PI.md`](docs/INSTALL_ORANGE_PI.md) — complete software install
- [`docs/FRESH_INSTALL_CHECKLIST.md`](docs/FRESH_INSTALL_CHECKLIST.md) — staged commissioning
- [`docs/MIGRATION_V2_5_1_TO_V2_6_0.md`](docs/MIGRATION_V2_5_1_TO_V2_6_0.md) — rename map
- [`docs/V2_6_0_REFERENCE.md`](docs/V2_6_0_REFERENCE.md) — package and command reference
- [`docs/COMMAND_CHEATSHEET.md`](docs/COMMAND_CHEATSHEET.md) — common commands
- [`docs/KNOWN_LEGACY_AND_CAVEATS.md`](docs/KNOWN_LEGACY_AND_CAVEATS.md) — intentional legacy references and limits
- [`docs/V2_6_0_VALIDATION.md`](docs/V2_6_0_VALIDATION.md) — static validation record and on-device boundary
- [`docs/V2_6_0_BUILD_MANIFEST.yaml`](docs/V2_6_0_BUILD_MANIFEST.yaml) — package and critical-artifact hashes
- [`docs/history/pre_v2_6/`](docs/history/pre_v2_6/) — preserved historical records

## Validation status

The distributed source archive is statically validated for package manifests, Python syntax, YAML/XML parsing, shell syntax, active-name migration, entry points, selected C++ source compilation without ROS, and preservation of critical v2.5.1 configuration/model artifacts. A real ROS 2 build, UART test, IMU test, and hardware acceptance run must be completed on the Orange Pi.
