# Berkeley-Humanoid-Lite ROS 2 Workspace v2.5.1

## Release purpose

v2.5.1 is an organization and commissioning release. It separates the general laboratory toolbox from the real-time servo authority while preserving the validated v2.5 ST3215 command, feedback, calibration, and safety behavior.

The release has five boundaries:

1. `bhl_st3215_driver` is the sole normal owner of the ST3215 UART and the final hardware safety boundary.
2. `bhl_st3215_tools` owns guarded calibration, characterization, preflight, auditing, and dataset generation through ROS interfaces.
3. `bhl_st3215_maintenance` owns offline, read-only direct-bus inspection while the runtime driver is stopped.
4. `bhl_imu_tools` validates the canonical `/imu/data` contract independent of micro-ROS, I2C, or SPI transport.
5. `berkeley_biped_pkg` now supports a true policy shadow output mode that does not publish `/desired_position`.

## Scope lock

v2.5.1 does **not**:

- change the calibrated servo centers, signs, joint order, physical limits, or raw step limits;
- change the fixed ST3215 `speed=0`, `acceleration=0` baseline;
- add aggressive outer-PD behavior;
- integrate the unfinished Track 1 deployment bundle;
- change the full 0x38..0x46 feedback read used by the native driver;
- add EEPROM writes, ID changes, baud changes, or factory reset tools;
- add a reduced-register `runtime_fast` driver mode.

## Package versions

| Package | Version | Role |
|---|---:|---|
| `bhl_st3215_driver` | 0.2.8 | Bounded runtime UART/bus authority |
| `bhl_st3215_tools` | 0.1.0 | ROS-side laboratory and commissioning tools |
| `bhl_st3215_maintenance` | 0.1.0 | Offline read-only direct-bus maintenance |
| `bhl_imu_tools` | 0.1.0 | Source-independent IMU validation |
| `berkeley_biped_pkg` | 0.2.1 | Policy runtime with live/shadow/disabled output modes |

## Driver profiles

Profiles are launch-time YAML overlays. They do not enable writes and they do not change bus timing or register reads.

### Commissioning

```bash
ros2 launch bhl_st3215_driver bhl_st3215_driver.launch.py \
  profile:=commissioning \
  enable_writes:=false
```

Publishes joint state, feedback age, raw position, raw speed, cycle telemetry, diagnostics, and target debug.

### Runtime safe

```bash
ros2 launch bhl_st3215_driver bhl_st3215_driver.launch.py \
  profile:=runtime_safe \
  enable_writes:=false
```

Publishes joint state, feedback age, and diagnostics. Laboratory high-rate topics and target debug are disabled. The driver still reads the complete feedback block and retains health/error information in diagnostics.

`enable_writes` remains an explicit independent launch argument in both profiles.

## First commands after build

```bash
source ~/berkeley_ros2_ws/install/setup.bash

# Feedback-only commissioning driver
ros2 launch bhl_st3215_driver bhl_st3215_driver.launch.py \
  profile:=commissioning enable_writes:=false

# In another terminal
ros2 run bhl_st3215_tools st3215_preflight --mode feedback
ros2 run bhl_st3215_tools hardware_snapshot
ros2 run bhl_imu_tools imu_preflight
```

## Policy shadow

Use the dedicated policy-only launch. It does not launch the PD controller or servo driver:

```bash
ros2 launch berkeley_biped_pkg policy_shadow.launch.py
```

In shadow mode:

- `/desired_position` is not created by the policy node;
- proposed targets are published on `/policy_shadow/desired_position`;
- the exact live observation builder, ONNX session, action mapping, limit clipping, and previous-action semantics are used;
- policy readiness and debug topics remain active.

For initial hardware shadow work, run the ST3215 driver separately with `profile:=runtime_safe enable_writes:=false`.

## Exit-code contract

| Code | Meaning |
|---:|---|
| 0 | PASS |
| 2 | Test completed but acceptance criteria failed |
| 3 | Refused safety/precondition |
| 4 | Timeout or required ROS resource unavailable |
| 5 | Configuration error |
| 6 | Hardware or I/O error |
| 7 | Operator abort |
| 70 | Internal software error |
| 130 | Interrupted by SIGINT |

## Legacy package

`servo_test_pkg` now contains `COLCON_IGNORE`. Its old topic, limits, and unguarded test paths are not part of the v2.5.1 hardware workflow.
