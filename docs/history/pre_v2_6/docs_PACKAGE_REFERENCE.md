# Package Reference — berkeley_ros2_ws v2.5.1

## `bhl_st3215_driver` 0.2.8 — runtime hardware authority

The only normal owner of `/dev/ttyS3`. It owns UART configuration, ST3215 packet transactions, the 50 Hz worker, command watchdog, canonical radian-to-step conversion, final radian/raw-step clamping, feedback state, diagnostics, torque services, and software pose-hold services.

```bash
ros2 launch bhl_st3215_driver bhl_st3215_driver.launch.py \
  profile:=commissioning enable_writes:=false
```

Profiles:

- `commissioning`: joint state, feedback age, raw position, raw speed, cycle telemetry, diagnostics, target debug.
- `runtime_safe`: joint state, feedback age, diagnostics; laboratory high-rate publications disabled.

Profiles do not change bus timing, full feedback reads, or write authorization.

Authoritative configuration:

- `config/servo_driver.yaml`
- `config/servo_map.yaml`
- `config/profiles/commissioning.yaml`
- `config/profiles/runtime_safe.yaml`

## `bhl_st3215_tools` 0.1.0 — laboratory and commissioning tools

Uses ROS topics/services and never opens the UART.

```text
st3215_preflight
hardware_snapshot
print_default_pose
capture_calibration
apply_calibration
verify_calibration
pose_console
servo_identification
standing_characterization
dataset_manifest
```

Tool configuration:

- `config/standing_pose_library.yaml`
- `config/track1_action_contract_v3.yaml`

Preflight modes are deliberately small and explicit: `feedback`, `commissioning`, and `runtime`.

## `bhl_st3215_maintenance` 0.1.0 — offline direct-bus inspection

Read-only v2.5.1 commands:

```text
bus_scan
verify_ids
register_dump
backup_control_tables
```

The runtime driver must be stopped. The shared `SerialPort` lock refuses competing ownership of the same UART. There are no ID, baud, EEPROM, or factory-reset write commands in this release.

## `bhl_imu_tools` 0.1.0 — canonical IMU contract validation

Works at the `sensor_msgs/msg/Imu` boundary and is independent of whether the source is micro-ROS, direct I2C, or direct SPI.

```text
imu_preflight
stationary_characterization
orientation_audit
imu_recorder
```

The contract is `config/imu_contract.yaml`.

## `berkeley_biped_pkg` 0.2.1 — policy inference

Loads the paired YAML/ONNX policy, builds the 45-element observation, applies the configured IMU-to-base transform, checks sensor freshness and finite values, maps/clips 12 actions, and publishes policy status/debug information.

Output modes:

- `live`: publishes `/desired_position`.
- `shadow`: publishes only `/policy_shadow/desired_position` while preserving live observation/inference/action semantics.
- `disabled`: evaluates readiness but does not run inference or publish targets.

Dedicated policy-only shadow launch:

```bash
ros2 launch berkeley_biped_pkg policy_shadow.launch.py
```

Track 1 deployment-bundle integration is intentionally deferred.

## `pd_controller_pkg` — downstream command shaping

Retained unchanged for v2.5.1. Initial policy work remains `safety_only`; aggressive outer-PD integration is not part of this release.

## `joystick_bridge`

Mirrors `/command_velocity` to `/tmp/joystick_cmd.txt`. It is not part of servo-bus safety authority.

## `servo_test_pkg`

Legacy and excluded by `COLCON_IGNORE`. Do not use it for v2.5.1 hardware work.

## `lilgreen_description`

Visualization/description package. URDF limits are not the runtime hardware authority; runtime limits remain in the canonical policy joint map and final driver servo map.
