# lgh_st3215_driver

Native ROS 2 Humble single-bus ST3215 runtime driver for LittleGreen on the Orange Pi 5 Max.

## Package boundary

This package is the sole normal hardware authority for the ST3215 bus. It owns:

- `/dev/ttyS3` at 1,000,000 baud by default;
- the one-thread 50 Hz bus worker;
- 12-servo `SyncWritePosEx` command transactions;
- one contiguous `0x38..0x46` feedback read per servo;
- rotating first-read order;
- canonical radian-to-step conversion from `servo_map.yaml`;
- final radian and raw-step clamping;
- command watchdog and startup hold;
- feedback timestamps, ages, diagnostics, and guarded runtime services.

Calibration, characterization, preflight, and dataset generation live in `lgh_st3215_tools`. Offline direct-bus inspection lives in `lgh_st3215_maintenance` and requires this driver to be stopped.

## UART ownership

The serial implementation acquires an advisory `flock` before opening the device. For `/dev/ttyS3`, the default lock is:

```text
/tmp/lgh_st3215_dev_ttyS3.lock
```

A second driver or maintenance process refuses to open the device. This is a process-ownership guard, not an electrical safety mechanism.

## Driver profiles

Profiles are YAML publication overlays. They do not enable writes and do not change bus timing, register reads, joint mapping, or the ST3215 motion profile.

### Commissioning

```bash
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=commissioning \
  enable_writes:=false
```

Publishes joint state, feedback age, diagnostics, raw position, raw speed, cycle telemetry, and target-step debug.

### Runtime safe

```bash
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=runtime_safe \
  enable_writes:=false
```

Publishes joint state, feedback age, and diagnostics. Laboratory high-rate publications are disabled, while the full hardware feedback read remains active internally.

## Command input

`/servo_target_radians`

- type: `std_msgs/msg/Float64MultiArray`;
- canonical 12-joint order;
- finite values required;
- final physical clamp always applied by the driver.

## State output

`/joint_states`

- type: `sensor_msgs/msg/JointState`;
- compact default contract: `name=[]`, `position[12]`, `velocity[12]`, `effort=[]`.

`/joint_feedback_age_ms`

- type: `std_msgs/msg/UInt32MultiArray`;
- 12 physical-read ages in canonical order;
- `UINT32_MAX` means no valid sample.

Commissioning raw outputs:

```text
/st3215_driver/raw_position_steps
/st3215_driver/raw_speed
/st3215_driver/telemetry
```

## Diagnostics

`/st3215_driver/diagnostics` includes the active profile, publication flags, cycle timing, protocol/error counters, feedback readiness and ages, command/watchdog state, write state, torque state, pose-override state, and hardware feedback summary.

## Runtime services

```text
/st3215_driver/move_to_default_pose
/st3215_driver/abort_pose_move
/st3215_driver/hold_current_pose
/st3215_driver/release_pose_override
/st3215_driver/disable_torque_all
/st3215_driver/enable_torque_hold_current
```

All torque operations are serialized through the bus worker. Software holds are not hardware E-stops.

## Safety defaults

- writes disabled;
- full feedback required before writes;
- startup target initialized from measured position;
- stale command behavior holds the last safe target;
- fixed ST3215 baseline `speed=0`, `acceleration=0`;
- final limits in `config/servo_map.yaml`.

See:

- [`../../docs/INTERFACES_AND_PARAMETERS.md`](../../docs/INTERFACES_AND_PARAMETERS.md)
- [`../../docs/FRESH_INSTALL_CHECKLIST.md`](../../docs/FRESH_INSTALL_CHECKLIST.md)
- [`../lgh_st3215_tools/README.md`](../lgh_st3215_tools/README.md)

## Command authority

A write-enabled driver accepts `/servo_target_radians` whenever no internal pose override is active. Before calibration or before releasing an override, inspect:

```bash
ros2 topic info /servo_target_radians --verbose
```

See [`../../docs/ROS_GRAPH_AND_AUTHORITY.md`](../../docs/ROS_GRAPH_AND_AUTHORITY.md) and [`../../docs/COMMAND_REFERENCE.md`](../../docs/COMMAND_REFERENCE.md).
