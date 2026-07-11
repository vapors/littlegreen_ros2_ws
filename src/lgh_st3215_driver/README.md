# lgh_st3215_driver 0.3.0

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

`SerialPort` acquires an advisory `flock` before opening the device. For `/dev/ttyS3`, the lock path is:

```text
/tmp/lgh_st3215_dev_ttyS3.lock
```

A second driver or maintenance process refuses the open rather than competing for the UART. This is a process-ownership guard, not an electrical safety mechanism.

## Driver profiles

Profiles are YAML publication overlays selected at launch. They do not enable writes and do not change bus timing, register reads, joint mapping, or motion profile.

### Commissioning

```bash
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=commissioning \
  enable_writes:=false
```

Publishes:

- `/joint_states`
- `/joint_feedback_age_ms`
- `/st3215_driver/raw_position_steps`
- `/st3215_driver/raw_speed`
- `/st3215_driver/telemetry`
- `/st3215_driver/diagnostics`
- `/servo_target_steps_debug`

### Runtime safe

```bash
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=runtime_safe \
  enable_writes:=false
```

Publishes joint state, feedback age, and diagnostics. Raw/cycle telemetry and target debug are disabled. The driver still performs the full feedback read and exposes health/error information through diagnostics.

The default launch remains backward compatible and resolves to `commissioning` unless `profile` is supplied.

## Command input

`/servo_target_radians`

- type: `std_msgs/msg/Float64MultiArray`
- canonical 12-joint hardware/policy order;
- best-effort, keep-last 1 subscription;
- finite values required;
- final physical clamp always applied in the driver.

## State output

`/joint_states`

- type: `sensor_msgs/msg/JointState`;
- sensor-data QoS;
- compact default contract: `name=[]`, `position[12]`, `velocity[12]`, `effort=[]`.

`/joint_feedback_age_ms`

- type: `std_msgs/msg/UInt32MultiArray`;
- 12 physical-read ages in canonical order;
- `UINT32_MAX` means no valid sample.

Commissioning-only raw outputs:

```text
/st3215_driver/raw_position_steps   std_msgs/msg/Int32MultiArray
/st3215_driver/raw_speed            std_msgs/msg/Int32MultiArray
```

## Cycle-synchronous telemetry

`/st3215_driver/telemetry` uses `lgh_st3215_driver/msg/ServoTelemetry`. Each message is built from one completed bus cycle and includes:

- cycle/command/write/sample steady-clock timestamps;
- reference radians and quantized target steps;
- SyncWrite attempted/ok state;
- feedback sweep and cycle work durations;
- measured position and filtered velocity;
- raw position/speed;
- signed load, voltage, temperature, status, moving flag;
- raw current and decoded amperes;
- feedback ages and telemetry drop count.

When `publish_telemetry=false`, the bus worker does not enqueue telemetry snapshots and the publisher thread is not started.

## Diagnostics

`/st3215_driver/diagnostics` reports:

- active driver profile and publication flags;
- cycle rate and cycle-work mean/p99/max;
- feedback-sweep and read-RTT statistics;
- SyncWrite timing;
- protocol/I/O/error counters;
- feedback readiness and per-joint ages;
- command age/watchdog state;
- writes and torque state;
- current pose-override state;
- configured speed/acceleration profile;
- current raw feedback summary.

## Runtime services

```text
/st3215_driver/move_to_default_pose
/st3215_driver/abort_pose_move
/st3215_driver/hold_current_pose
/st3215_driver/release_pose_override
/st3215_driver/disable_torque_all
/st3215_driver/enable_torque_hold_current
```

All torque operations are serialized through the bus worker. `hold_current_pose` and `abort_pose_move` are software position holds, not hardware E-stops.

## Safety defaults

- writes disabled by default;
- complete feedback required before writes;
- startup command initialized from measured current position;
- stale command behavior holds the last safe target;
- fixed maximum-envelope ST3215 baseline remains `speed=0`, `acceleration=0` for every joint;
- final raw-step limits remain in `config/servo_map.yaml`.

## Guarded tooling

Current commands are installed by `lgh_st3215_tools`:

```bash
ros2 run lgh_st3215_tools st3215_preflight --mode feedback
ros2 run lgh_st3215_tools hardware_snapshot
ros2 run lgh_st3215_tools pose_console
ros2 run lgh_st3215_tools servo_identification --help
ros2 run lgh_st3215_tools standing_characterization --help
```

See `../lgh_st3215_tools/README.md` and `../../docs/MIGRATION_V2_5_1_TO_V2_6_0.md`.

## Offline maintenance

Stop the driver before:

```bash
ros2 run lgh_st3215_maintenance bus_scan --first-id 1 --last-id 12
ros2 run lgh_st3215_maintenance verify_ids
ros2 run lgh_st3215_maintenance register_dump --id 1 --address 0x00 --length 0x47
ros2 run lgh_st3215_maintenance backup_control_tables
```

v2.6.0 maintenance is read-only.
