# Source audit and migration notes

This package was drafted from the supplied:

- `littlegreen_ros2_ws_src.zip`
- `lgh_st3215_microros_pio_v6_5_8_compact_joint_state.zip`
- `lgh_icm20948_microros_pio_v_1_0_1.zip`

## Existing ROS interface found in the workspace

The current policy node subscribes to:

- `/joint_states` as `sensor_msgs/msg/JointState`
- `/joint_feedback_age_ms` as `std_msgs/msg/UInt32MultiArray`

for the physical path using best-effort sensor QoS.

The policy node still subscribes to the legacy split topics `/joint_states_position` and `/joint_states_velocity`, but comments in the current node explicitly mark them as backward compatibility only. The v6.5.8 servo firmware publishes the compact canonical `/joint_states` path, so this native package does the same.

The current PD/safety node publishes:

- `/servo_target_radians` as `std_msgs/msg/Float64MultiArray`
- reliable keep-last 1 QoS
- 12 radians in canonical joint order

The supplied micro-ROS v6.5.8 firmware requests BEST_EFFORT for this subscription. The native driver preserves that requested QoS (keep-last 1); the current RELIABLE PD publisher is compatible with the BEST_EFFORT subscriber.

## Compact JointState compatibility

The v6.5.8 firmware publishes:

```text
name=[]
position[12]
velocity[12]
effort=[]
```

The current policy and PD nodes both explicitly accept unnamed 12-value `JointState` messages in canonical hardware order. The native package keeps compact mode enabled by default.

## Feedback ages

The existing policy freshness gate expects 12 age values. It rejects missing/invalid ages and adds message receipt latency to the reported hardware age when evaluating freshness.

The native package computes each age from the actual per-joint steady-clock timestamp taken after a successful physical UART reply. Failed reads leave the last successful timestamp untouched so age increases naturally.

## Command timeout behavior

The native first pass defaults to:

```text
command_timeout_ms: 500
command_timeout_behavior: hold_last
```

A stale command is never extrapolated. The last safe step targets remain the hold target. The servo feedback loop and diagnostics continue running.

## Read-order rotation

Read-order rotation changes only bus polling sequence, not ROS array order. All state is stored by canonical joint index before publication.

## IMU source

No IMU code is included in this package. The supplied ICM-20948 micro-ROS source was reviewed only to preserve the current system boundary. The servo driver remains independent of `/imu/data`; the current IMU ESP32 path can continue unchanged while the native servo driver is commissioned.
