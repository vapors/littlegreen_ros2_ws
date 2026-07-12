# Interfaces and Parameters

This page describes the active LittleGreen interfaces. Values shown are defaults from the current source tree.

## 1. ST3215 driver profiles

Profiles are launch-time YAML overlays. They control which ROS publications are enabled. They do **not** enable writes or change bus timing, register reads, joint mapping, or the configured ST3215 speed/acceleration values.

| Capability | `commissioning` | `runtime_safe` |
|---|:---:|:---:|
| `/joint_states` | Yes | Yes |
| `/joint_feedback_age_ms` | Yes | Yes |
| `/st3215_driver/diagnostics` | Yes | Yes |
| `/st3215_driver/raw_position_steps` | Yes | No |
| `/st3215_driver/raw_speed` | Yes | No |
| `/st3215_driver/telemetry` | Yes | No |
| `/servo_target_steps_debug` | Yes | No |
| `/st3215_feedback_debug` | No | No |

Launch examples:

```bash
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=commissioning \
  enable_writes:=false
```

```bash
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=runtime_safe \
  enable_writes:=false
```

`enable_writes` is independent of the selected profile.

## 2. `lgh_st3215_driver.launch.py`

| Argument | Default | Meaning |
|---|---|---|
| `profile` | `commissioning` | `commissioning` or `runtime_safe` |
| `config` | package `config/servo_driver.yaml` | Base driver parameter YAML |
| `servo_map` | package `config/servo_map.yaml` | Calibrated hardware map |
| `port` | `/dev/ttyS3` | UART device |
| `enable_writes` | `false` | Enables physical SyncWrite commands |
| `default_pose_move_duration_sec` | `4.0` | Guarded default-pose ramp duration |

Unsupported profile names cause launch to fail before the node starts.

## 3. ST3215 driver ROS interface

### Subscription

| Topic | Type | Purpose |
|---|---|---|
| `/servo_target_radians` | `std_msgs/msg/Float64MultiArray` | Canonical 12-joint command target |

The subscription exists in both profiles. Physical writes occur only when `writes_enabled=true` and the driver safety gates permit them.

### Publications

| Topic | Type | Profile |
|---|---|---|
| `/joint_states` | `sensor_msgs/msg/JointState` | both |
| `/joint_feedback_age_ms` | `std_msgs/msg/UInt32MultiArray` | both |
| `/st3215_driver/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | both |
| `/st3215_driver/raw_position_steps` | `std_msgs/msg/Int32MultiArray` | commissioning |
| `/st3215_driver/raw_speed` | `std_msgs/msg/Int32MultiArray` | commissioning |
| `/st3215_driver/telemetry` | `lgh_st3215_driver/msg/ServoTelemetry` | commissioning |
| `/servo_target_steps_debug` | `std_msgs/msg/String` | commissioning |
| `/st3215_feedback_debug` | `std_msgs/msg/String` | disabled by default |

The compact `/joint_states` contract is:

```text
name=[]
position[12]
velocity[12]
effort=[]
```

### Services

All services use `std_srvs/srv/Trigger`.

| Service | Meaning |
|---|---|
| `/st3215_driver/move_to_default_pose` | Ramp from measured pose to the configured default pose and assert the pose override |
| `/st3215_driver/abort_pose_move` | Stop the ramp and hold the best current measured pose |
| `/st3215_driver/hold_current_pose` | Latch the current measured pose and block external targets |
| `/st3215_driver/release_pose_override` | Return command authority to `/servo_target_radians` |
| `/st3215_driver/disable_torque_all` | Request torque-off for all servos |
| `/st3215_driver/enable_torque_hold_current` | Seed and hold current position, then enable torque |

Software position holds are not hardware E-stops.

## 4. ST3215 driver parameters

### Hardware and timing

| Parameter | Default |
|---|---|
| `port` | `/dev/ttyS3` |
| `baud` | `1000000` |
| `joint_map_path` | empty; launch supplies `servo_map.yaml` |
| `bus_rate_hz` | `50.0` |
| `command_rate_hz` | `50.0` |
| `joint_state_publish_hz` | `50.0` |
| `diagnostics_rate_hz` | `1.0` |
| `read_timeout_ms` | `10` |
| `write_timeout_ms` | `5` |
| `rotate_read_order` | `true` |
| `read_order_stride` | `1` |
| `worker_cpu` | `-1` |
| `realtime_priority` | `0` |

### Command and safety behavior

| Parameter | Default |
|---|---|
| `writes_enabled` | `false` |
| `require_full_feedback_before_writes` | `true` |
| `startup_hold_current_position` | `true` |
| `command_timeout_ms` | `500` |
| `command_timeout_behavior` | `hold_last` |
| `skip_unchanged_writes` | `false` |
| `write_keepalive_ms` | `200` |
| `default_speed` | `0` |
| `default_acceleration` | `0` |
| `default_pose_move_duration_sec` | `4.0` |
| `default_pose_ramp_rate_hz` | `50.0` |
| `default_pose_hold_after_move` | `true` |

### Feedback and diagnostics

| Parameter | Default |
|---|---|
| `velocity_filter_alpha` | `0.30` |
| `velocity_deadband_rad_s` | `0.001` |
| `compact_joint_state` | `true` |
| `frame_id` | `st3215_bus` |
| `max_feedback_warn_age_ms` | `250` |
| `diagnostic_window_cycles` | `500` |

### Publication flags

| Parameter | `commissioning` | `runtime_safe` |
|---|:---:|:---:|
| `publish_joint_states` | `true` | `true` |
| `publish_feedback_age` | `true` | `true` |
| `publish_diagnostics` | `true` | `true` |
| `publish_raw_position` | `true` | `false` |
| `publish_raw_speed` | `true` | `false` |
| `publish_telemetry` | `true` | `false` |
| `publish_target_debug_string` | `true` | `false` |
| `publish_legacy_debug_string` | `false` | `false` |

## 5. ST3215 preflight

### Modes

| Mode | Expected profile | Writes expectation | Additional checks |
|---|---|---|---|
| `feedback` | any | forced `false` | required streams, feedback, timing, counters |
| `commissioning` | `commissioning` | `auto`, `true`, or `false` | laboratory topics present; policy node absent |
| `runtime` | `runtime_safe` | `auto`, `true`, or `false` | laboratory topics absent |

Examples:

```bash
ros2 run lgh_st3215_tools st3215_preflight \
  --mode feedback \
  --expect-writes false
```

```bash
ros2 run lgh_st3215_tools st3215_preflight \
  --mode commissioning \
  --expect-writes false
```

```bash
ros2 run lgh_st3215_tools st3215_preflight \
  --mode runtime \
  --expect-writes false
```

Key options:

```text
--sample-sec 3.0
--timeout-sec 8.0
--max-feedback-age-ms 100
--min-cycle-rate-hz 45.0
--max-cycle-work-p99-us 15000
--output-root PATH
```

## 6. Tool exit codes

| Code | Meaning |
|---:|---|
| `0` | pass |
| `2` | test completed but acceptance criteria failed |
| `3` | refused safety/precondition |
| `4` | timeout or ROS resource unavailable |
| `5` | configuration error |
| `6` | hardware or I/O error |
| `7` | operator abort |
| `70` | internal software error |
| `130` | interrupted by `SIGINT` |

## 7. Policy launches

The policy launch files do not start the ST3215 driver or IMU source. Those hardware authorities remain separately managed.

### `policy_shadow.launch.py`

Starts only `littlegreen_biped_node` with `policy_output_mode=shadow`.

| Argument | Default |
|---|---|
| `policy_config` | packaged `policy_latest.yaml` |
| `policy_runtime_config` | packaged `policy_runtime.yaml` |
| `joint_map` | packaged `joint_map.yaml` |
| `onnx_model_path` | empty |
| `use_sim` | `false` |
| `override_imu` | `false` |
| `shadow_desired_position_topic` | `/policy_shadow/desired_position` |

### `policy_live.launch.py`

Starts `littlegreen_biped_node` in live mode and starts `pd_controller_node`. It does not start teleop.

| Argument | Default | Meaning |
|---|---|---|
| `policy_config` | packaged `policy_latest.yaml` | Policy YAML paired with the ONNX model |
| `policy_runtime_config` | packaged `policy_runtime.yaml` | Freshness gates and IMU transform |
| `joint_map` | packaged `joint_map.yaml` | Canonical joint defaults and physical bounds |
| `onnx_model_path` | empty | Explicit model override |
| `pd_config` | packaged `pd_config.yaml` | Downstream safety and shaping config |
| `controller_mode` | `safety_only` | Initial live deployment must use `safety_only` |
| `use_sim` | `false` | Simulation QoS/data behavior |
| `override_imu` | `false` | Nominal IMU override; not recommended on live hardware |

### `littlegreen_biped_launch.py`

Starts joystick input, teleop, the policy node, the command-file bridge, and `pd_controller_node`.

| Argument | Default | Meaning |
|---|---|---|
| `policy_config` | packaged `policy_latest.yaml` | Policy metadata paired with the ONNX model |
| `policy_runtime_config` | packaged `policy_runtime.yaml` | Freshness gates and IMU transform |
| `onnx_model_path` | empty | Explicit model override |
| `joint_map` | packaged `joint_map.yaml` | Canonical joint order, defaults, and limits |
| `pd_config` | packaged `pd_config.yaml` | Downstream controller configuration |
| `controller_mode` | `safety_only` | `safety_only`, `outer_pd`, or `outer_pid` |
| `teleop_config` | packaged `shanwan.config.yaml` | Joystick mapping |
| `use_sim` | `false` | Simulation QoS/data behavior |
| `override_imu` | `false` | Inject nominal IMU values instead of `/imu/data` |
| `policy_output_mode` | `live` | `live`, `shadow`, or `disabled` |

### `biped_teleop_mux.launch.py`

Starts joystick and keyboard teleop, `twist_mux`, the policy node, the command-file bridge, and `pd_controller_node`. Keyboard teleop is opened with `xterm`.

Use the dedicated shadow and live launch files for first hardware deployment. See [`LIVE_POLICY_DEPLOYMENT.md`](LIVE_POLICY_DEPLOYMENT.md).

### Action contract v3

When `action_contract_version: 3` is present, `littlegreen_biped_node` requires the bounded default-centered symmetric residual transform and loads `action_residual_scale_rad` directly.

The node validates the following exported fields against `joint_map.yaml` before ONNX inference starts:

```text
action_indices
action_default_rad
action_target_lower_rad
action_target_upper_rad
joints[action_indices]
default_joint_positions[action_indices]
```

It also requires normalized action limits `[-1, 1]`, positive residual scales, and:

```text
previous_action_observation: bounded_normalized_action
```

Legacy YAML without `action_contract_version` remains readable through the older `action_scale` field, but current hardware deployment should use a paired v3 bundle.

## 8. Policy output modes

| Mode | Policy output |
|---|---|
| `live` | publishes `/desired_position` |
| `shadow` | publishes `/policy_shadow/desired_position`; does not create a policy publisher on `/desired_position` |
| `disabled` | validates and caches inputs without publishing a target |

Common policy inputs:

```text
/imu/data
/joint_states
/joint_feedback_age_ms
/command_velocity
/joy
```

Common policy status/debug outputs:

```text
/policy_ready
/policy_status
/policy_debug/observation
/policy_debug/raw_action
/policy_debug/clipped_raw_action
/policy_debug/target_unclipped
/policy_debug/target_clipped
/policy_debug/saturation_mask
```

## 9. Policy runtime parameters

| Parameter | Default |
|---|---|
| `use_sim` | `false` |
| `override_imu` | `false` |
| `publish_policy_debug` | `true` |
| `policy_output_mode` | `live` |
| `shadow_desired_position_topic` | `/policy_shadow/desired_position` |
| `imu_timeout_sec` | `0.050` |
| `joint_state_timeout_sec` | `0.150` |
| `require_joint_feedback_age` | `true` |
| `joint_feedback_age_topic_timeout_sec` | `0.150` |
| `joint_feedback_max_age_sec` | `0.250` |
| `command_timeout_sec` | `0.500` |
| `zero_command_on_timeout` | `true` |
| `require_joint_velocity` | `true` |
| `imu_to_base_matrix` | `[0, 1, 0, -1, 0, 0, 0, 0, 1]` |

## 10. PD controller

The default commissioning-compatible mode is:

```text
controller_mode=safety_only
```

Available modes:

```text
safety_only
outer_pd
outer_pid
```

The current workflow does not authorize aggressive outer-loop tuning. See the package configuration for the complete gain and limit arrays:

```text
src/pd_controller_pkg/config/pd_config.yaml
```

Main interface:

| Direction | Topic/service |
|---|---|
| input | `/desired_position` |
| input | `/desired_joint_position` |
| input | `/joint_states` |
| output | `/servo_target_radians` |
| output | `/safe_joint_targets` |
| service | `/pd_controller/reset_to_feedback` |

## 11. IMU tools

The IMU tools validate the ROS boundary and are independent of whether `/imu/data` comes from micro-ROS, direct I2C, or direct SPI.

```bash
ros2 run lgh_imu_tools imu_preflight --help
ros2 run lgh_imu_tools stationary_characterization --help
ros2 run lgh_imu_tools orientation_audit --help
ros2 run lgh_imu_tools imu_recorder --help
```

The canonical contract is:

```text
src/lgh_imu_tools/config/imu_contract.yaml
```

## 12. Laboratory tools

Installed executables:

```text
pose_console
print_default_pose
capture_calibration
apply_calibration
verify_calibration
servo_identification
standing_characterization
st3215_preflight
hardware_snapshot
dataset_manifest
```

Identification modes:

```text
step
step_sweep
deadband_staircase
triangle
hold_under_load
```

Use each executable's `--help` output as the authoritative CLI reference:

```bash
ros2 run lgh_st3215_tools servo_identification --help
ros2 run lgh_st3215_tools standing_characterization --help
```

## 13. Offline maintenance

Installed executables:

```text
bus_scan
verify_ids
register_dump
backup_control_tables
```

Maintenance opens `/dev/ttyS3` directly and refuses to start if the runtime driver owns the shared lock. The current package is read-only.
