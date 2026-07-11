# Interfaces, Switches, and Parameters — v2.6.0

## A. Primary launch switches

### `lgh_st3215_driver.launch.py`

| Argument | Default | Meaning |
|---|---:|---|
| `config` | package `config/servo_driver.yaml` | Driver ROS parameter YAML |
| `servo_map` | package `config/servo_map.yaml` | Native hardware map |
| `port` | `/dev/ttyS3` | UART device |
| `enable_writes` | `false` | Enables physical SyncWrite commands |
| `default_pose_move_duration_sec` | `4.0` | Guarded training-default ramp duration |

### `littlegreen_biped_launch.py`

| Argument | Default | Meaning |
|---|---|---|
| `policy_config` | `policy_latest.yaml` | Paired deployment policy YAML |
| `policy_runtime_config` | `policy_runtime.yaml` | Runtime freshness and IMU transform |
| `onnx_model_path` | empty | Explicit ONNX override; otherwise paired artifact resolution |
| `joint_map` | `joint_map.yaml` | Canonical policy/controller limit map |
| `pd_config` | `pd_config.yaml` | Downstream shaper/outer-loop config |
| `controller_mode` | `safety_only` | `safety_only`, `outer_pd`, `outer_pid` |
| `teleop_config` | `shanwan.config.yaml` | Joystick mapping |
| `use_sim` | `false` | Simulation QoS/data-layout behavior |
| `override_imu` | `false` | Use zero angular velocity and nominal gravity instead of `/imu/data` |

## B. Native ST3215 driver parameters

| Parameter | v2.6.0 value |
|---|---|
| `port` | `/dev/ttyS3` |
| `baud` | `1000000` |
| `joint_map_path` | `` |
| `bus_rate_hz` | `50.0` |
| `command_rate_hz` | `50.0` |
| `joint_state_publish_hz` | `50.0` |
| `diagnostics_rate_hz` | `1.0` |
| `read_timeout_ms` | `10` |
| `write_timeout_ms` | `5` |
| `command_timeout_ms` | `500` |
| `command_timeout_behavior` | `hold_last` |
| `writes_enabled` | `False` |
| `require_full_feedback_before_writes` | `True` |
| `startup_hold_current_position` | `True` |
| `skip_unchanged_writes` | `False` |
| `write_keepalive_ms` | `200` |
| `rotate_read_order` | `True` |
| `read_order_stride` | `1` |
| `velocity_filter_alpha` | `0.3` |
| `velocity_deadband_rad_s` | `0.001` |
| `default_speed` | `0` |
| `default_acceleration` | `0` |
| `compact_joint_state` | `True` |
| `frame_id` | `st3215_bus` |
| `max_feedback_warn_age_ms` | `250` |
| `diagnostic_window_cycles` | `500` |
| `publish_legacy_debug_string` | `False` |
| `publish_target_debug_string` | `True` |
| `worker_cpu` | `-1` |
| `realtime_priority` | `0` |
| `servo_target_topic` | `/servo_target_radians` |
| `joint_state_topic` | `/joint_states` |
| `feedback_age_topic` | `/joint_feedback_age_ms` |
| `raw_position_topic` | `/st3215_driver/raw_position_steps` |
| `raw_speed_topic` | `/st3215_driver/raw_speed` |
| `telemetry_topic` | `/st3215_driver/telemetry` |
| `diagnostics_topic` | `/st3215_driver/diagnostics` |
| `legacy_debug_topic` | `/st3215_feedback_debug` |
| `target_debug_topic` | `/servo_target_steps_debug` |
| `default_pose_move_duration_sec` | `4.0` |
| `default_pose_ramp_rate_hz` | `50.0` |
| `default_pose_hold_after_move` | `True` |
| `move_default_pose_service` | `/st3215_driver/move_to_default_pose` |
| `abort_pose_move_service` | `/st3215_driver/abort_pose_move` |
| `hold_current_pose_service` | `/st3215_driver/hold_current_pose` |
| `release_pose_override_service` | `/st3215_driver/release_pose_override` |
| `disable_torque_all_service` | `/st3215_driver/disable_torque_all` |
| `enable_torque_hold_current_service` | `/st3215_driver/enable_torque_hold_current` |

### Driver topic/service summary

See `topic_service_matrix.csv` for the machine-readable table. Operationally important endpoints:

```text
SUB  /servo_target_radians
PUB  /joint_states
PUB  /joint_feedback_age_ms
PUB  /st3215_driver/telemetry
PUB  /st3215_driver/diagnostics
SRV  /st3215_driver/move_to_default_pose
SRV  /st3215_driver/abort_pose_move
SRV  /st3215_driver/hold_current_pose
SRV  /st3215_driver/release_pose_override
SRV  /st3215_driver/disable_torque_all
SRV  /st3215_driver/enable_torque_hold_current
```

### Service semantics

- `move_to_default_pose`: requires writes enabled and fresh complete feedback; ramps from measured pose; asserts pose override.
- `abort_pose_move`: stops the ramp and holds the best current measured pose; override remains active.
- `hold_current_pose`: latches measured pose if fresh; otherwise blocks new external targets while holding last safe command. This is not a hardware E-stop.
- `release_pose_override`: allows `/servo_target_radians` to control the bus again; align the PD controller to feedback first.
- `disable_torque_all`: explicit torque-off request; robot must be mechanically supported.
- `enable_torque_hold_current`: seeds current measured pose, writes it while torque is off, enables torque, and leaves override active.

## C. Policy node runtime parameters

| Parameter | v2.6.0 value |
|---|---|
| `publish_policy_debug` | `True` |
| `imu_timeout_sec` | `0.05` |
| `joint_state_timeout_sec` | `0.15` |
| `require_joint_feedback_age` | `True` |
| `joint_feedback_age_topic_timeout_sec` | `0.15` |
| `joint_feedback_max_age_sec` | `0.25` |
| `command_timeout_sec` | `0.5` |
| `zero_command_on_timeout` | `True` |
| `require_joint_velocity` | `True` |
| `imu_to_base_matrix` | `[0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0]` |

Additional node parameters declared in code:

| Parameter | Default | Meaning |
|---|---|---|
| `use_sim` | `false` | Simulation QoS/data behavior |
| `override_imu` | `false` | Inject nominal IMU state instead of subscribing |
| `policy_config_path` | packaged `policy_latest.yaml` | Deployment metadata and action contract |
| `joint_map_path` | packaged `joint_map.yaml` | Canonical 12-joint map/limits |
| `onnx_model_path` | empty | Explicit model override |
| `publish_policy_debug` | `true` | Enable six policy-debug topic streams |

### Current policy YAML contract

| Field | Current value |
|---|---|
| `num_observations` | `45` |
| `num_actions` | `12` |
| `policy_dt` | `0.04` s |
| `control_dt` | `0.04` s |
| `physics_dt` | `0.005` s |
| `action_scale` | `0.25` |
| `action_indices` | `[0, 1, 2, 3, 4, 5, 7, 8, 9, 10, 11, 12]` |
| `policy_checkpoint_relative_path` | `policy.onnx` |

## D. PD controller parameters

| Parameter | v2.6.0 value |
|---|---|
| `joint_map_path` | `` |
| `controller_mode` | `safety_only` |
| `control_rate_hz` | `50.0` |
| `command_timeout_sec` | `0.5` |
| `feedback_timeout_sec` | `0.15` |
| `require_feedback_for_outer_loop` | `True` |
| `initialize_output_from_feedback` | `True` |
| `startup_publish_default_pose` | `False` |
| `desired_position_topic` | `/desired_position` |
| `desired_joint_position_topic` | `/desired_joint_position` |
| `canonical_joint_state_topic` | `/joint_states` |
| `legacy_position_topic` | `/joint_states_position` |
| `legacy_velocity_topic` | `/joint_states_velocity` |
| `servo_target_topic` | `/servo_target_radians` |
| `safe_target_joint_state_topic` | `/safe_joint_targets` |
| `pd_torque_debug_topic` | `/pd_torque_debug` |
| `outer_velocity_debug_topic` | `/outer_controller/velocity_command` |
| `outer_error_debug_topic` | `/outer_controller/position_error` |
| `outer_integral_debug_topic` | `/outer_controller/integral_error` |
| `controller_status_topic` | `/outer_controller/status` |
| `reset_to_feedback_service` | `/pd_controller/reset_to_feedback` |
| `publish_outer_loop_debug` | `True` |
| `command_filter_alpha` | `0.7` |
| `enable_rate_limit` | `True` |
| `enable_accel_limit` | `True` |
| `max_joint_speed_rad_s` | `[8.0, 8.0, 8.0, 8.0, 8.0, 8.0, 8.0, 8.0, 8.0, 8.0, 8.0, 8.0]` |
| `max_joint_accel_rad_s2` | `[80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0, 80.0]` |
| `kp` | `[2.5, 2.0, 3.0, 3.5, 3.0, 2.5, 2.5, 2.0, 3.0, 3.5, 3.0, 2.5]` |
| `kd` | `[0.1, 0.08, 0.12, 0.15, 0.12, 0.1, 0.1, 0.08, 0.12, 0.15, 0.12, 0.1]` |
| `ki` | `[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]` |
| `max_controller_velocity_rad_s` | `[1.5, 1.2, 2.0, 2.0, 1.8, 1.5, 1.5, 1.2, 2.0, 2.0, 1.8, 1.5]` |
| `max_controller_accel_rad_s2` | `[12.0, 10.0, 15.0, 15.0, 12.0, 10.0, 12.0, 10.0, 15.0, 15.0, 12.0, 10.0]` |
| `integral_error_limit_rad_sec` | `[0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 0.25]` |
| `publish_pd_torque_debug` | `False` |
| `torque_limit` | `1.5` |

## E. Calibration tool CLI

### `lgh_st3215_tools capture_calibration`

```text
--servo-map PATH
--raw-topic /st3215_driver/raw_position_steps
--age-topic /joint_feedback_age_ms
--diagnostics-topic /st3215_driver/diagnostics
--samples 250
--capture-timeout-sec 20
--preflight-timeout-sec 5
--max-feedback-age-ms 50
--fine-threshold-steps 25
--inspect-threshold-steps 100
--max-sample-span-steps 8
--output-dir calibration_reports
--allow-writes-enabled
--yes
```

### `lgh_st3215_tools apply_calibration`

```text
proposal                       positional proposal YAML
--source-servo-map PATH       explicit source-tree servo_map.yaml
--apply                        actually modify the source servo_map.yaml
--allow-map-mismatch
--allow-large-corrections
```

### `lgh_st3215_tools verify_calibration`

```text
--servo-map PATH
--joint-topic /joint_states
--age-topic /joint_feedback_age_ms
--diagnostics-topic /st3215_driver/diagnostics
--samples 250
--max-feedback-age-ms 50
--pass-tolerance-rad 0.02
--warn-tolerance-rad 0.05
--timeout-sec 20
--allow-writes-enabled
--output-dir calibration_reports
```

## F. Servo identification runner CLI

Modes:

```text
step
step_sweep
deadband_staircase
triangle
hold_under_load
```

Core switches:

```text
--joint NAME_OR_INDEX                 required
--mode MODE                           required
--command-path direct|outer           default direct
--servo-map PATH
--output-dir identification_reports
--support-condition securely_supported
--direction positive|negative|both    default both
--amplitude-rad 0.02
--amplitudes-rad 0.02,0.05,0.1
--test-center-offset-rad 0.0
--test-center-move-sec 2.0
--test-center-settle-sec 1.5
--max-test-offset-rad 0.20
--joint-limit-margin-rad 0.01
--motion-threshold-rad 0.002
--velocity-plateau-fraction 0.10
--baseline-sec 1.5
--step-hold-sec 2.5
--between-trials-sec 1.0
--return-sec 2.0
--final-hold-sec 1.0
--deadband-offsets-rad 0.002,0.005,0.01,0.02
--deadband-dwell-sec 1.5
--triangle-amplitude-rad 0.02
--triangle-frequency-hz 0.10
--triangle-cycles 2
--load-baseline-sec 3
--load-prepare-sec 5
--load-hold-sec 10
--load-offset-rad 0
--load-force-n VALUE
--load-mass-kg VALUE
--lever-arm-m VALUE
--stiffness-min-deflection-rad 0.0015
--max-feedback-age-ms 100
--joint-state-timeout-sec 0.15
--preflight-timeout-sec 8
--countdown-sec 3
--direct-topic /servo_target_radians
--outer-reference-topic /desired_position
--allow-nonmax-motion-profile
--allow-all-2048-centers
```

Output directory contains at least:

```text
timeseries.csv
metadata.yaml
summary.yaml
summary.txt
```

## G. Standing-load characterization CLI

Modes:

```text
capture_pose
evaluate
```

Shared/default paths:

```text
pose library:       ~/.ros/lgh_standing_poses.yaml
capture audit root: ~/.ros/lgh_standing_pose_capture_audits
evaluation root:    ~/littlegreen_ros2_ws/track2_standing_reports
```

Capture switches:

```text
--pose-name NAME
--base-com-height-mean-m VALUE
--capture-window-sec 2.0
--capture-min-samples 60
--capture-max-q-std-rad 0.01
--capture-audit-root PATH
--reenable-torque-hold-after-capture
```

Evaluation switches:

```text
--poses normal_stand,shallow_crouch,medium_crouch,deep_crouch
--target-base-com-height-m VALUE
--height-match-tolerance-m 0.015
--return-between-poses / --no-return-between-poses
--crouch-speed-rad-s 0.20
--stand-return-speed-rad-s 0.15
--transition-speed-rad-s 0.15
--min-transition-sec 1.0
--command-rate-hz 50
--settle-sec 5
--hold-sec 20
--deep-pose-name deep_crouch
--deep-hold-sec 8
--repeats 1
--output-root PATH
--preflight-timeout-sec 10
--max-feedback-age-ms 250
--joint-state-timeout-sec 0.5
--joint-limit-margin-rad 0.01
--max-current-a 1.5         (0 disables)
--max-load-ratio 0.9        (0 disables)
--min-voltage-v 9.0         (0 disables)
--max-temp-c 60.0           (0 disables)
--guard-consecutive-cycles 5
```

Evaluation artifacts:

```text
timeseries.csv
pose_joint_summary.csv
pose_level_summary.csv
bilateral_pose_summary.csv
transition_joint_summary.csv
metadata.yaml
summary.txt
```

## H. Diagnostic keys emitted by the native driver

```text
cycle_rate_hz
cycle_work_us_mean / p99 / max
feedback_sweep_us_mean / p99 / max
sync_write_call_us_mean / max
read_rtt_us_mean / p99 / max
cycle_count
sync_write_count / sync_write_error_count
read_success_count / read_timeout_count
checksum_error_count
malformed_frame_count
wrong_id_count
io_error_count
servo_status_error_count
deadline_miss_count
cycles_over_period_count
command_rx_count / command_reject_count
command_age_ms
feedback_ready
writes_enabled
motion_profile
configured_speed_steps_s
configured_acceleration_units
pose_override_active
pose_move_running
pose_abort_count
hold_pose_latch_count
command_ignored_pose_override_count
telemetry_dropped_count
torque_enabled_state
max_joint_age_ms
per_joint_age_ms
last_read_ok
per_joint_read_ok_count
per_joint_read_fail_count
raw_position_steps
raw_speed
last_error
```

---

# v2.6.0 additions

This section supersedes older tool-location references above.

## ST3215 driver profile launch argument

```text
profile = commissioning | runtime_safe
```

`enable_writes` remains independent and defaults false.

New driver parameters:

```text
driver_profile
publish_joint_states
publish_feedback_age
publish_raw_position
publish_raw_speed
publish_telemetry
publish_diagnostics
```

## ST3215 tool executables

```text
lgh_st3215_tools/st3215_preflight
lgh_st3215_tools/hardware_snapshot
lgh_st3215_tools/print_default_pose
lgh_st3215_tools/capture_calibration
lgh_st3215_tools/apply_calibration
lgh_st3215_tools/verify_calibration
lgh_st3215_tools/pose_console
lgh_st3215_tools/servo_identification
lgh_st3215_tools/standing_characterization
lgh_st3215_tools/dataset_manifest
```

## Offline maintenance executables

```text
lgh_st3215_maintenance/bus_scan
lgh_st3215_maintenance/verify_ids
lgh_st3215_maintenance/register_dump
lgh_st3215_maintenance/backup_control_tables
```

## IMU tool executables

```text
lgh_imu_tools/imu_preflight
lgh_imu_tools/stationary_characterization
lgh_imu_tools/orientation_audit
lgh_imu_tools/imu_recorder
```

## Policy output parameters

```text
policy_output_mode = live | shadow | disabled
shadow_desired_position_topic = /policy_shadow/desired_position
```
