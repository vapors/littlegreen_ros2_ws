# LittleGreen Command and Option Reference

This page exposes the available first-party commands, launch arguments, service behavior, and frequently useful ROS inspection commands in v2.8.0. It is intentionally more detailed than the command cheat sheet.

## 1. How to discover options from the installed workspace

The installed workspace is the authority for the commands available on a specific machine.

```bash
ros2 pkg executables lgh_st3215_tools
ros2 pkg executables lgh_st3215_maintenance
ros2 pkg executables lgh_imu_tools
ros2 pkg executables littlegreen_biped_pkg
```

Show command-line options:

```bash
ros2 run <package> <executable> --help
```

Show launch arguments:

```bash
ros2 launch <package> <launch_file> --show-args
```

Inspect a running node:

```bash
ros2 node info /lgh_st3215_driver
ros2 param list /lgh_st3215_driver
ros2 param get /lgh_st3215_driver writes_enabled
ros2 param dump /lgh_st3215_driver
```

Inspect authority and QoS:

```bash
ros2 topic info /servo_target_radians --verbose
ros2 topic info /desired_position --verbose
ros2 service list -t
```

## 2. Workspace scripts

### `scripts/build_workspace.sh`

```text
--clean          remove build/, install/, and log/ before building
--skip-rosdep    skip rosdep install
--release        CMAKE_BUILD_TYPE=Release
--debug          CMAKE_BUILD_TYPE=Debug
-h, --help       print usage
```

Examples:

```bash
./scripts/build_workspace.sh
./scripts/build_workspace.sh --clean --release
./scripts/build_workspace.sh --skip-rosdep --debug
```

### `scripts/install_orange_pi.sh`

```text
--skip-ros       do not install ROS 2 or system dependencies
--skip-onnx      do not download ONNX Runtime
--skip-build     do not run rosdep/colcon
--no-bashrc      do not add the environment source block to ~/.bashrc
-h, --help       print usage
```

### `scripts/install_ubuntu_x86_64.sh`

Uses the same switches as the Orange Pi installer, but installs the x86_64 ONNX Runtime package.

### `scripts/verify_install.sh`

```text
--software-only  skip checks requiring attached robot hardware
```

## 3. micro-ROS agent for the current IMU source

Normal agent command:

```bash
ros2 run micro_ros_agent micro_ros_agent serial \
  --dev /dev/ttyACM0 \
  -b 115200 \
  -v0
```

Meaning:

| Option | Meaning |
|---|---|
| `serial` | use a serial transport |
| `--dev /dev/ttyACM0` | USB CDC device currently assigned to the IMU controller |
| `-b 115200` | serial baud rate expected by the firmware/transport |
| `-v0` | quiet normal-operation verbosity |

Discover the current device and agent options:

```bash
ls -l /dev/ttyACM*
ls -l /dev/serial/by-id/
ros2 run micro_ros_agent micro_ros_agent --help
```

The micro-ROS agent owns the USB serial device. Do not start two agents on the same device.

## 4. ST3215 driver launch arguments

```bash
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py --show-args
```

| Launch argument | Default | Purpose |
|---|---|---|
| `profile` | `commissioning` | publication profile: `commissioning` or `runtime_safe` |
| `config` | package `config/servo_driver.yaml` | base ROS parameter file |
| `servo_map` | package `config/servo_map.yaml` | calibrated servo/joint map |
| `port` | `/dev/ttyS3` | ST3215 UART device |
| `enable_writes` | `false` | enable physical position/torque writes; independent of profile |
| `default_pose_move_duration_sec` | `4.0` | duration of the guarded ramp to policy default |

Examples:

```bash
# Feedback only, full commissioning telemetry
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=commissioning enable_writes:=false

# Runtime publication surface, writes enabled
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=runtime_safe enable_writes:=true

# Alternate map and slower policy-default ramp
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=commissioning \
  servo_map:=/absolute/path/to/servo_map.yaml \
  default_pose_move_duration_sec:=8.0 \
  enable_writes:=true
```

### Driver profiles

Profiles alter ROS publications only. They do not enable writes, change bus timing, alter servo mapping, or select policy authority.

| Publication | commissioning | runtime_safe |
|---|:---:|:---:|
| `/joint_states` | yes | yes |
| `/joint_feedback_age_ms` | yes | yes |
| `/st3215_driver/diagnostics` | yes | yes |
| `/st3215_driver/raw_position_steps` | yes | no |
| `/st3215_driver/raw_speed` | yes | no |
| `/st3215_driver/telemetry` | yes | no |
| `/servo_target_steps_debug` | yes | no |

## 5. ST3215 driver parameters

The base values are in `src/lgh_st3215_driver/config/servo_driver.yaml`. Launch arguments override `port`, `joint_map_path`, `writes_enabled`, and `default_pose_move_duration_sec`.

### Hardware and rates

| Parameter | Default | Notes |
|---|---:|---|
| `port` | `/dev/ttyS3` | direct ST3215 UART |
| `baud` | `1000000` | servo bus baud |
| `joint_map_path` | empty | launch supplies `servo_map.yaml` |
| `bus_rate_hz` | `50.0` | native bus worker rate |
| `command_rate_hz` | `50.0` | command write cadence |
| `joint_state_publish_hz` | `50.0` | ROS state publication rate |
| `diagnostics_rate_hz` | `1.0` | diagnostic publication rate |
| `read_timeout_ms` | `10` | per transaction read bound |
| `write_timeout_ms` | `5` | write bound |

### Command and write behavior

| Parameter | Default | Notes |
|---|---:|---|
| `writes_enabled` | `false` | physical writes gate |
| `require_full_feedback_before_writes` | `true` | blocks writes until all joints have valid feedback |
| `startup_hold_current_position` | `true` | initializes the command buffer from measured state when available; it does not block a later active publisher |
| `command_timeout_ms` | `500` | stale external command timeout |
| `command_timeout_behavior` | `hold_last` | behavior after command timeout |
| `skip_unchanged_writes` | `false` | optionally suppress identical writes |
| `write_keepalive_ms` | `200` | maximum suppression interval when change detection is used |
| `default_speed` | `0` | fallback servo speed field |
| `default_acceleration` | `0` | fallback servo acceleration field |

### Feedback processing and scheduling

| Parameter | Default |
|---|---:|
| `rotate_read_order` | `true` |
| `read_order_stride` | `1` |
| `velocity_filter_alpha` | `0.30` |
| `velocity_deadband_rad_s` | `0.001` |
| `max_feedback_warn_age_ms` | `250` |
| `diagnostic_window_cycles` | `500` |
| `worker_cpu` | `-1` |
| `realtime_priority` | `0` |

`worker_cpu=-1` and `realtime_priority=0` leave CPU affinity and real-time scheduling disabled.

### Publication and naming parameters

The driver exposes parameters for each canonical topic/service name. They should normally remain unchanged because the rest of the workspace expects the standard interface.

```text
servo_target_topic
joint_state_topic
feedback_age_topic
raw_position_topic
raw_speed_topic
diagnostics_topic
telemetry_topic
legacy_debug_topic
target_debug_topic
move_default_pose_service
release_pose_override_service
abort_pose_move_service
hold_current_pose_service
disable_torque_all_service
enable_torque_hold_current_service
```

### Policy-default ramp parameters

| Parameter | Default | Meaning |
|---|---:|---|
| `default_pose_move_duration_sec` | `4.0` | total smooth ramp time |
| `default_pose_ramp_rate_hz` | `50.0` | ramp update rate |
| `default_pose_hold_after_move` | `true` | retain internal override after reaching the policy default |

## 6. Driver service behavior

All services use `std_srvs/srv/Trigger`.

| Service | Preconditions | Resulting authority state |
|---|---|---|
| `/st3215_driver/hold_current_pose` | writes enabled; fresh feedback preferred | measured pose is held; external targets blocked |
| `/st3215_driver/enable_torque_hold_current` | writes enabled; complete fresh feedback required | torque enabled at measured pose; external targets blocked |
| `/st3215_driver/move_to_default_pose` | writes enabled; complete fresh feedback required | smooth ramp to **policy default**; override normally remains active |
| `/st3215_driver/abort_pose_move` | ramp or override active | ramp stops; latest pose/last target is held; override remains active |
| `/st3215_driver/release_pose_override` | override active | external `/servo_target_radians` publisher gains authority immediately |
| `/st3215_driver/disable_torque_all` | writes enabled | torque disabled; override remains active |

Before releasing the override:

```bash
ros2 topic info /servo_target_radians --verbose
```

## 7. ST3215 preflight

```bash
ros2 run lgh_st3215_tools st3215_preflight --help
```

| Option | Default | Meaning |
|---|---:|---|
| `--mode` | required | `feedback`, `commissioning`, or `runtime` expectations |
| `--expect-writes` | `auto` | `auto`, `true`, or `false` |
| `--sample-sec` | `3.0` | observation window |
| `--timeout-sec` | `8.0` | startup/data timeout |
| `--max-feedback-age-ms` | `100` | per-joint freshness threshold |
| `--min-cycle-rate-hz` | `45.0` | minimum accepted bus cycle rate |
| `--max-cycle-work-p99-us` | `15000` | maximum accepted p99 cycle work time |
| `--output-root` | automatic | report root override |
| topic override options | canonical topics | diagnostics, joint, age, and telemetry topics |

## 8. Calibration commands

### `print_model_zero` / `print_policy_default`

```bash
ros2 run lgh_st3215_tools print_model_zero [--servo-map PATH]
ros2 run lgh_st3215_tools print_policy_default [--servo-map PATH]
```

`print_default_pose` remains an alias for policy default.

### `capture_calibration`

```bash
ros2 run lgh_st3215_tools capture_calibration --help
```

Important options:

| Option | Default | Meaning |
|---|---:|---|
| `--reference` | `model-zero` | `model-zero` recommended; `policy-default` retains legacy fixture behavior |
| `--joint NAME` | all joints | repeat to select multiple joints |
| `--samples` | `250` | raw samples per selected joint set |
| `--capture-timeout-sec` | `20.0` | collection timeout |
| `--preflight-timeout-sec` | `5.0` | initial data/diagnostic timeout |
| `--max-feedback-age-ms` | `50` | freshness gate |
| `--fine-threshold-steps` | `25` | fine correction classification |
| `--inspect-threshold-steps` | `100` | mechanical review classification threshold |
| `--max-sample-span-steps` | `8` | stability gate across samples |
| `--output-dir` | `calibration_reports` | report destination |
| `--allow-writes-enabled` | off | override normal writes-disabled requirement |
| `--yes` | off | skip typed `CAPTURE` confirmation |

### `apply_calibration`

```bash
ros2 run lgh_st3215_tools apply_calibration --help
```

| Option | Meaning |
|---|---|
| positional `proposal` | generated `center_step_proposal.yaml` |
| `--source-servo-map PATH` | required source-tree map to update |
| `--source-joint-map PATH` | explicit joint-map mirror; normally auto-detected |
| `--no-joint-map-sync` | update only the servo map |
| `--apply` | modify files; otherwise dry-run only |
| `--allow-map-mismatch` | override proposal/source hash mismatch after deliberate review |
| `--allow-large-corrections` | accept reviewed large center corrections |

The tool preserves `min_rad`, `max_rad`, `joint_zero_rad`, and `training_default_rad`, while recalculating raw `min_step`/`max_step` from the proposed center.

### `verify_model_zero` / `verify_policy_default`

`verify_calibration` remains a compatibility alias for `verify_model_zero`.

| Option | Default | Meaning |
|---|---:|---|
| `--joint NAME` | all | repeatable joint selection |
| `--samples` | `100` | verification samples |
| `--timeout-sec` | `10.0` | data timeout |
| `--max-feedback-age-ms` | `50` | freshness gate |
| `--pass-tolerance-steps` | `8` | PASS threshold |
| `--warn-tolerance-steps` | `16` | WARN threshold |
| `--allow-writes-enabled` | off | required for normal policy-default verification while torque/writes are active |
| `--output-dir` | `calibration_reports` | report destination |

## 9. Policy-default pose console

```bash
ros2 run lgh_st3215_tools assume_policy_default --help
```

Options:

```text
--move-service             default /st3215_driver/move_to_default_pose
--abort-service            default /st3215_driver/abort_pose_move
--diagnostics-topic        default /st3215_driver/diagnostics
--service-wait-sec         default 5.0
```

`pose_console` is a compatibility alias.

## 10. Servo identification

```bash
ros2 run lgh_st3215_tools servo_identification --help
```

Required selections:

| Option | Values |
|---|---|
| `--joint` | canonical name or index `0..11` |
| `--mode` | `step`, `step_sweep`, `deadband_staircase`, `triangle`, `hold_under_load` |

Command paths:

| Option | Default | Meaning |
|---|---|---|
| `--command-path` | `direct` | `direct` publishes `/servo_target_radians`; `outer` publishes `/desired_position` through the controller |
| `--direct-topic` | `/servo_target_radians` | direct command topic |
| `--outer-reference-topic` | `/desired_position` | outer-loop reference topic |

Step and sweep options:

```text
--direction positive|negative|both     default both
--amplitude-rad FLOAT                  default 0.02
--amplitudes-rad LIST                  default 0.02,0.05,0.10
--test-center-offset-rad FLOAT         default 0.0
--test-center-move-sec FLOAT           default 2.0
--test-center-settle-sec FLOAT         default 1.5
--max-test-offset-rad FLOAT            default 0.2
--joint-limit-margin-rad FLOAT         default 0.01
--baseline-sec FLOAT                   default 1.5
--step-hold-sec FLOAT                  default 2.5
--between-trials-sec FLOAT             default 1.0
--return-sec FLOAT                     default 2.0
--final-hold-sec FLOAT                 default 1.0
```

Deadband mode:

```text
--deadband-offsets-rad LIST            default 0.002,0.005,0.010,0.020
--deadband-dwell-sec FLOAT             default 1.5
--motion-threshold-rad FLOAT           default 0.002
```

Triangle mode:

```text
--triangle-amplitude-rad FLOAT         default 0.02
--triangle-frequency-hz FLOAT          default 0.1
--triangle-cycles FLOAT                default 2.0
```

Loaded hold mode:

```text
--load-baseline-sec FLOAT              default 3.0
--load-prepare-sec FLOAT               default 5.0
--load-hold-sec FLOAT                  default 10.0
--load-offset-rad FLOAT                default 0.0
--load-force-n FLOAT                   optional
--load-mass-kg FLOAT                   optional
--lever-arm-m FLOAT                    default 0.0
--stiffness-min-deflection-rad FLOAT   default 0.0015
```

Safety/data options:

```text
--max-feedback-age-ms INT              default 100
--joint-state-timeout-sec FLOAT        default 0.15
--preflight-timeout-sec FLOAT          default 8.0
--countdown-sec INT                    default 3
--support-condition TEXT               default securely_supported
--output-dir PATH                      default identification_reports
--notes TEXT
--allow-nonmax-motion-profile
--allow-all-2048-centers
```

## 11. Standing characterization

```bash
ros2 run lgh_st3215_tools standing_characterization --help
```

Required mode:

```text
--mode capture_pose|evaluate
```

Shared configuration:

```text
--servo-map PATH
--track1-contract PATH
--pose-library PATH
--standing-pose-name NAME              default normal_stand
--support-condition TEXT
--notes TEXT
```

Capture mode:

```text
--pose-name NAME
--base-com-height-mean-m FLOAT
--capture-window-sec FLOAT             default 2.0
--capture-min-samples INT              default 60
--capture-max-q-std-rad FLOAT          default 0.01
--capture-audit-root PATH
--reenable-torque-hold-after-capture
```

Evaluation mode:

```text
--poses CSV                            default normal_stand,shallow_crouch,medium_crouch,deep_crouch
--target-base-com-height-m FLOAT
--height-match-tolerance-m FLOAT       default 0.015
--return-between-poses                 default enabled
--no-return-between-poses
--crouch-speed-rad-s FLOAT             default 0.2
--stand-return-speed-rad-s FLOAT       default 0.15
--transition-speed-rad-s FLOAT         default 0.15
--min-transition-sec FLOAT             default 1.0
--command-rate-hz FLOAT                default 50.0
--settle-sec FLOAT                     default 5.0
--hold-sec FLOAT                       default 20.0
--deep-pose-name NAME                  default deep_crouch
--deep-hold-sec FLOAT                  default 8.0
--repeats INT                          default 1
```

Hardware guards:

```text
--max-feedback-age-ms INT              default 250
--max-current-a FLOAT                  default 1.5; 0 disables
--max-load-ratio FLOAT                 default 0.9; 0 disables
--min-voltage-v FLOAT                  default 9.0; 0 disables
--max-temp-c FLOAT                     default 60.0; 0 disables
--guard-consecutive-cycles INT         default 5
```

## 12. Reports and manifests

### Hardware snapshot

```bash
ros2 run lgh_st3215_tools hardware_snapshot \
  [--timeout-sec 5.0] \
  [--output-root PATH]
```

### Dataset manifest

```bash
ros2 run lgh_st3215_tools dataset_manifest \
  <output_dir> \
  [--experiment-type manual] \
  [--servo-map PATH]
```

## 13. Offline maintenance commands

The runtime driver must be stopped. These commands directly own the ST3215 UART and are read-only.

### `bus_scan`

```text
--port PATH          default /dev/ttyS3
--baud INT           default 1000000
--first-id INT       default 1
--last-id INT        default 253
--timeout-ms INT     default 3
--output PATH        optional YAML output
```

### `verify_ids`

```text
--port PATH
--baud INT
--servo-map PATH
--timeout-ms INT
```

### `register_dump`

```text
--id INT             required
--address INT        supports values such as 0x00
--length INT
--port PATH
--baud INT
--timeout-ms INT
--output PATH
```

### `backup_control_tables`

```text
--port PATH
--baud INT
--servo-map PATH
--output-root PATH
--address INT
--length INT
--timeout-ms INT
```

## 14. Hardware-limit tool

The standalone tool does not use the ROS driver. Stop the driver first.

### Capture

```bash
python3 tools/lgh_hardware_limit_tool/lgh_hardware_limit_tool.py capture --help
```

```text
--servo-map PATH                     current calibrated map
--device PATH                        default /dev/ttyS3
--baud INT                           default 1000000
--uart-timeout-s FLOAT               default 0.03
--samples INT                        default 80 per endpoint
--sample-rate-hz FLOAT               default 40.0
--margin-steps INT                   default 10
--training-contract PATH             alias: --deployment-contract
--compare-tolerance-rad FLOAT        default 0.02
--output-dir PATH                    required
--resume
--recapture-joint NAME               repeatable with --resume
--overwrite
```

### Re-render

```text
render
--capture PATH                       required physical_limit_capture.yaml
--servo-map PATH
--margin-steps INT                   default 10
--training-contract PATH             alias: --deployment-contract
--compare-tolerance-rad FLOAT        default 0.02
--output-dir PATH                    required
```

## 15. IMU tools

Start the micro-ROS agent before using these tools when the XIAO firmware is the active sensor source.

### `imu_preflight`

```text
--contract PATH
--duration-sec FLOAT       default 3.0
--timeout-sec FLOAT        default 8.0
--output-root PATH         default ~/.ros/lgh_reports
```

### `stationary_characterization`

```text
--topic NAME               default /imu/data
--duration-sec FLOAT       default 20.0
--output-root PATH         default ~/.ros/lgh_imu_datasets
```

### `orientation_audit`

```text
--pose TEXT                required
--contract PATH
--duration-sec FLOAT       default 3.0
--expected-axis x|y|z
--expected-sign positive|negative
--minimum-magnitude FLOAT  default 0.5
--output-root PATH         default ~/.ros/lgh_imu_audits
```

### `imu_recorder`

```text
--topic NAME               default /imu/data
--duration-sec FLOAT       default 10.0
--output-root PATH         default ~/.ros/lgh_imu_datasets
```

## 16. Policy bundle tools

### `policy_bundle_audit`

```text
--policy-yaml PATH         default packaged policy_latest.yaml
--joint-map PATH           default packaged joint_map.yaml
--onnx PATH                optional explicit ONNX path
--onnx-shape-probe PATH    optional explicit policy_onnx_contract_probe
--skip-onnx-shape-check    source-development escape hatch; never deployment acceptance
```

The installed command automatically locates `policy_onnx_contract_probe` beside the audit executable and verifies the actual float32 ONNX tensor shapes. Supported interfaces are `[1,45] -> [1,12]` and `[1,47] -> [1,12]`, with matching YAML metadata.

### `annotate_phase_guided_policy`

```text
--policy-yaml PATH         required genuine exported v1.4.7 YAML
--output PATH              optional; default POLICY_STEM.phase_guided.yaml
```

The tool adds only the canonical 47-D observation metadata to a separate YAML. It refuses 45-D policies, non-v4 actions, non-50-Hz timing, missing checksums, and unexpected tasks. It does not modify the ONNX model or checksum.

### `policy_onnx_contract_probe`

```bash
ros2 run littlegreen_biped_pkg policy_onnx_contract_probe /path/to/policy.onnx
```

Prints a JSON object containing input/output names, shapes, and element types. Normally invoked through `policy_bundle_audit`.

### `policy_runtime_metrics`

```text
--duration-sec FLOAT                 default 20.0
--policy-yaml PATH
--joint-map PATH
--freshness-sec FLOAT                default 0.2
--standing-command-threshold FLOAT   default 0.05
--joint-velocity-limit-rad-s FLOAT   default 4.72
--output-dir PATH
```

## 17. Policy launch arguments

### Shadow

```bash
ros2 launch littlegreen_biped_pkg policy_shadow.launch.py --show-args
```

| Argument | Default / purpose |
|---|---|
| `policy_config` | packaged paired policy YAML |
| `policy_runtime_config` | freshness, safety, and IMU transform YAML |
| `joint_map` | canonical hardware map |
| `onnx_model_path` | optional explicit ONNX override |
| `use_sim` | `false` |
| `override_imu` | `false`; nominal IMU substitution is not a live-hardware validation |
| `shadow_desired_position_topic` | `/policy_shadow/desired_position` |

### Live

```bash
ros2 launch littlegreen_biped_pkg policy_live.launch.py --show-args
```

Adds:

| Argument | Default / purpose |
|---|---|
| `pd_config` | downstream controller parameter file |
| `controller_mode` | `safety_only`; `outer_pd`/`outer_pid` are experimental |

The live launch does not start the ST3215 driver, micro-ROS agent, or joystick. Those remain separate terminals/processes.

### Full biped launch

`littlegreen_biped_launch.py` starts joystick, teleop, policy, joystick file bridge, and downstream controller. It does not start the ST3215 driver or micro-ROS agent.

Arguments:

```text
policy_config
policy_runtime_config
onnx_model_path
joint_map
pd_config
controller_mode
teleop_config
use_sim
override_imu
policy_output_mode live|shadow|disabled
```

### Teleop mux launch

`biped_teleop_mux.launch.py` additionally starts joystick, keyboard teleop in an `xterm`, and `twist_mux`. It requires a graphical terminal environment for the keyboard node.

## 18. Policy-node runtime parameters

The normal values are loaded from `policy_runtime.yaml`.

| Parameter | Default | Meaning |
|---|---:|---|
| `policy_output_mode` | `live` | `live`, `shadow`, or `disabled` |
| `publish_policy_debug` | `true` | publish observation/action diagnostics |
| `imu_timeout_sec` | `0.050` | `/imu/data` transport freshness gate |
| `joint_state_timeout_sec` | `0.150` | `/joint_states` transport freshness gate |
| `require_joint_feedback_age` | `true` | require the physical-age topic |
| `joint_feedback_age_topic_timeout_sec` | `0.150` | age-topic transport timeout |
| `joint_feedback_max_age_sec` | `0.250` | per-servo physical read-age limit |
| `command_timeout_sec` | `0.500` | stale command velocity timeout |
| `zero_command_on_timeout` | `true` | zero command observation after timeout |
| `require_joint_velocity` | `true` | policy requires qdot[12] |
| `override_imu` | `false` | nominal IMU substitution; not recommended live |
| `imu_to_base_matrix` | configured 3×3 transform | physical sensor frame to base frame |

For a 47-D policy the following interfaces are also active:

```text
/policy_debug/gait_phase   Float64MultiArray
/policy/reset_gait_phase   std_srvs/srv/Trigger
```

The phase debug array is `[phase, tick, period_ticks, sin, cos, expected_half_cycle]`. Reset is allowed in shadow/disabled and refused in live mode.

The policy YAML, not a launch parameter, defines the supported observation contract. v2.8.0 does not provide an operator override for the gait period or append order because those values are part of the exported model contract.

## 19. Downstream controller modes and options

Controller modes:

| Mode | Behavior |
|---|---|
| `safety_only` | sanitize, clamp, filter, speed-limit, and acceleration-limit policy targets |
| `outer_pd` | velocity-form outer PD around measured joint state |
| `outer_pid` | outer PD plus integral state and anti-windup |

Initial hardware deployment should use `safety_only`.

Important parameters in `pd_config.yaml`:

```text
control_rate_hz
command_timeout_sec
feedback_timeout_sec
require_feedback_for_outer_loop
initialize_output_from_feedback
startup_publish_default_pose
command_filter_alpha
enable_rate_limit
enable_accel_limit
max_joint_speed_rad_s[12]
max_joint_accel_rad_s2[12]
kp[12], kd[12], ki[12]
max_controller_velocity_rad_s[12]
max_controller_accel_rad_s2[12]
integral_error_limit_rad_sec[12]
publish_pd_torque_debug
publish_outer_loop_debug
```

Reset the controller output/state to measured feedback:

```bash
ros2 service call \
  /pd_controller/reset_to_feedback \
  std_srvs/srv/Trigger '{}'
```

## 20. Exit codes used by first-party tools

| Code | Meaning |
|---:|---|
| `0` | pass/success |
| `2` | test completed but acceptance failed |
| `3` | refused precondition or ownership conflict |
| `4` | timeout or required data unavailable |
| `5` | configuration error |
| `6` | hardware or I/O failure |
| `7` | operator abort |
| `70` | internal software error |
| `130` | interrupted with Ctrl+C |
