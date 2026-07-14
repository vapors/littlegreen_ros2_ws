# Command Cheat Sheet

## Environment

New interactive Bash terminals load LittleGreen automatically through `~/.bashrc`.

For the current terminal after installation:

```bash
source ~/.bashrc
```

Direct environment sourcing is optional:

```bash
source ~/.config/littlegreen/ros2_env.sh
```

Verify the active overlay:

```bash
ros2 pkg prefix lgh_st3215_driver
```

## Validate and build

```bash
cd ~/littlegreen_ros2_ws
./scripts/validate_source_tree.py
./scripts/verify_install.sh --software-only
./scripts/build_workspace.sh
./scripts/build_workspace.sh --clean
```

Build one package:

```bash
colcon build --symlink-install --packages-select lgh_st3215_tools
```

## ST3215 driver profiles

### Commissioning, feedback only

```bash
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=commissioning \
  enable_writes:=false
```

### Commissioning, writes enabled

Use only for a planned guarded operation:

```bash
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=commissioning \
  enable_writes:=true
```

### Runtime-safe, feedback only

```bash
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=runtime_safe \
  enable_writes:=false
```

### Override UART port or pose-ramp duration

```bash
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=commissioning \
  port:=/dev/ttyS3 \
  enable_writes:=true \
  default_pose_move_duration_sec:=8.0
```

## Preflight and snapshots

```bash
ros2 run lgh_st3215_tools st3215_preflight \
  --mode feedback --expect-writes false

ros2 run lgh_st3215_tools st3215_preflight \
  --mode commissioning --expect-writes false

ros2 run lgh_st3215_tools st3215_preflight \
  --mode runtime --expect-writes false

ros2 run lgh_st3215_tools hardware_snapshot
```

## Inspect driver health

```bash
ros2 node list
ros2 topic list
ros2 topic hz /joint_states
ros2 topic hz /joint_feedback_age_ms
ros2 topic echo /st3215_driver/diagnostics --once
ros2 topic echo /joint_states --once
ros2 topic echo /joint_feedback_age_ms --once
```

Commissioning telemetry:

```bash
ros2 topic hz /st3215_driver/telemetry
ros2 topic echo /st3215_driver/telemetry --once
```

## Guarded driver services

```bash
ros2 service call \
  /st3215_driver/move_to_default_pose \
  std_srvs/srv/Trigger '{}'

ros2 service call \
  /st3215_driver/abort_pose_move \
  std_srvs/srv/Trigger '{}'

ros2 service call \
  /st3215_driver/hold_current_pose \
  std_srvs/srv/Trigger '{}'

ros2 service call \
  /st3215_driver/release_pose_override \
  std_srvs/srv/Trigger '{}'

ros2 service call \
  /st3215_driver/disable_torque_all \
  std_srvs/srv/Trigger '{}'

ros2 service call \
  /st3215_driver/enable_torque_hold_current \
  std_srvs/srv/Trigger '{}'
```

Software holds are not electrical emergency stops.

## Calibration and servo replacement

Print the two distinct references:

```bash
ros2 run lgh_st3215_tools print_model_zero
ros2 run lgh_st3215_tools print_policy_default
```

Capture model zero for one replacement servo:

```bash
ros2 run lgh_st3215_tools capture_calibration \
  --reference model-zero \
  --joint leg_left_knee_pitch_joint \
  --servo-map ~/littlegreen_ros2_ws/src/lgh_st3215_driver/config/servo_map.yaml
```

Capture all 12 model-zero centers by omitting `--joint`.

Dry-run/apply:

```bash
ros2 run lgh_st3215_tools apply_calibration \
  calibration_reports/<timestamp>/center_step_proposal.yaml \
  --source-servo-map ~/littlegreen_ros2_ws/src/lgh_st3215_driver/config/servo_map.yaml

ros2 run lgh_st3215_tools apply_calibration \
  calibration_reports/<timestamp>/center_step_proposal.yaml \
  --source-servo-map ~/littlegreen_ros2_ws/src/lgh_st3215_driver/config/servo_map.yaml \
  --apply
```

Verify model zero:

```bash
ros2 run lgh_st3215_tools verify_model_zero
```

Guarded move and policy-default verification:

```bash
ros2 run lgh_st3215_tools assume_policy_default

ros2 run lgh_st3215_tools verify_policy_default \
  --allow-writes-enabled
```

The model-space limits remain unchanged during center calibration; raw limits are derived from the new center. See `SERVO_REPLACEMENT_CHECKLIST.md`.

## Servo identification

Show the current CLI:

```bash
ros2 run lgh_st3215_tools servo_identification --help
```

Example:

```bash
ros2 run lgh_st3215_tools servo_identification \
  --joint leg_left_ankle_pitch_joint \
  --mode step_sweep \
  --direction both \
  --amplitudes-rad 0.02,0.05,0.10 \
  --support-condition securely_supported
```

Keep the policy disconnected and use the `commissioning` profile.

## Standing characterization

```bash
ros2 run lgh_st3215_tools standing_characterization --help
```

## Offline maintenance

Stop the runtime driver before running any maintenance command:

```bash
ros2 run lgh_st3215_maintenance bus_scan --first-id 1 --last-id 12
ros2 run lgh_st3215_maintenance verify_ids
ros2 run lgh_st3215_maintenance register_dump \
  --id 1 --address 0x00 --length 0x47
ros2 run lgh_st3215_maintenance backup_control_tables
```

## IMU tools

```bash
ros2 run lgh_imu_tools imu_preflight
ros2 run lgh_imu_tools stationary_characterization --duration-sec 20
ros2 run lgh_imu_tools orientation_audit --pose neutral
ros2 run lgh_imu_tools orientation_audit \
  --pose forward_pitch \
  --expected-axis x \
  --expected-sign positive
ros2 run lgh_imu_tools imu_recorder --duration-sec 10
```

## Policy deployment

Audit the paired policy bundle and hardware map:

```bash
ros2 run littlegreen_biped_pkg policy_bundle_audit
```

Rebuild after replacing a policy YAML/ONNX pair:

```bash
cd ~/littlegreen_ros2_ws
colcon build --symlink-install --packages-select littlegreen_biped_pkg
source install/setup.bash
```

Shadow mode, with the driver separately running in `runtime_safe` and writes disabled:

```bash
ros2 launch littlegreen_biped_pkg policy_shadow.launch.py
```

Inspect:

```bash
ros2 topic hz /policy_shadow/desired_position
ros2 topic echo /policy_status --once
ros2 topic echo /policy_ready --once
ros2 topic info /desired_position --verbose
```

Capture Track 1-aligned observable runtime metrics:

```bash
ros2 run littlegreen_biped_pkg policy_runtime_metrics --duration-sec 30
```

Guarded live mode, only after shadow acceptance and a write-enabled runtime preflight:

```bash
ros2 launch littlegreen_biped_pkg policy_live.launch.py \
  controller_mode:=safety_only
```

Full sequence: [`LIVE_POLICY_DEPLOYMENT.md`](LIVE_POLICY_DEPLOYMENT.md).

## ROS graph inspection

```bash
ros2 node list
ros2 topic list
ros2 topic info /servo_target_radians --verbose
ros2 topic info /desired_position --verbose
ros2 service list | grep st3215
```

## Tool exit codes

| Code | Meaning |
|---:|---|
| `0` | pass |
| `2` | test ran but acceptance criteria failed |
| `3` | refused safety/precondition |
| `4` | timeout or ROS resource unavailable |
| `5` | configuration error |
| `6` | hardware or I/O error |
| `7` | operator abort |
| `70` | internal software error |
| `130` | interrupted by `SIGINT` |
