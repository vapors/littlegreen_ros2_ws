# LittleGreen Command Cheat Sheet

This page is the fast operating reference. For every option and parameter, use [`COMMAND_REFERENCE.md`](COMMAND_REFERENCE.md).

## 1. Environment and discovery

New interactive terminals normally source the LittleGreen overlay through `~/.bashrc`.

```bash
source ~/.bashrc
ros2 pkg prefix lgh_st3215_driver
```

Useful discovery commands:

```bash
ros2 pkg executables lgh_st3215_tools
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py --show-args
ros2 run lgh_st3215_tools servo_identification --help
ros2 node list
ros2 topic list -t
ros2 service list -t
ros2 param list /lgh_st3215_driver
```

## 2. Build and validate

```bash
cd ~/littlegreen_ros2_ws
./scripts/validate_source_tree.py
./scripts/verify_install.sh --software-only
./scripts/build_workspace.sh
./scripts/build_workspace.sh --clean
```

Package-only rebuild:

```bash
colcon build --symlink-install --packages-select <package_name>
source install/setup.bash
```

## 3. Start the micro-ROS IMU source

Use a dedicated terminal and keep it running whenever the current ICM-20948 micro-ROS firmware is the active `/imu/data` source:

```bash
ros2 run micro_ros_agent micro_ros_agent serial \
  --dev /dev/ttyACM0 \
  -b 115200 \
  -v0
```

Verify:

```bash
ros2 topic hz /imu/data
ros2 topic echo /imu/data --once
ros2 run lgh_imu_tools imu_preflight
```

If the USB device number changed:

```bash
ls -l /dev/ttyACM*
ls -l /dev/serial/by-id/
```

## 4. ST3215 driver launch modes

### Feedback-only commissioning

Use for calibration, inspection, and preflight. No physical SyncWrite commands are sent.

```bash
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=commissioning \
  enable_writes:=false
```

### Write-enabled commissioning

Use only for a deliberate guarded operation:

```bash
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=commissioning \
  enable_writes:=true
```

### Feedback-only runtime profile

```bash
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=runtime_safe \
  enable_writes:=false
```

### Write-enabled runtime profile

```bash
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=runtime_safe \
  enable_writes:=true
```

Before any write-enabled launch, inspect command authority:

```bash
ros2 topic info /servo_target_radians --verbose
```

For calibration or manual pose work, the expected publisher count is `0`.

### Common launch overrides

```bash
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=commissioning \
  port:=/dev/ttyS3 \
  enable_writes:=true \
  default_pose_move_duration_sec:=8.0
```

Show all launch arguments:

```bash
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py --show-args
```

## 5. Driver preflight and health

```bash
ros2 run lgh_st3215_tools st3215_preflight \
  --mode feedback \
  --expect-writes false

ros2 run lgh_st3215_tools st3215_preflight \
  --mode commissioning \
  --expect-writes false

ros2 run lgh_st3215_tools st3215_preflight \
  --mode runtime \
  --expect-writes true
```

```bash
ros2 run lgh_st3215_tools hardware_snapshot
ros2 topic hz /joint_states
ros2 topic hz /joint_feedback_age_ms
ros2 topic echo /st3215_driver/diagnostics --once
```

Commissioning-only inspection:

```bash
ros2 topic echo /st3215_driver/raw_position_steps --once
ros2 topic echo /servo_target_steps_debug --once
ros2 topic hz /st3215_driver/telemetry
```

## 6. Command authority and guarded services

### Hold the current measured pose

Blocks external `/servo_target_radians` commands until release:

```bash
ros2 service call \
  /st3215_driver/hold_current_pose \
  std_srvs/srv/Trigger '{}'
```

### Enable torque at the current measured pose

Seeds the current pose, enables torque, and keeps the pose override active:

```bash
ros2 service call \
  /st3215_driver/enable_torque_hold_current \
  std_srvs/srv/Trigger '{}'
```

### Move to the policy-default stance

```bash
ros2 run lgh_st3215_tools assume_policy_default
```

Equivalent compatibility service:

```bash
ros2 service call \
  /st3215_driver/move_to_default_pose \
  std_srvs/srv/Trigger '{}'
```

This means **policy default**, not model zero.

### Abort a policy-default ramp

```bash
ros2 service call \
  /st3215_driver/abort_pose_move \
  std_srvs/srv/Trigger '{}'
```

### Release command authority

Check the publisher first, because release is immediate:

```bash
ros2 topic info /servo_target_radians --verbose
ros2 service call \
  /st3215_driver/release_pose_override \
  std_srvs/srv/Trigger '{}'
```

### Disable torque

```bash
ros2 service call \
  /st3215_driver/disable_torque_all \
  std_srvs/srv/Trigger '{}'
```

Software holds and ROS services are not electrical emergency stops.

## 7. Calibration and servo replacement

Print the two distinct references:

```bash
ros2 run lgh_st3215_tools print_model_zero
ros2 run lgh_st3215_tools print_policy_default
```

Capture all model-zero centers:

```bash
ros2 run lgh_st3215_tools capture_calibration \
  --reference model-zero \
  --servo-map ~/littlegreen_ros2_ws/src/lgh_st3215_driver/config/servo_map.yaml
```

Capture one replacement servo:

```bash
ros2 run lgh_st3215_tools capture_calibration \
  --reference model-zero \
  --joint leg_left_knee_pitch_joint \
  --servo-map ~/littlegreen_ros2_ws/src/lgh_st3215_driver/config/servo_map.yaml
```

Dry-run and apply:

```bash
ros2 run lgh_st3215_tools apply_calibration \
  calibration_reports/<timestamp>/center_step_proposal.yaml \
  --source-servo-map ~/littlegreen_ros2_ws/src/lgh_st3215_driver/config/servo_map.yaml

ros2 run lgh_st3215_tools apply_calibration \
  calibration_reports/<timestamp>/center_step_proposal.yaml \
  --source-servo-map ~/littlegreen_ros2_ws/src/lgh_st3215_driver/config/servo_map.yaml \
  --allow-large-corrections \
  --apply
```

Verify:

```bash
ros2 run lgh_st3215_tools verify_model_zero
ros2 run lgh_st3215_tools assume_policy_default
ros2 run lgh_st3215_tools verify_policy_default --allow-writes-enabled
```

See [`SERVO_REPLACEMENT_CHECKLIST.md`](SERVO_REPLACEMENT_CHECKLIST.md).

## 8. Hardware-limit tool

The runtime driver must be stopped because this tool opens `/dev/ttyS3` directly.

Capture physical endpoints:

```bash
cd ~/littlegreen_ros2_ws
python3 tools/lgh_hardware_limit_tool/lgh_hardware_limit_tool.py capture \
  --device /dev/ttyS3 \
  --margin-steps 10 \
  --output-dir ~/lgh_limit_capture
```

Re-render an existing radian-limit capture after center calibration:

```bash
python3 tools/lgh_hardware_limit_tool/lgh_hardware_limit_tool.py render \
  --capture ~/lgh_limit_capture/physical_limit_capture.yaml \
  --margin-steps 10 \
  --output-dir ~/lgh_limit_capture/rendered_after_zero_calibration
```

## 9. Servo identification

```bash
ros2 run lgh_st3215_tools servo_identification --help
```

Example step sweep:

```bash
ros2 run lgh_st3215_tools servo_identification \
  --joint leg_left_ankle_pitch_joint \
  --mode step_sweep \
  --direction both \
  --amplitudes-rad 0.02,0.05,0.10 \
  --test-center-offset-rad 0.05 \
  --support-condition securely_supported
```

Keep the policy disconnected and run one joint at a time.

## 10. Standing characterization

```bash
ros2 run lgh_st3215_tools standing_characterization --help
```

Capture a manually established stance:

```bash
ros2 run lgh_st3215_tools standing_characterization \
  --mode capture_pose \
  --pose-name normal_stand \
  --base-com-height-mean-m 0.44
```

Evaluate the pose library:

```bash
ros2 run lgh_st3215_tools standing_characterization \
  --mode evaluate \
  --poses normal_stand,shallow_crouch,medium_crouch
```

## 11. Offline ST3215 maintenance

Stop `lgh_st3215_driver` first.

```bash
ros2 run lgh_st3215_maintenance bus_scan --first-id 1 --last-id 12
ros2 run lgh_st3215_maintenance verify_ids
ros2 run lgh_st3215_maintenance register_dump \
  --id 1 --address 0x00 --length 0x47
ros2 run lgh_st3215_maintenance backup_control_tables
```

## 12. IMU tools

Start the micro-ROS agent first, then:

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

## 13. Policy bundle, shadow, and live launch

Audit:

```bash
ros2 run littlegreen_biped_pkg policy_bundle_audit
```

Shadow:

```bash
ros2 launch littlegreen_biped_pkg policy_shadow.launch.py
ros2 run littlegreen_biped_pkg policy_runtime_metrics --duration-sec 30
```

Live policy plus safety-only downstream controller:

```bash
ros2 launch littlegreen_biped_pkg policy_live.launch.py \
  controller_mode:=safety_only
```

Show launch arguments:

```bash
ros2 launch littlegreen_biped_pkg policy_shadow.launch.py --show-args
ros2 launch littlegreen_biped_pkg policy_live.launch.py --show-args
```

## 14. Stop and inspect

```bash
ros2 node list
ros2 topic info /servo_target_radians --verbose
ros2 topic info /desired_position --verbose
ros2 topic info /command_velocity --verbose
```

Use `Ctrl+C` in launch terminals, then confirm the nodes and publishers actually stopped.
