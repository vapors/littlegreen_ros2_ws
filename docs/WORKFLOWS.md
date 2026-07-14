# Recommended Workflows

This page identifies which processes should be running for each task. Command options are in [`COMMAND_REFERENCE.md`](COMMAND_REFERENCE.md); the compact commands are in [`COMMAND_CHEATSHEET.md`](COMMAND_CHEATSHEET.md).

## 1. Fresh installation

```text
validate source
→ install software
→ open a new shell
→ verify the overlay
→ run staged commissioning
```

Use:

- [`INSTALL_ORANGE_PI.md`](INSTALL_ORANGE_PI.md)
- [`FRESH_INSTALL_CHECKLIST.md`](FRESH_INSTALL_CHECKLIST.md)

## 2. Feedback-only ST3215 observation

Running processes:

```text
lgh_st3215_driver: commissioning, writes disabled
no policy
no PD controller
```

```bash
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=commissioning \
  enable_writes:=false
```

```bash
ros2 run lgh_st3215_tools st3215_preflight \
  --mode feedback \
  --expect-writes false

ros2 run lgh_st3215_tools hardware_snapshot
```

## 3. Current IMU source and validation

Running processes:

```text
micro_ros_agent owns /dev/ttyACM0
XIAO firmware publishes /imu/data
```

```bash
ros2 run micro_ros_agent micro_ros_agent serial \
  --dev /dev/ttyACM0 \
  -b 115200 \
  -v0
```

```bash
ros2 topic hz /imu/data
ros2 run lgh_imu_tools imu_preflight
ros2 run lgh_imu_tools stationary_characterization --duration-sec 20
```

Use [`IMU_CALIBRATION.md`](IMU_CALIBRATION.md) after source, firmware, or mount changes.

## 4. Offline bus maintenance

Running processes:

```text
lgh_st3215_driver stopped
one maintenance command owns /dev/ttyS3
policy and controller stopped
```

```bash
ros2 run lgh_st3215_maintenance bus_scan --first-id 1 --last-id 12
ros2 run lgh_st3215_maintenance verify_ids
ros2 run lgh_st3215_maintenance register_dump \
  --id 1 --address 0x00 --length 0x47
ros2 run lgh_st3215_maintenance backup_control_tables
```

## 5. Model-zero calibration

Running processes:

```text
commissioning driver, writes disabled
no /servo_target_radians publisher
robot mechanically supported
selected joints manually aligned to model zero
```

Check authority:

```bash
ros2 topic info /servo_target_radians --verbose
```

Capture:

```bash
ros2 run lgh_st3215_tools capture_calibration \
  --reference model-zero \
  --servo-map ~/littlegreen_ros2_ws/src/lgh_st3215_driver/config/servo_map.yaml
```

Then dry-run, apply, rebuild, verify model zero, and command policy default. Use:

- [`CALIBRATION_WORKFLOW.md`](CALIBRATION_WORKFLOW.md)
- [`SERVO_REPLACEMENT_CHECKLIST.md`](SERVO_REPLACEMENT_CHECKLIST.md)

## 6. Policy-default pose verification

Running processes:

```text
commissioning driver, writes enabled
policy and PD controller stopped
robot supported
```

Before launch:

```bash
ros2 topic info /servo_target_radians --verbose
```

Then:

```bash
ros2 run lgh_st3215_tools assume_policy_default
ros2 run lgh_st3215_tools verify_policy_default --allow-writes-enabled
```

The guarded ramp normally leaves the internal pose override active. External targets remain blocked until `release_pose_override` is called.

## 7. Hardware-limit capture

Running processes:

```text
runtime driver stopped
standalone tool owns /dev/ttyS3
robot supported and torque state managed by the tool
```

```bash
python3 tools/lgh_hardware_limit_tool/lgh_hardware_limit_tool.py capture \
  --device /dev/ttyS3 \
  --output-dir ~/lgh_limit_capture
```

A like-for-like servo replacement normally needs center calibration, not another endpoint capture.

## 8. Servo identification

Running processes:

```text
commissioning driver, writes enabled
policy disconnected
PD controller absent for direct command path
one identification tool
one joint under test
```

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
  --test-center-offset-rad 0.05 \
  --support-condition securely_supported
```

## 9. Standing characterization

Use only after suspended calibration and identification are accepted.

Running processes:

```text
commissioning driver, writes enabled
policy disconnected
standing characterization tool owns the command path
real IMU recommended
fall arrest and power disconnect available
```

```bash
ros2 run lgh_st3215_tools standing_characterization --help
```

## 10. Policy shadow

Recommended terminals:

```text
A: micro-ROS agent
B: runtime_safe driver, writes disabled
C: policy_shadow.launch.py
D: preflight/metrics/graph checks
```

```bash
# Terminal A
ros2 run micro_ros_agent micro_ros_agent serial \
  --dev /dev/ttyACM0 -b 115200 -v0

# Terminal B
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=runtime_safe enable_writes:=false

# Terminal C
ros2 launch littlegreen_biped_pkg policy_shadow.launch.py
```

The packaged default remains the 45-D v1.4.5s3 pair. A future 47-D v1.4.7 pair must pass strict YAML/ONNX tensor-shape audit before it is selected.

Verify:

```bash
ros2 topic info /desired_position --verbose
ros2 topic info /policy_shadow/desired_position --verbose
ros2 run littlegreen_biped_pkg policy_runtime_metrics --duration-sec 30
```

For a 47-D policy also verify the 36-tick expected phase clock:

```bash
ros2 topic echo /policy_debug/gait_phase
```

This is expected policy timing, not measured foot contact. The phase reset service may be used only in shadow/disabled mode.

## 11. Guarded live policy

Recommended terminals:

```text
A: micro-ROS agent
B: runtime_safe driver, writes enabled
C: policy_live.launch.py
D: command source, when added
E: diagnostics and hold/stop console
```

Required transition:

```text
policy bundle audit
→ shadow accepted
→ stop shadow
→ verify stale publishers are gone
→ write-enabled driver preflight
→ hold current pose
→ start live policy/controller
→ inspect /desired_position and /servo_target_radians publishers
→ release driver override
```

Commands:

```bash
ros2 run littlegreen_biped_pkg policy_bundle_audit

ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=runtime_safe enable_writes:=true

ros2 service call \
  /st3215_driver/hold_current_pose \
  std_srvs/srv/Trigger '{}'

ros2 launch littlegreen_biped_pkg policy_live.launch.py \
  controller_mode:=safety_only

ros2 topic info /servo_target_radians --verbose

ros2 service call \
  /st3215_driver/release_pose_override \
  std_srvs/srv/Trigger '{}'
```

Use [`LIVE_POLICY_DEPLOYMENT.md`](LIVE_POLICY_DEPLOYMENT.md) for the complete acceptance and stop sequence.

## 12. Shutdown verification

Stopping one launch does not stop the independent driver, IMU agent, or another launch.

```bash
ros2 node list
ros2 topic info /servo_target_radians --verbose
ros2 topic info /desired_position --verbose
ps -ef | grep -E 'ros2|micro_ros_agent|littlegreen|pd_controller' | grep -v grep
```
