# LittleGreen ROS 2 Workspace v2.6.0 — Bash Command Cheat Sheet

Assumes:

```bash
cd ~/littlegreen_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## Build

### Install dependencies

```bash
rosdep install --from-paths src --ignore-src -r -y
```

### Build core project packages

```bash
colcon build --packages-select \
  lgh_st3215_driver lgh_st3215_tools lgh_st3215_maintenance \
  lgh_imu_tools littlegreen_biped_pkg pd_controller_pkg \
  joystick_bridge littlegreen_description \
  --symlink-install
source install/setup.bash
```

### Rebuild native driver only

```bash
colcon build --packages-select lgh_st3215_driver lgh_st3215_tools lgh_st3215_maintenance --symlink-install
source install/setup.bash
```

### Rebuild policy/control path

```bash
colcon build   --packages-select littlegreen_biped_pkg pd_controller_pkg joystick_bridge   --symlink-install
source install/setup.bash
```

## Native driver bring-up

### Feedback only — safest first launch

```bash
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=commissioning enable_writes:=false
```

### Write enabled

```bash
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=commissioning enable_writes:=true
```

### Write enabled with slower default-pose ramp

```bash
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=commissioning enable_writes:=true default_pose_move_duration_sec:=8.0
```

### Override UART device

```bash
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=commissioning port:=/dev/ttyS3 enable_writes:=false
```

### Runtime-safe publication profile

```bash
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=runtime_safe enable_writes:=false
```

## Domain preflight and snapshots

```bash
ros2 run lgh_st3215_tools st3215_preflight --mode feedback --expect-writes false
ros2 run lgh_st3215_tools hardware_snapshot
ros2 run lgh_imu_tools imu_preflight
```

Maintenance requires the driver to be stopped:

```bash
ros2 run lgh_st3215_maintenance verify_ids
ros2 run lgh_st3215_maintenance backup_control_tables
```

## Basic health checks

```bash
ros2 node list
ros2 topic list
ros2 service list
```

```bash
ros2 topic hz /joint_states
ros2 topic hz /joint_feedback_age_ms
ros2 topic hz /st3215_driver/telemetry
ros2 topic hz /st3215_driver/diagnostics
```

```bash
ros2 topic echo /joint_states --once
ros2 topic echo /joint_feedback_age_ms --once
ros2 topic echo /st3215_driver/diagnostics --once
ros2 topic echo /st3215_driver/telemetry --once
```

```bash
ros2 topic info -v /st3215_driver/diagnostics
ros2 topic info -v /servo_target_radians
```

## Default pose and pose override services

### Interactive guarded move with keyboard abort

```bash
ros2 run lgh_st3215_tools pose_console
```

### Direct service call: move to training default

```bash
ros2 service call   /st3215_driver/move_to_default_pose   std_srvs/srv/Trigger '{}'
```

### Abort ramp and hold

```bash
ros2 service call   /st3215_driver/abort_pose_move   std_srvs/srv/Trigger '{}'
```

### Latch current measured pose

```bash
ros2 service call   /st3215_driver/hold_current_pose   std_srvs/srv/Trigger '{}'
```

### Align PD controller to current feedback

```bash
ros2 service call   /pd_controller/reset_to_feedback   std_srvs/srv/Trigger '{}'
```

### Release native driver pose override

```bash
ros2 service call   /st3215_driver/release_pose_override   std_srvs/srv/Trigger '{}'
```

### Explicit all-servo torque off

```bash
ros2 service call   /st3215_driver/disable_torque_all   std_srvs/srv/Trigger '{}'
```

### Re-enable torque at measured current pose

```bash
ros2 service call   /st3215_driver/enable_torque_hold_current   std_srvs/srv/Trigger '{}'
```

## Calibration workflow

### Print default-pose reference

```bash
ros2 run lgh_st3215_tools print_default_pose --servo-map ~/littlegreen_ros2_ws/src/lgh_st3215_driver/config/servo_map.yaml
```

### Capture center-step proposal

```bash
ros2 run lgh_st3215_tools capture_calibration --servo-map ~/littlegreen_ros2_ws/src/lgh_st3215_driver/config/servo_map.yaml
```

### Dry-run proposed calibration

```bash
ros2 run lgh_st3215_tools apply_calibration calibration_reports/<timestamp>/center_step_proposal.yaml --source-servo-map ~/littlegreen_ros2_ws/src/lgh_st3215_driver/config/servo_map.yaml
```

### Apply proposal

```bash
ros2 run lgh_st3215_tools apply_calibration calibration_reports/<timestamp>/center_step_proposal.yaml --source-servo-map ~/littlegreen_ros2_ws/src/lgh_st3215_driver/config/servo_map.yaml   --apply
```

### Verify default pose

```bash
ros2 run lgh_st3215_tools verify_calibration --servo-map ~/littlegreen_ros2_ws/src/lgh_st3215_driver/config/servo_map.yaml
```

## Policy/control stack

### Dedicated policy shadow — recommended v2.6.0 path

```bash
ros2 launch littlegreen_biped_pkg policy_shadow.launch.py
```

### Primary stack with safety-only downstream controller

Use only after the Track 1 deployment bundle has been paired and audited:

```bash
ros2 launch littlegreen_biped_pkg littlegreen_biped_launch.py \
  controller_mode:=safety_only
```

### IMU override for bench/simulation diagnostics

```bash
ros2 launch littlegreen_biped_pkg littlegreen_biped_launch.py \
  controller_mode:=safety_only \
  policy_output_mode:=shadow \
  override_imu:=true
```

`override_imu` is not a hardware-commissioning substitute.

### Explicit policy/model overrides in shadow mode

```bash
ros2 launch littlegreen_biped_pkg policy_shadow.launch.py \
  policy_config:=/absolute/path/policy.yaml \
  onnx_model_path:=/absolute/path/policy.onnx
```

### Deferred outer-loop modes

The package still supports `outer_pd` and `outer_pid` for controlled future experiments, but v2.6.0 does not authorize aggressive outer-loop tuning during first deployment.

```bash
ros2 launch littlegreen_biped_pkg littlegreen_biped_launch.py \
  controller_mode:=outer_pd
```

```bash
ros2 launch littlegreen_biped_pkg littlegreen_biped_launch.py \
  controller_mode:=outer_pid
```

## Policy readiness and debug

```bash
ros2 topic echo /policy_ready --once
ros2 topic echo /policy_status --once
```

```bash
ros2 topic echo /policy_debug/observation --once
ros2 topic echo /policy_debug/raw_action --once
ros2 topic echo /policy_debug/clipped_raw_action --once
ros2 topic echo /policy_debug/target_unclipped --once
ros2 topic echo /policy_debug/target_clipped --once
ros2 topic echo /policy_debug/saturation_mask --once
```

## Outer-controller debug

```bash
ros2 topic echo /outer_controller/status --once
ros2 topic echo /outer_controller/position_error --once
ros2 topic echo /outer_controller/velocity_command --once
ros2 topic echo /outer_controller/integral_error --once
```

## Joystick commands

### Primary full launch already starts joy and teleop

```bash
ros2 launch littlegreen_biped_pkg littlegreen_biped_launch.py
```

### Inspect raw joystick

```bash
ros2 topic echo /joy
```

### Inspect canonical command velocity

```bash
ros2 topic echo /command_velocity
```

### Enumerate SDL joystick devices

```bash
ros2 run joy joy_enumerate_devices
```

### Standalone joystick driver

```bash
ros2 run joy joy_node
```

## Servo identification

### Direct single step

```bash
ros2 run lgh_st3215_tools servo_identification   --joint leg_left_ankle_pitch_joint   --mode step   --command-path direct   --amplitude-rad 0.10   --direction both
```

### Direct step sweep

```bash
ros2 run lgh_st3215_tools servo_identification   --joint leg_left_ankle_pitch_joint   --mode step_sweep   --command-path direct   --amplitudes-rad 0.02,0.05,0.10,0.15,0.20   --direction both
```

### Deadband staircase

```bash
ros2 run lgh_st3215_tools servo_identification   --joint leg_left_ankle_pitch_joint   --mode deadband_staircase   --deadband-offsets-rad 0.002,0.005,0.01,0.02
```

### Triangle excitation

```bash
ros2 run lgh_st3215_tools servo_identification   --joint leg_left_ankle_pitch_joint   --mode triangle   --triangle-amplitude-rad 0.02   --triangle-frequency-hz 0.10   --triangle-cycles 2
```

### Known-load hold

```bash
ros2 run lgh_st3215_tools servo_identification   --joint leg_left_knee_pitch_joint   --mode hold_under_load   --load-mass-kg 1.0   --lever-arm-m 0.10
```

## Standing-pose capture

Default pose library:

```text
~/.ros/lgh_standing_poses.yaml
```

### Capture normal stand

```bash
ros2 run lgh_st3215_tools standing_characterization   --mode capture_pose   --pose-name normal_stand   --base-com-height-mean-m 0.46   --pose-library ~/.ros/lgh_standing_poses.yaml
```

### Recapture only medium crouch

```bash
ros2 run lgh_st3215_tools standing_characterization   --mode capture_pose   --pose-name medium_crouch   --base-com-height-mean-m 0.38   --pose-library ~/.ros/lgh_standing_poses.yaml
```

## Standing-load evaluation

### Conservative full ladder

```bash
ros2 run lgh_st3215_tools standing_characterization   --mode evaluate   --pose-library ~/.ros/lgh_standing_poses.yaml   --poses normal_stand,shallow_crouch,medium_crouch,deep_crouch,medium_crouch,shallow_crouch,normal_stand   --no-return-between-poses   --crouch-speed-rad-s 0.08   --stand-return-speed-rad-s 0.06   --settle-sec 5   --hold-sec 15   --deep-hold-sec 8
```

### Loaded speed sweep example: 3 rad/s with 0.25 s floor

```bash
ros2 run lgh_st3215_tools standing_characterization   --mode evaluate   --pose-library ~/.ros/lgh_standing_poses.yaml   --poses normal_stand,shallow_crouch,medium_crouch,deep_crouch,medium_crouch,shallow_crouch,normal_stand   --no-return-between-poses   --crouch-speed-rad-s 3.0   --stand-return-speed-rad-s 3.0   --min-transition-sec 0.25   --settle-sec 5   --hold-sec 8   --deep-hold-sec 8
```

## RViz and Gazebo description

```bash
ros2 launch littlegreen_description display.launch.py gui:=True
```

```bash
ros2 launch littlegreen_description gazebo.launch.py
```

## Useful process and graph troubleshooting

```bash
pgrep -af lgh_st3215_driver_node
pgrep -af 'ros2 launch'
```

```bash
watch -n 0.5 'date; echo; pgrep -af lgh_st3215_driver_node'
```

```bash
echo "RMW=$RMW_IMPLEMENTATION"
echo "DOMAIN=$ROS_DOMAIN_ID"
echo "LOCALHOST=$ROS_LOCALHOST_ONLY"
env | grep -E '^(ROS|RMW|FAST|CYCLONE)'
```

```bash
sudo journalctl -k --since '10 minutes ago' |   grep -Ei 'oom|killed process|segfault|st3215|fastdds|cyclone'
```

## Policy shadow

Run the servo driver separately in `runtime_safe` with writes disabled, then:

```bash
ros2 launch littlegreen_biped_pkg policy_shadow.launch.py
ros2 topic info /desired_position --verbose
ros2 topic info /policy_shadow/desired_position --verbose
ros2 topic echo /policy_status --once
```
