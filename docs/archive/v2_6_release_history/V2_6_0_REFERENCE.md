# LittleGreen ROS 2 v2.6.0 Reference

## Workspace layout

```text
littlegreen_ros2_ws/
├── src/
├── scripts/
├── docs/
├── VERSION
└── README.md
```

`build/`, `install/`, and `log/` are generated locally and are excluded from the source release.

## Core packages

| Package | Build type | Primary role |
|---|---|---|
| `lgh_st3215_driver` | `ament_cmake` | ST3215 UART runtime and telemetry |
| `lgh_st3215_tools` | `ament_python` | guarded laboratory and commissioning tools |
| `lgh_st3215_maintenance` | `ament_cmake` | offline read-only direct-bus tools |
| `lgh_imu_tools` | `ament_python` | canonical IMU validation |
| `littlegreen_biped_pkg` | `ament_cmake` | ONNX policy runtime and shadow mode |
| `pd_controller_pkg` | `ament_cmake` | safety envelope and optional outer-loop shaping |
| `littlegreen_description` | `ament_python` | URDF/xacro and visualization resources |
| `joystick_bridge` | `ament_python` | command-velocity logging bridge |
| `teleop_twist_joy` | `ament_cmake` | joystick to Twist |

The source also carries the required ROS joystick driver packages.

## Install and validation scripts

```text
scripts/install_orange_pi.sh
scripts/install_onnxruntime_aarch64.sh
scripts/build_workspace.sh
scripts/configure_environment.sh
scripts/validate_source_tree.py
scripts/verify_install.sh
```

## ST3215 runtime

### Launch

```bash
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=commissioning \
  enable_writes:=false
```

Profiles:

```text
commissioning
runtime_safe
```

`enable_writes` remains a separate explicit launch argument.

### Command topic

```text
/servo_target_radians    std_msgs/msg/Float64MultiArray
```

### State and health

```text
/joint_states                         sensor_msgs/msg/JointState
/joint_feedback_age_ms                std_msgs/msg/UInt32MultiArray
/st3215_driver/diagnostics            diagnostic_msgs/msg/DiagnosticArray
```

Commissioning profile additionally publishes:

```text
/st3215_driver/raw_position_steps
/st3215_driver/raw_speed
/st3215_driver/telemetry
/servo_target_steps_debug
```

### Runtime services

```text
/st3215_driver/move_to_default_pose
/st3215_driver/abort_pose_move
/st3215_driver/hold_current_pose
/st3215_driver/release_pose_override
/st3215_driver/disable_torque_all
/st3215_driver/enable_torque_hold_current
```

## ST3215 tools

```bash
ros2 run lgh_st3215_tools st3215_preflight --mode feedback
ros2 run lgh_st3215_tools hardware_snapshot
ros2 run lgh_st3215_tools print_default_pose
ros2 run lgh_st3215_tools capture_calibration
ros2 run lgh_st3215_tools apply_calibration --help
ros2 run lgh_st3215_tools verify_calibration
ros2 run lgh_st3215_tools pose_console
ros2 run lgh_st3215_tools servo_identification --help
ros2 run lgh_st3215_tools standing_characterization --help
```

Common exit codes:

| Code | Meaning |
|---:|---|
| 0 | pass |
| 2 | test completed but criteria failed |
| 3 | refused precondition |
| 4 | timeout/unavailable |
| 5 | configuration error |
| 6 | hardware/I/O error |
| 7 | operator abort |
| 70 | internal software error |
| 130 | SIGINT |

## Offline maintenance

Stop `lgh_st3215_driver` before use:

```bash
ros2 run lgh_st3215_maintenance bus_scan
ros2 run lgh_st3215_maintenance verify_ids
ros2 run lgh_st3215_maintenance register_dump --id 1 --address 0x00 --length 0x47
ros2 run lgh_st3215_maintenance backup_control_tables
```

v2.6.0 maintenance is read-only. Direct-UART ownership is protected by an advisory local `flock`.

## IMU tools

```bash
ros2 run lgh_imu_tools imu_preflight
ros2 run lgh_imu_tools stationary_characterization --duration-sec 20
ros2 run lgh_imu_tools orientation_audit --pose neutral
ros2 run lgh_imu_tools imu_recorder --duration-sec 10
```

The source driver must publish `sensor_msgs/msg/Imu` on `/imu/data` according to `lgh_imu_tools/config/imu_contract.yaml`.

## Policy modes

```text
disabled   build observations, no ONNX action output
shadow     ONNX output to /policy_shadow/desired_position only
live       output to /desired_position
```

Shadow launch:

```bash
ros2 launch littlegreen_biped_pkg policy_shadow.launch.py
```

Full stack launch remains available:

```bash
ros2 launch littlegreen_biped_pkg littlegreen_biped_launch.py \
  policy_output_mode:=shadow \
  controller_mode:=safety_only
```

The dedicated shadow launch is preferred for initial validation because it does not launch the PD controller or joystick stack.

## Build examples

Complete build:

```bash
cd ~/littlegreen_ros2_ws
./scripts/build_workspace.sh
```

Selected packages:

```bash
colcon build --symlink-install --packages-select \
  lgh_st3215_driver lgh_st3215_tools lgh_st3215_maintenance \
  lgh_imu_tools littlegreen_biped_pkg pd_controller_pkg
```
