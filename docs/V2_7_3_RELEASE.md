# LittleGreen ROS 2 v2.7.3

v2.7.3 is a documentation and command-discoverability release. Runtime code, driver configuration, calibration values, policy YAML, ONNX model, joint map, and action contract are unchanged from v2.7.2.

## Main additions

### Full command reference

Added [`COMMAND_REFERENCE.md`](COMMAND_REFERENCE.md), covering:

- workspace build/install script switches;
- ST3215 driver launch arguments and parameters;
- driver service preconditions and authority effects;
- preflight, calibration, verification, identification, and standing-tool options;
- offline maintenance options;
- hardware-limit capture and render options;
- IMU tools and micro-ROS agent startup;
- policy audit/metrics options;
- shadow, live, full biped, and teleop-mux launch arguments;
- policy runtime and downstream controller parameters;
- first-party exit-code meanings.

### Updated command cheat sheet

The cheat sheet is now task-oriented and includes:

- discovery commands such as `--show-args`, `--help`, `ros2 param`, and `ros2 pkg executables`;
- feedback-only and write-enabled driver combinations;
- explicit command-authority checks;
- all guarded driver services;
- model-zero and policy-default calibration commands;
- hardware-limit capture/render commands;
- identification and standing examples;
- policy shadow/live sequences.

### Current micro-ROS agent command

The current IMU source command is now included wherever `/imu/data` is required:

```bash
ros2 run micro_ros_agent micro_ros_agent serial \
  --dev /dev/ttyACM0 \
  -b 115200 \
  -v0
```

The docs also explain USB device re-enumeration, `/dev/serial/by-id`, device ownership, and IMU topic validation.

### ROS graph and command authority

Added [`ROS_GRAPH_AND_AUTHORITY.md`](ROS_GRAPH_AND_AUTHORITY.md), documenting:

- the policy → controller → driver command chain;
- write-enabled and pose-override authority states;
- why `startup_hold_current_position` is not a persistent authority lock;
- checks for stale `/servo_target_radians` publishers;
- safe override release;
- independent terminal/process layout for calibration, shadow, and live operation.

### Workflow and troubleshooting refresh

Updated commissioning, fresh-install, calibration, replacement-servo, policy deployment, IMU, workflow, interface, and troubleshooting pages to expose the relevant commands and process boundaries.

## Compatibility

No migration is required from v2.7.2. A documentation hotfix can be applied without rebuilding ROS packages.

A rebuild is unnecessary unless the operator also changed source code, configuration, calibration maps, or policy artifacts independently.
