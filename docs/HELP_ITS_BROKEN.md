# Troubleshooting Guide

Start by identifying which boundary failed: environment, serial device, ROS graph, driver feedback, IMU source, policy readiness, or command authority.

## 1. Confirm the active workspace

```bash
ros2 pkg prefix lgh_st3215_driver
ros2 pkg prefix littlegreen_biped_pkg
ros2 pkg prefix littlegreen_description
```

Inspect overlays:

```bash
echo "$AMENT_PREFIX_PATH" | tr ':' '\n'
echo "$CMAKE_PREFIX_PATH" | tr ':' '\n'
```

Search generated setup files for old Berkeley/BHL underlays:

```bash
grep -RnsE \
  'berkeley_ros2_ws|bhl_st3215_driver|berkeley_biped_pkg|lilgreen_description' \
  ~/littlegreen_ros2_ws/install/setup.* \
  ~/littlegreen_ros2_ws/install/_local_setup_util_* \
  2>/dev/null
```

If an old workspace appears, perform a clean rebuild from a sterile shell.

## 2. Clean rebuild without inherited overlays

```bash
env -i \
  HOME="$HOME" \
  USER="$USER" \
  LOGNAME="$LOGNAME" \
  SHELL=/bin/bash \
  TERM="${TERM:-xterm}" \
  PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  LANG="${LANG:-en_US.UTF-8}" \
  bash --noprofile --norc
```

Inside the clean shell:

```bash
source /opt/ros/humble/setup.bash
cd ~/littlegreen_ros2_ws
rm -rf build install log
export ONNXRUNTIME_DIR="$HOME/libs/onnxruntime-linux-aarch64-1.22.0"
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
colcon build --symlink-install --event-handlers console_direct+
source install/setup.bash
```

Verify the new packages and confirm the old names are absent.

## 3. ST3215 UART cannot open

```bash
ls -l /dev/ttyS3
id -nG
lsof /dev/ttyS3
fuser -v /dev/ttyS3
```

Common causes:

- another driver or maintenance process owns the UART;
- the user is not in `dialout`;
- the Orange Pi UART overlay/pinmux is not active;
- the configured launch `port` is wrong.

The runtime driver and offline maintenance commands must never run together.

## 4. Driver starts but feedback is missing or stale

Use commissioning mode:

```bash
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=commissioning enable_writes:=false
```

Inspect:

```bash
ros2 topic hz /joint_states
ros2 topic echo /joint_feedback_age_ms --once
ros2 topic echo /st3215_driver/diagnostics --once
ros2 run lgh_st3215_tools st3215_preflight \
  --mode commissioning --expect-writes false
```

Check servo power, UART wiring, IDs, baud, and increasing protocol/I/O counters before changing timing parameters.

## 5. Robot moves unexpectedly at driver startup

Immediately stop or remove servo power if motion is unsafe. Then inspect command authority with writes disabled:

```bash
ros2 topic info /servo_target_radians --verbose
ros2 node list | grep -E 'biped|policy|pd_controller|identification|standing|teleop'
```

A stale `pd_controller_node`, policy launch, or laboratory tool can continue publishing after another terminal is restarted.

During calibration, expected state:

```text
/servo_target_radians Publisher count: 0
```

Before releasing a pose override, inspect both publisher identity and target:

```bash
ros2 topic info /servo_target_radians --verbose
ros2 topic echo /servo_target_radians --once
```

See [`ROS_GRAPH_AND_AUTHORITY.md`](ROS_GRAPH_AND_AUTHORITY.md).

## 6. Driver pose override will not accept commands

Inspect diagnostics:

```bash
ros2 topic echo /st3215_driver/diagnostics --once
```

If `pose_override_active=true`, external targets are intentionally blocked. Release only after identifying the publisher:

```bash
ros2 topic info /servo_target_radians --verbose
ros2 service call \
  /st3215_driver/release_pose_override \
  std_srvs/srv/Trigger '{}'
```

## 7. micro-ROS agent or `/imu/data` is missing

Check USB enumeration:

```bash
ls -l /dev/ttyACM*
ls -l /dev/serial/by-id/
lsof /dev/ttyACM0
```

Start the agent using the actual device:

```bash
ros2 run micro_ros_agent micro_ros_agent serial \
  --dev /dev/ttyACM0 \
  -b 115200 \
  -v0
```

Then:

```bash
ros2 topic hz /imu/data
ros2 topic echo /imu/data --once
ros2 run lgh_imu_tools imu_preflight
```

Do not start two agents on the same device. A serial monitor can also block the port.

## 8. Policy is not ready

Check every required input independently:

```bash
ros2 topic hz /imu/data
ros2 topic hz /joint_states
ros2 topic hz /joint_feedback_age_ms
ros2 topic echo /policy_status --once
ros2 topic echo /policy_ready --once
```

Audit the installed pair:

```bash
ros2 run littlegreen_biped_pkg policy_bundle_audit
```

Common causes:

- micro-ROS agent stopped or wrong USB device;
- stale servo feedback;
- policy YAML and ONNX checksum mismatch;
- YAML observation count does not match the ONNX input tensor;
- a 47-D YAML is missing exact gait-phase metadata;
- unsupported observation count (anything other than 45 or 47);
- policy/joint-map contract mismatch;
- policy node was not restarted after replacing files;
- `override_imu` or output mode differs from the intended launch.

For a phase-guided policy, inspect:

```bash
ros2 topic echo /policy_debug/gait_phase --once
```

The phase freezes while readiness is closed. That is expected and is not a clock fault. In shadow mode only, reset explicitly with:

```bash
ros2 service call /policy/reset_gait_phase std_srvs/srv/Trigger '{}'
```

Live-mode reset refusal is intentional.

## 9. Shadow mode publishes live targets

Shadow should publish only:

```text
/policy_shadow/desired_position
```

Check:

```bash
ros2 topic info /desired_position --verbose
ros2 topic info /policy_shadow/desired_position --verbose
```

If `/desired_position` has a publisher, another live/full launch is still running.

## 10. Calibration proposal looks large

Confirm the robot is at **model zero**, not policy default. The two references are different:

```bash
ros2 run lgh_st3215_tools print_model_zero
ros2 run lgh_st3215_tools print_policy_default
```

Large correction flags are review gates. Use `--allow-large-corrections` only after confirming the physical model-zero fixture and reviewing derived raw limits.

## 11. Command or launch option seems hidden

```bash
ros2 pkg executables <package>
ros2 run <package> <executable> --help
ros2 launch <package> <launch_file> --show-args
ros2 param list <node_name>
ros2 param dump <node_name>
```

See [`COMMAND_REFERENCE.md`](COMMAND_REFERENCE.md).
