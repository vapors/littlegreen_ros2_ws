# Live Policy Deployment

This page covers the guarded transition from a Track 1 deployment bundle to a live LittleGreen hardware policy. It assumes that servo, IMU, and shadow-mode commissioning already pass.

Live deployment is not a single launch command. Treat it as a staged sequence with an explicit stop point between each stage.

## 1. Runtime data path

```text
/command_velocity
/imu/data
/joint_states
/joint_feedback_age_ms
        │
        ▼
littlegreen_biped_node
        │  /desired_position
        ▼
pd_controller_node
  controller_mode=safety_only
        │  /servo_target_radians
        ▼
lgh_st3215_driver
        │  /dev/ttyS3 @ 1 Mbps
        ▼
12 × ST3215 servos
```

`littlegreen_biped_node` owns observation construction, ONNX inference, action-contract transformation, and policy target generation. `pd_controller_node` owns the downstream safety envelope and command shaping. `lgh_st3215_driver` remains the only normal runtime owner of the servo UART.

## 2. Required deployment bundle

Deploy the YAML and ONNX model as a pair:

```text
src/littlegreen_biped_pkg/src/configs/policy_latest.yaml
src/littlegreen_biped_pkg/src/configs/policy.onnx
```

The YAML should resolve the model with:

```yaml
policy_checkpoint_relative_path: policy.onnx
policy_sha256: <sha256 of policy.onnx>
```

For action contract v3, the policy YAML must contain:

```yaml
action_contract_version: 3
action_transform: bounded_default_centered_symmetric_residual
action_limit_lower: -1.0
action_limit_upper: 1.0
action_residual_scale_rad: [12 values]
action_default_rad: [12 values]
action_target_lower_rad: [12 values]
action_target_upper_rad: [12 values]
action_indices: [12 values]
previous_action_observation: bounded_normalized_action
```

At startup, the policy node verifies:

- the ONNX SHA-256 against `policy_sha256`;
- ONNX input and output dimensions against `45 → 12`;
- `action_indices` against the canonical `sim_joint_index` values in `joint_map.yaml`;
- exported action joint names against the canonical joint names;
- `action_default_rad` and selected `default_joint_positions` against `default_joint_rad`;
- `action_target_lower_rad` and `action_target_upper_rad` against the hardware bounds in `joint_map.yaml`;
- normalized action bounds of `[-1, 1]`;
- positive residual scales;
- `previous_action_observation: bounded_normalized_action`.

Any mismatch is fatal. Do not bypass the check by editing a single field in isolation.

## 3. Install a new policy pair

Keep a backup of the currently installed source pair:

```bash
cd ~/littlegreen_ros2_ws

cp src/littlegreen_biped_pkg/src/configs/policy_latest.yaml \
  src/littlegreen_biped_pkg/src/configs/policy_latest.yaml.previous

cp src/littlegreen_biped_pkg/src/configs/policy.onnx \
  src/littlegreen_biped_pkg/src/configs/policy.onnx.previous
```

Copy the Track 1 export:

```bash
cp /path/to/exported/policy.yaml \
  src/littlegreen_biped_pkg/src/configs/policy_latest.yaml

cp /path/to/exported/policy.onnx \
  src/littlegreen_biped_pkg/src/configs/policy.onnx
```

Confirm the checksum before rebuilding:

```bash
cd ~/littlegreen_ros2_ws

EXPECTED_SHA="$(python3 - <<'PY'
import yaml
with open('src/littlegreen_biped_pkg/src/configs/policy_latest.yaml') as f:
    print(yaml.safe_load(f)['policy_sha256'])
PY
)"

ACTUAL_SHA="$(sha256sum src/littlegreen_biped_pkg/src/configs/policy.onnx | awk '{print $1}')"

printf 'expected: %s\nactual:   %s\n' "$EXPECTED_SHA" "$ACTUAL_SHA"
test "$EXPECTED_SHA" = "$ACTUAL_SHA"
```

Rebuild only the policy package:

```bash
cd ~/littlegreen_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

colcon build \
  --symlink-install \
  --packages-select littlegreen_biped_pkg \
  --event-handlers console_direct+

source install/setup.bash
```

A running node must be restarted after a policy update. The node loads the YAML and ONNX files only during startup.

## 4. Stage A — hardware feedback and IMU

Mechanically support the robot and keep servo writes disabled:

```bash
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=runtime_safe \
  enable_writes:=false
```

Start the current IMU source in another terminal. The source may be micro-ROS, direct I2C, or direct SPI, but it must publish the canonical `/imu/data` contract.

Run the runtime preflight:

```bash
ros2 run lgh_st3215_tools st3215_preflight \
  --mode runtime \
  --expect-writes false
```

Confirm the IMU boundary separately:

```bash
ros2 run lgh_imu_tools imu_preflight
```

Do not continue until both checks pass.

## 5. Stage B — policy shadow

Launch the policy without downstream command authority:

```bash
ros2 launch littlegreen_biped_pkg policy_shadow.launch.py
```

Expected startup lines include:

```text
Policy config loaded: ... action_contract=v3
Action contract v3 validated against joint_map.yaml ...
Policy artifact checksum verified ...
ONNX model loaded ...
```

Validate the graph:

```bash
ros2 topic info /desired_position --verbose
ros2 topic info /policy_shadow/desired_position --verbose
ros2 topic echo /policy_status --once
ros2 topic echo /policy_ready --once
ros2 topic hz /policy_shadow/desired_position
```

In shadow mode:

- the policy publishes `/policy_shadow/desired_position`;
- the policy does not create a publisher on `/desired_position`;
- `pd_controller_node` is not launched;
- the servo driver remains feedback-only.

Inspect the debug topics before authorizing live output:

```bash
ros2 topic echo /policy_debug/raw_action --once
ros2 topic echo /policy_debug/clipped_raw_action --once
ros2 topic echo /policy_debug/target_clipped --once
ros2 topic echo /policy_debug/saturation_mask --once
```

Stop shadow mode before proceeding.

## 6. Stage C — write-enabled driver hold

Keep the robot supported and the physical servo-power disconnect immediately accessible.

Stop the feedback-only driver, then restart it with writes enabled:

```bash
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=runtime_safe \
  enable_writes:=true
```

Before the policy stack starts, the driver uses its current-position startup hold after complete feedback becomes available.

Run preflight again:

```bash
ros2 run lgh_st3215_tools st3215_preflight \
  --mode runtime \
  --expect-writes true
```

Do not continue if feedback is stale, diagnostics are not healthy, or the driver reports an unexpected command publisher.

## 7. Stage D — live policy with safety-only shaping

Start the explicit live launch:

```bash
ros2 launch littlegreen_biped_pkg policy_live.launch.py \
  controller_mode:=safety_only
```

`policy_live.launch.py` starts only:

```text
littlegreen_biped_node
pd_controller_node
```

It does not start the servo driver, IMU source, joystick, or keyboard. Those remain separately controlled so each authority boundary is visible.

Verify the command chain:

```bash
ros2 node list
ros2 topic info /desired_position --verbose
ros2 topic info /servo_target_radians --verbose
ros2 topic echo /policy_status --once
ros2 topic echo /safe_joint_targets --once
```

The first live runs should use:

```text
controller_mode=safety_only
override_imu=false
zero command velocity
short run duration
mechanical fall arrest
```

Do not use `outer_pd` or `outer_pid` during the initial live-policy campaign.

## 8. Command sources

Without a command source, the policy node applies its configured zero-on-timeout behavior. For initial standing tests, that is preferred.

For joystick operation after zero-command standing is accepted, use the broader launch:

```bash
ros2 launch littlegreen_biped_pkg littlegreen_biped_launch.py \
  controller_mode:=safety_only \
  policy_output_mode:=live
```

That launch starts joystick input, teleop, the policy node, the command-file bridge, and `pd_controller_node`. It still does not start the ST3215 driver or IMU source.

For keyboard and joystick multiplexing:

```bash
ros2 launch littlegreen_biped_pkg biped_teleop_mux.launch.py \
  controller_mode:=safety_only \
  policy_output_mode:=live
```

The mux launch opens the keyboard teleop process through `xterm` and should be used only on a host with a graphical session.

## 9. Stop and abort behavior

Normal stop sequence:

1. stop the live policy launch;
2. call the driver hold service;
3. inspect diagnostics;
4. stop the driver or disable torque only when the robot is safely supported.

Latch the measured pose:

```bash
ros2 service call \
  /st3215_driver/hold_current_pose \
  std_srvs/srv/Trigger '{}'
```

Torque-off is not the first response while the robot is unsupported. The physical power disconnect remains the emergency action.

## 10. Launch-file roles

| Launch file | Starts | Intended use |
|---|---|---|
| `policy_shadow.launch.py` | policy node only, shadow output | first real-sensor policy evaluation |
| `policy_live.launch.py` | policy node + `pd_controller_node` | guarded live policy with an externally managed driver and IMU |
| `littlegreen_biped_launch.py` | joystick + teleop + policy + bridge + PD | live command operation after standing acceptance |
| `biped_teleop_mux.launch.py` | joystick + keyboard + mux + policy + bridge + PD | desktop teleoperation after standing acceptance |
| `lgh_st3215_driver.launch.py` | ST3215 runtime driver | launched separately in `runtime_safe` profile |

The dedicated shadow and live launch files are the recommended deployment path because they keep hardware, sensor, and policy authority explicit.
