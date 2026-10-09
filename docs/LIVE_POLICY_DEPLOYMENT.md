# Live Policy Deployment

> **v2.9.4 locomotion note:** live locomotion no longer enters policy control from training `q_default`. Power-on remains a measured-current hold. When live locomotion is intentionally started, Track 2 ramps to the SHA-bound Track-1 `learned_zero_command` handoff state, seeds the exported previous-action observation, verifies IMU/joint readiness, and only then enables policy authority. See [V2_9_4_HANDOFF_CALIBRATION.md](V2_9_4_HANDOFF_CALIBRATION.md).

This page covers the guarded transition from a paired Track 1 export to a live LittleGreen hardware policy. Servo, IMU, and shadow commissioning must already pass.

Live deployment is a staged sequence. Stop between stages and review the result before continuing. v2.9.4 packages the audited LittleGreen Humanoid Lite v10.2 `model_10000` locomotion bundle as the active source policy and consumes its exported YAML unchanged.

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

The policy node owns observation construction, ONNX inference, action-contract transformation, and target generation. `pd_controller_node` owns the downstream safety envelope. `lgh_st3215_driver` remains the sole normal UART owner.

## 2. Active Track 1 policy contract

The repository source tree is synchronized to the current v10.2 locomotion deployment. Historical Stand artifacts remain rollback/reference material but are not the active clean-source fallback. The active bundle is:

```text
Task:                 Velocity-Lilgreen-Locomotion-ST3215-Loaded-v102
Task role:            locomotion
Interface:            observation[47] -> action[12]
Rate:                 50 Hz
Observation contract: littlegreen_velocity_47d_phase_v1
Phase mode:           neutral_static
Phase tail:           obs[45:47] = [0, 1]
Action contract:      v4 / v1_4_5_stabilized_vector_residual
Command envelope:     vx [-0.45,+0.65], vy [-0.30,+0.30], yaw [-0.50,+0.50]
```

`q_default` remains the protected observation/action reference. For live locomotion, it is **not** the policy-entry pose. v2.9.4 loads `policy_handoff.yaml`, which is bound to the active task and ONNX SHA. Live authority remains disabled until the robot is within the learned zero-command handoff pose tolerance and the previous-action observation has been seeded.

Action contract v4 remains:

```text
bounded_action[i] = clip(raw_action[i], -1, 1)
nominal_target[i] = q_default[i] + residual_scale_rad[i] * bounded_action[i]
q_target[i]       = clip(nominal_target[i], physical_lower[i], physical_upper[i])
previous_action_observation[i] = bounded_action[i]
```

For first entry into the current locomotion policy, use `ros2 run littlegreen_biped_pkg policy_handoff_control pose` to validate the physical handoff posture, then `policy_handoff_control live` only after the supported-robot checks pass.

## 3. Required complete bundle

Deploy these together:

```text
src/littlegreen_biped_pkg/src/configs/policy.onnx
src/littlegreen_biped_pkg/src/configs/policy.yaml
src/littlegreen_biped_pkg/src/configs/policy_latest.yaml
src/littlegreen_biped_pkg/src/configs/deployment_contract.yaml
src/littlegreen_biped_pkg/src/configs/policy.sha256
src/littlegreen_biped_pkg/src/configs/bundle_manifest.yaml
```

`policy.yaml` and `policy_latest.yaml` are aliases of the same exported metadata and must remain byte-identical. The runtime and audit validate:

- export schema 2 and observation-contract version 1;
- exact compact 47-D layout and explicit index ranges;
- `neutral_static` locomotion semantics with `obs[45:47] = [0,1]`;
- all bundle hashes and actual float32 ONNX `[1,47] -> [1,12]` tensors;
- canonical action indices and joint names;
- exported defaults and physical bounds against `joint_map.yaml`;
- normalized action limits, non-uniform v4 scales, nominal residual bounds, and previous-action semantics.

Any mismatch is fatal. Do not edit a bundle field to make an incompatible runtime accept it.

## 4. Install and audit a Track 1 export

Back up the current complete bundle:

```bash
cd ~/littlegreen_ros2_ws
mkdir -p ~/littlegreen_policy_backup
cp -a \
  src/littlegreen_biped_pkg/src/configs/policy.onnx \
  src/littlegreen_biped_pkg/src/configs/policy.yaml \
  src/littlegreen_biped_pkg/src/configs/policy_latest.yaml \
  src/littlegreen_biped_pkg/src/configs/deployment_contract.yaml \
  src/littlegreen_biped_pkg/src/configs/policy.sha256 \
  src/littlegreen_biped_pkg/src/configs/bundle_manifest.yaml \
  ~/littlegreen_policy_backup/
```

Copy the exported bundle without rewriting its YAML:

```bash
cp /path/to/exported/policy.onnx \
  src/littlegreen_biped_pkg/src/configs/policy.onnx
cp /path/to/exported/policy.yaml \
  src/littlegreen_biped_pkg/src/configs/policy.yaml
cp /path/to/exported/policy.yaml \
  src/littlegreen_biped_pkg/src/configs/policy_latest.yaml
cp /path/to/exported/deployment_contract.yaml \
  src/littlegreen_biped_pkg/src/configs/deployment_contract.yaml
cp /path/to/exported/policy.sha256 \
  src/littlegreen_biped_pkg/src/configs/policy.sha256
cp /path/to/exported/bundle_manifest.yaml \
  src/littlegreen_biped_pkg/src/configs/bundle_manifest.yaml
```

Run the source audit directly:

```bash
python3 src/littlegreen_biped_pkg/scripts/policy_bundle_audit.py \
  --policy-yaml src/littlegreen_biped_pkg/src/configs/policy_latest.yaml \
  --onnx src/littlegreen_biped_pkg/src/configs/policy.onnx \
  --joint-map src/littlegreen_biped_pkg/src/configs/joint_map.yaml \
  --deployment-contract src/littlegreen_biped_pkg/src/configs/deployment_contract.yaml \
  --policy-sha256-file src/littlegreen_biped_pkg/src/configs/policy.sha256 \
  --bundle-manifest src/littlegreen_biped_pkg/src/configs/bundle_manifest.yaml
```

After installation:

```bash
ros2 run littlegreen_biped_pkg policy_bundle_audit
```

A successful audit exits `0`. A contract/tensor mismatch exits `2`; malformed configuration exits `5`. The old metadata annotation helper is a v2.8 legacy migration tool and must not be used on the current v10.2 bundle.

## 5. Rebuild and restart

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

Restart every running policy node after a policy update. The YAML and ONNX model are loaded only at startup.

## 6. Stage A — feedback-only hardware and IMU

Mechanically support the robot and keep writes disabled:

```bash
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=runtime_safe \
  enable_writes:=false
```

Start the current micro-ROS IMU source in a separate terminal and keep it running:

```bash
ros2 run micro_ros_agent micro_ros_agent serial \
  --dev /dev/ttyACM0 \
  -b 115200 \
  -v0
```

If the device number changed, inspect `/dev/ttyACM*` and `/dev/serial/by-id/` before changing `--dev`. A future direct I2C/SPI source may replace the agent, but it must publish the same canonical `/imu/data` contract.

Run both preflights:

```bash
ros2 run lgh_st3215_tools st3215_preflight \
  --mode runtime \
  --expect-writes false

ros2 topic hz /imu/data
ros2 run lgh_imu_tools imu_preflight
```

After a sensor, mount, transport, or driver change, also run:

```bash
ros2 run lgh_imu_tools stationary_characterization --duration-sec 10
ros2 run lgh_imu_tools orientation_audit --pose neutral
```

Do not continue until servo and IMU checks pass.

## 7. Stage B — policy shadow

```bash
ros2 launch littlegreen_biped_pkg policy_shadow.launch.py
```

Expected startup lines include:

```text
Policy config loaded: num_observations=..., observation_contract=..., action_contract=v4, profile=...
Action contract v4 validated against joint_map.yaml ... nominal residual bounds match.
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
- the policy creates no publisher on `/desired_position`;
- `pd_controller_node` is not launched;
- the driver remains feedback-only.

Inspect policy post-processing:

```bash
ros2 topic echo /policy_debug/raw_action --once
ros2 topic echo /policy_debug/clipped_raw_action --once
ros2 topic echo /policy_debug/target_unclipped --once
ros2 topic echo /policy_debug/target_clipped --once
ros2 topic echo /policy_debug/saturation_mask --once
```

For the current v10.2 shared 47-D locomotion bundle verify:

```bash
ros2 topic echo /policy_debug/observation --once
ros2 topic echo /policy_debug/gait_phase --once
```

The observation tail must remain exactly `obs[45:47] = [0.0, 1.0]`. `neutral_static` does not advance with time, command, contact, or inference count. The live runtime intentionally refuses gait-phase resets for this contract.

Capture Track 1-aligned real-hardware metrics:

```bash
ros2 run littlegreen_biped_pkg policy_runtime_metrics \
  --duration-sec 30
```

See [`TRACK1_TRACK2_POLICY_METRICS.md`](TRACK1_TRACK2_POLICY_METRICS.md) for interpretation and current observability limits.

Stop shadow mode before proceeding.

## 8. Stage C — write-enabled driver hold

Keep the robot supported and the physical servo-power disconnect immediately accessible. Stop shadow mode and the feedback-only driver first. Confirm the previous command nodes are gone:

```bash
ros2 node list
ros2 topic info /servo_target_radians --verbose
```

Restart the driver with writes enabled:

```bash
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=runtime_safe \
  enable_writes:=true
```

Run preflight again:

```bash
ros2 run lgh_st3215_tools st3215_preflight \
  --mode runtime \
  --expect-writes true
```

For a deliberate current-pose hold before starting the live publisher:

```bash
ros2 service call \
  /st3215_driver/hold_current_pose \
  std_srvs/srv/Trigger '{}'
```

Do not continue if feedback is stale, diagnostics are unhealthy, the pose override is unexpected, or the command graph contains an unrecognized publisher. See [`ROS_GRAPH_AND_AUTHORITY.md`](ROS_GRAPH_AND_AUTHORITY.md).

## 9. Stage D — live policy with safety-only shaping

```bash
ros2 launch littlegreen_biped_pkg policy_live.launch.py \
  controller_mode:=safety_only
```

The launch starts only:

```text
littlegreen_biped_node
pd_controller_node
```

It does not start the driver, IMU source, joystick, or keyboard.

Verify the command chain before any live authority transition:

```bash
ros2 node list
ros2 topic info /desired_position --verbose
ros2 topic info /servo_target_radians --verbose
ros2 topic echo /policy_status --once
ros2 topic echo /safe_joint_targets --once
```

For a v2.9.4 **locomotion** bundle, do **not** manually release the driver override. Live authority starts disabled and the zero-command handoff must coordinate the release:

```bash
ros2 run littlegreen_biped_pkg policy_handoff_control live
```

This ramps to the SHA-bound Track-1 learned zero-command pose, waits for readiness, seeds `obs[33:45]`, releases the driver override, and then enables policy authority. If the final enable fails, the helper requests `hold_current_pose`.

The manual `/st3215_driver/release_pose_override` service remains available for commissioning and non-handoff workflows, but bypassing the v2.9.4 locomotion handoff is not the deployment path.

For 47-D live locomotion, `/policy/reset_gait_phase` remains intentionally refused. The current v10.2 contract uses `neutral_static`, so its phase tail is fixed at `[0,1]`.

First live runs use:

```text
controller_mode=safety_only
override_imu=false
zero command velocity
short run duration
mechanical fall arrest
physical power disconnect immediately accessible
```

Do not use `outer_pd` or `outer_pid` during the initial v10.2 sim-to-real campaign.


## Recommended terminal layout

```text
Terminal A: micro-ROS agent (`/dev/ttyACM0`, `/imu/data`)
Terminal B: `lgh_st3215_driver`
Terminal C: policy shadow or policy live launch
Terminal D: joystick/command source, when enabled
Terminal E: preflight, diagnostics, and authority inspection
```

The live launch does not start or stop the micro-ROS agent or ST3215 driver. Treat each terminal as an independent process that must be stopped and verified separately.

## 10. Command sources

Without a command source, timeout handling supplies a zero command. That is preferred for initial standing tests.

After zero-command standing is accepted, the broader joystick launch is:

```bash
ros2 launch littlegreen_biped_pkg littlegreen_biped_launch.py \
  controller_mode:=safety_only \
  policy_output_mode:=live
```

It starts joystick input, teleop, the policy node, command bridge, and `pd_controller_node`; it still does not start the ST3215 driver or IMU source.

## 11. Stop and hold

Normal stop sequence:

1. stop the live policy launch;
2. request the driver current-pose hold if needed;
3. verify the policy publisher has disappeared;
4. remove servo power when physical intervention is required.

```bash
ros2 service call /st3215_driver/hold_current_pose std_srvs/srv/Trigger '{}'
ros2 topic info /desired_position --verbose
```

The software hold is not an electrical emergency stop.

## 12. Contract-safe posture changes

The v4 target is centered on the exported `q_default`. Do not alter `joint_map.yaml`, `servo_map.yaml`, or the controller defaults to cosmetically change the policy posture.

A Track 1 posture or height change must follow this sequence:

```text
Track 1 task/default update
  -> train or fine-tune
  -> export paired YAML + ONNX
  -> offline bundle audit
  -> shadow validation
  -> guarded live deployment
```

Servo center changes are reserved for correcting a measured physical-to-model zero error, not for changing the learned standing pose.
