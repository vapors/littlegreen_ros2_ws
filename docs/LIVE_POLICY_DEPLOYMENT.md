# Live Policy Deployment

This page covers the guarded transition from a paired Track 1 export to a live LittleGreen hardware policy. Servo, IMU, and shadow commissioning must already pass.

Live deployment is a staged sequence. Stop between stages and review the result before continuing. v2.9.0 packages the complete LittleGreen Humanoid Lite v2.3.1 canonical Stand bundle and must consume its exported YAML unchanged.

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

## 2. Current packaged Track 1 policy contract

```text
Task:                 Velocity-Lilgreen-Stand-ST3215-Loaded-v23
Task role:            stand
Interface:            observation[47] -> action[12]
Rate:                 50 Hz
Observation contract: littlegreen_velocity_47d_phase_v1
Phase mode:           randomized_static_per_episode
Action contract:      4
Transform:            bounded_default_centered_vector_residual
```

The shared actor layout is fixed. The Stand phase is sampled uniformly once during intentional policy-episode startup and remains unchanged. It does not advance with time, command, contact, inference count, or transient readiness loss.

Action contract v4 remains:

```text
bounded_action[i] = clip(raw_action[i], -1, 1)
nominal_target[i] = q_default[i] + residual_scale_rad[i] * bounded_action[i]
q_target[i]       = clip(nominal_target[i], physical_lower[i], physical_upper[i])
previous_action_observation[i] = bounded_action[i]
```

Future Walk uses a different phase mode. Live Walk is blocked until its exported checkpoint explicitly pins the intended deployment stage or exact period.

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
- `randomized_static_per_episode` Stand semantics;
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

A successful audit exits `0`. A contract/tensor mismatch exits `2`; malformed configuration exits `5`. The old metadata annotation helper is a v2.8 legacy migration tool and must not be used on a v2.3.1 bundle.

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

For the v2.3.1 shared 47-D Stand bundle verify:

```bash
ros2 topic echo /policy_debug/observation --once
ros2 topic echo /policy_debug/gait_phase --once
```

The sampled Stand phase/sine/cosine must remain constant for the entire episode. A readiness outage must not resample it. The successful-tick counter may increase, but it does not evolve Stand phase.

In shadow mode, an explicit intentional new-episode reset is available:

```bash
ros2 service call \
  /policy/reset_gait_phase \
  std_srvs/srv/Trigger '{}'
```

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

Verify the command chain before releasing any driver pose override:

```bash
ros2 node list
ros2 topic info /desired_position --verbose
ros2 topic info /servo_target_radians --verbose
ros2 topic echo /policy_status --once
ros2 topic echo /safe_joint_targets --once
```

When the policy and controller publisher are confirmed intentional, release the driver override:

```bash
ros2 service call \
  /st3215_driver/release_pose_override \
  std_srvs/srv/Trigger '{}'
```

The release is immediate; an active `/servo_target_radians` publisher becomes authoritative at once.

For a 47-D live policy, `/policy/reset_gait_phase` is intentionally refused. Stop and restart the guarded live policy while supported to begin a new phase-zero deployment episode.

First live runs use:

```text
controller_mode=safety_only
override_imu=false
zero command velocity
short run duration
mechanical fall arrest
physical power disconnect immediately accessible
```

Do not use `outer_pd` or `outer_pid` during the initial v1.4.5s3 campaign.


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
