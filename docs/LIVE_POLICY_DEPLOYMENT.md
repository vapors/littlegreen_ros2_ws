# Live Policy Deployment

This page covers the guarded transition from a paired Track 1 export to a live LittleGreen hardware policy. Servo, IMU, and shadow commissioning must already pass.

Live deployment is a staged sequence. Stop between stages and review the result before continuing. v2.8.0 supports 45-D and 47-D observations, but the packaged default remains the 45-D v1.4.5s3 policy; no deployable v1.4.7 pair is included.

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

The packaged Track 1 v1.4.5s3 bundle uses:

```text
Task:      Velocity-Lilgreen-Stand-ST3215-Loaded-v5s3
Interface: observation[45] -> action[12]
Rate:      50 Hz
Contract:  action_contract_version: 4
Transform: bounded_default_centered_vector_residual
Profile:   v1_4_5_stabilized_vector_residual
```

Contract v4 applies a per-joint residual vector:

```text
bounded_action[i] = clip(raw_action[i], -1, 1)
nominal_target[i] = q_default[i] + residual_scale_rad[i] * bounded_action[i]
q_target[i]       = clip(nominal_target[i], physical_lower[i], physical_upper[i])
```

The previous-action observation stores the bounded normalized action, not the resulting position target.

The policy node also retains compatibility with action contract v3. Current v1.4.5s3 deployment must use v4; do not convert it to a scalar or uniform residual scale.

### v2.8.0 phase-guided compatibility

A future Track 1 v1.4.7 bundle may use `observation[47] -> action[12]`. It retains action contract v4 and appends phase sine/cosine after the previous-action block. The runtime requires the exact metadata and lifecycle defined in [`OBSERVATION_CONTRACT.md`](OBSERVATION_CONTRACT.md).

Do not edit the packaged 45-D YAML to say 47. The ONNX input tensor must actually be `[1,47]`.

## 3. Required paired bundle

Deploy these together:

```text
src/littlegreen_biped_pkg/src/configs/policy_latest.yaml
src/littlegreen_biped_pkg/src/configs/policy.onnx
```

Every deployable YAML must identify its observation count and retain the v4 action fields. The existing 45-D pair may use the legacy observation compatibility path. A 47-D pair must additionally include the explicit phase metadata from `OBSERVATION_CONTRACT.md`.

```yaml
num_observations: 45  # or 47 with explicit phase metadata
num_actions: 12
action_contract_version: 4
action_transform: bounded_default_centered_vector_residual
action_residual_scale_rad: [12 per-joint values]
action_default_rad: [12 values]
action_target_lower_rad: [12 values]
action_target_upper_rad: [12 values]
action_nominal_residual_lower_rad: [12 values]
action_nominal_residual_upper_rad: [12 values]
action_indices: [12 values]
previous_action_observation: bounded_normalized_action
deployment_contract_profile: v1_4_5_stabilized_vector_residual
deployment_requires_action_contract_v4_transform: true
policy_sha256: <sha256 of policy.onnx>
```

Before loading the ONNX session, the node validates:

- ONNX SHA-256 and the actual float32 `[1,45] -> [1,12]` or `[1,47] -> [1,12]` tensor interface;
- the exact supported observation layout and, for 47-D, phase period, encoding, append order, and training semantics;
- action indices and selected simulation joint names;
- exported defaults against `joint_map.yaml`;
- exported physical lower/upper bounds against `joint_map.yaml`;
- normalized action limits `[-1, 1]`;
- positive, non-uniform v4 residual scales;
- nominal residual bounds recomputed from defaults, scales, and physical limits;
- `previous_action_observation: bounded_normalized_action`;
- the required v4 transform flag and deployment profile.

Any mismatch is fatal. Do not bypass validation by editing a single ROS-side field.

## 4. Install and audit a Track 1 export

Back up the current pair:

```bash
cd ~/littlegreen_ros2_ws

cp src/littlegreen_biped_pkg/src/configs/policy_latest.yaml \
  src/littlegreen_biped_pkg/src/configs/policy_latest.yaml.previous

cp src/littlegreen_biped_pkg/src/configs/policy.onnx \
  src/littlegreen_biped_pkg/src/configs/policy.onnx.previous
```

Copy the new export:

```bash
cp /path/to/exported/policy.yaml \
  src/littlegreen_biped_pkg/src/configs/policy_latest.yaml

cp /path/to/exported/policy.onnx \
  src/littlegreen_biped_pkg/src/configs/policy.onnx
```

Before building, the source script can validate YAML, checksum, and hardware-map semantics. Tensor-shape inspection is unavailable until the C++ probe is built, so the skip flag is for source development only:

```bash
python3 src/littlegreen_biped_pkg/scripts/policy_bundle_audit.py \
  --policy-yaml src/littlegreen_biped_pkg/src/configs/policy_latest.yaml \
  --onnx src/littlegreen_biped_pkg/src/configs/policy.onnx \
  --joint-map src/littlegreen_biped_pkg/src/configs/joint_map.yaml \
  --skip-onnx-shape-check
```

After installation, run the deployment-acceptance audit without skipping shape inspection:

```bash
ros2 run littlegreen_biped_pkg policy_bundle_audit
```

The installed audit automatically invokes `policy_onnx_contract_probe`. A successful audit exits `0`. A contract or tensor mismatch exits `2`; malformed configuration exits `5`.

For a genuine unannotated v1.4.7 47-D export, first create a separate YAML:

```bash
ros2 run littlegreen_biped_pkg annotate_phase_guided_policy \
  --policy-yaml /path/to/exported/policy.yaml \
  --output /path/to/exported/policy.phase_guided.yaml
```

The annotation tool never changes ONNX bytes or `policy_sha256` and refuses 45-D exports.

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

For a 47-D phase-guided bundle also verify:

```bash
ros2 topic echo /policy_debug/observation --once
ros2 topic echo /policy_debug/gait_phase --once
```

The first successful inference after startup begins at approximately `[sin, cos] = [0, 1]`; the clock wraps every 36 successful policy ticks. A readiness outage freezes phase rather than advancing it. The debug half-cycle is expected policy timing, not measured foot contact.

In shadow mode only, an explicit test reset is available:

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
