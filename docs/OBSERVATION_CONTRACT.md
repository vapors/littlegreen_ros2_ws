# Policy Observation Contract

LittleGreen ROS 2 v2.8.0 validates the policy observation contract independently from the action contract. The runtime supports exactly two observation layouts.

## 1. Legacy 45-D hardware observation

The packaged v1.4.5s3 policy continues to use:

```text
obs[0:3]    commanded [vx, vy, yaw_rate]
obs[3:6]    base angular velocity in the policy/base frame
obs[6:9]    projected gravity in the policy/base frame
obs[9:21]   12 actionable joint positions relative to q_default
obs[21:33]  12 actionable joint velocities
obs[33:45]  previous bounded normalized action
```

Old 45-D bundles that predate explicit observation metadata remain readable. New 45-D exports should declare:

```yaml
observation_contract_version: 1
observation_contract_name: littlegreen_hardware_45_v1
gait_phase_enabled: false
```

The 45-D compatibility path does not synthesize, append, or infer a gait phase.

## 2. Phase-guided 47-D hardware observation

A phase-guided policy uses the same first 45 values and appends:

```text
obs[45]  sin(2*pi*phase)
obs[46]  cos(2*pi*phase)
```

The required exported metadata is:

```yaml
num_observations: 47
num_actions: 12
policy_dt: 0.02

observation_contract_version: 2
observation_contract_name: littlegreen_hardware_phase_guided_47_v1
observation_layout:
  - command_velocity_3
  - base_angular_velocity_3
  - projected_gravity_3
  - joint_position_relative_to_default_12
  - joint_velocity_12
  - previous_bounded_normalized_action_12
  - gait_phase_sin_cos_2

gait_phase_enabled: true
gait_phase_period_s: 0.72
gait_phase_encoding: sin_cos_2pi
gait_phase_append_order: after_previous_action
gait_phase_training_timebase: episode_step_time
gait_phase_training_reset_semantics: environment_episode_reset
```

The supported 47-D contract also requires `action_contract_version: 4`. The observation change does not create action-contract v5.

## 3. Deterministic deployment gait clock

The v1.4.7 period is 0.72 seconds and the policy period is 0.02 seconds, producing exactly 36 policy ticks per gait cycle.

The runtime uses an integer logical tick clock:

```text
wrapped_tick = phase_tick % 36
phase        = wrapped_tick / 36
```

This avoids accumulated floating-point integration drift.

Lifecycle rules:

- The first successful inference after node startup uses phase zero: `[sin, cos] = [0, 1]`.
- The phase advances only after a complete successful inference, action transformation, and output-path update.
- Missing or stale IMU, joint state, or hardware feedback freezes phase because inference is gated.
- ONNX or post-processing errors freeze phase.
- A zero command velocity does not stop the clock; Track 1 used episode time rather than command-dependent time.
- Shadow and live modes use the same observation builder and clock.
- Node restart begins a new deployment episode and resets phase to zero.
- Phase is never persisted across process restarts.

Expected phase convention:

```text
phase [0.0, 0.5): expected left stance / right swing
phase [0.5, 1.0): expected right stance / left swing
```

These are policy expectations, not measured foot contacts. Brief double-support behavior near the transitions cannot be confirmed from the current runtime topics.

## 4. Debug interface

For 47-D policies:

```text
/policy_debug/observation
/policy_debug/gait_phase
```

`/policy_debug/gait_phase` is a `Float64MultiArray` containing:

```text
[phase, tick, period_ticks, sin, cos, expected_half_cycle]
```

`expected_half_cycle` is `0` for the left-stance half and `1` for the right-stance half.

The exact phase values supplied to ONNX remain visible at observation indices 45 and 46.

## 5. Explicit reset service

A phase-guided policy exposes:

```bash
ros2 service call \
  /policy/reset_gait_phase \
  std_srvs/srv/Trigger '{}'
```

The service is allowed in `shadow` and `disabled` modes. It is refused in `live` mode because an asynchronous live reset can create an abrupt gait discontinuity. To reset a live deployment, stop and restart the guarded live policy process while the robot is supported.

## 6. Bundle validation

The installed audit verifies:

- supported 45-D or 47-D metadata;
- the exact 47-D append order and phase settings;
- action-contract v3/v4 semantics;
- joint order, defaults, residual scales, nominal bounds, and physical limits;
- ONNX SHA-256;
- actual ONNX tensor types and dimensions through `policy_onnx_contract_probe`.

Run:

```bash
ros2 run littlegreen_biped_pkg policy_bundle_audit
```

A YAML declaring 47 observations with a `[1,45]` ONNX input is rejected. A 45-D ONNX must never be renamed or relabeled as a phase-guided policy.

For a genuine Track 1 v1.4.7 export that already reports 47 observations, the metadata helper can create a separate annotated YAML without modifying the ONNX or checksum:

```bash
ros2 run littlegreen_biped_pkg annotate_phase_guided_policy \
  --policy-yaml /path/to/exported/policy.yaml \
  --output /path/to/exported/policy.phase_guided.yaml
```

The helper refuses 45-D policies, non-v4 actions, non-50-Hz timing, missing checksums, and tasks other than `Velocity-Lilgreen-Hardware-ST3215-Loaded-v7`.

## 7. Current packaged artifact

v2.8.0 intentionally retains the known-good packaged policy:

```text
Velocity-Lilgreen-Stand-ST3215-Loaded-v5s3
observation[45] -> action[12]
```

No deployable v1.4.7 ONNX/YAML pair is included in this release.
