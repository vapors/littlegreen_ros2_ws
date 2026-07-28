# Policy Observation Contract

LittleGreen ROS 2 v2.9.0 validates observation and action contracts independently. The active packaged policy is the LittleGreen Humanoid Lite v2.3.1 canonical Stand export using the shared 47-D Stand/Walk actor observation and action contract v4.

## Shared 47-D actor observation

```text
obs[0:3]    commanded [vx, vy, yaw_rate]
obs[3:6]    base angular velocity in the policy/base frame
obs[6:9]    projected gravity in the policy/base frame
obs[9:21]   12 actionable joint positions relative to q_default
obs[21:33]  12 actionable joint velocities
obs[33:45]  previous bounded normalized action
obs[45]     sin(2*pi*phase)
obs[46]     cos(2*pi*phase)
```

The actor input is always 47 values for the v2.3.1 Stand and Walk tasks. The critic's 50-value training input is not a deployment interface.

Required exported metadata includes:

```yaml
schema_version: 2
num_observations: 47
num_actions: 12
observation_contract_version: 1
observation_contract_name: littlegreen_velocity_47d_phase_v1
observation_layout: command3,base_ang_vel3,projected_gravity3,joint_pos_rel12,joint_vel12,previous_bounded_action12,phase_sin1,phase_cos1
phase_indices: [45, 46]
phase_encoding: sin_cos_2pi
shared_47d_stand_walk_contract: true
legacy_45d_checkpoint_support: false
policy_dt: 0.02
```

The runtime verifies the compact layout and every explicit range in `observation_layout_ranges`. It does not rewrite or annotate the exported YAML.

## Stand phase: randomized static per episode

The packaged Stand bundle declares:

```yaml
phase_mode: randomized_static_per_episode
phase_reset_semantics: sample_uniform_once_for_each_reset_environment_and_hold
```

At the start of a deployment episode:

```text
phase ~ Uniform[0,1)
phase_sin = sin(2*pi*phase)
phase_cos = cos(2*pi*phase)
```

The pair is then immutable for the episode.

- It does not advance with time or successful inference count.
- It does not depend on command velocity, contact, COM, or reward state.
- It is sampled before the node has its first complete sensor snapshot.
- A transient readiness loss does not resample it.
- Command timeout or a zero command does not change it.
- An inference exception does not change it.
- Node startup begins a new episode.
- An intentional shadow/disabled reset begins a new episode.
- The reset service is refused in live mode.

`phase_period_s` is carried by the shared export but is not used to evolve Stand phase.

## Deterministic test injection

Production Stand deployment samples uniformly. Tests and replay may inject a fixed phase or deterministic seed:

```bash
ros2 launch littlegreen_biped_pkg policy_shadow.launch.py \
  enable_phase_test_override:=true \
  phase_test_fixed_value:=0.25
```

or:

```bash
ros2 launch littlegreen_biped_pkg policy_shadow.launch.py \
  enable_phase_test_override:=true \
  phase_test_seed:=42
```

The override is rejected in live mode. It is not a deployment tuning parameter.

Cardinal fixed phases produce:

```text
0.00 -> [ 0,  1]
0.25 -> [ 1,  0]
0.50 -> [ 0, -1]
0.75 -> [-1,  0]
```

## Future Walk phase

The runtime architecture supports `command_synchronized_continuous_nonblocking` for tests and shadow inspection.

Moving is defined by exported thresholds:

```text
norm([vx, vy]) > 0.20 m/s OR abs(yaw_rate) > 0.08 rad/s
```

When not moving, phase is zero and unfinished onset state is cleared. At each new movement onset, the preferred first swing alternates:

```text
left first swing  -> phase 0.5
right first swing -> phase 0.0
```

After the onset tick:

```text
phase = (phase + policy_dt / phase_period_s) mod 1.0
```

No touchdown, contact, COM, or reward-state signal gates the clock.

### Walk deployment-stage gate

Track 1 Walk curricula use stage-specific periods. Live Walk is blocked unless the exported bundle explicitly pins the checkpoint's intended deployment stage or exact period. A fresh exporter environment must not silently label a later checkpoint with Stage-0 timing.

## Action contract v4

The observation change does not alter the action contract:

```text
bounded_action = clip(raw_action, -1, +1)
q_target = clip(
    q_default + bounded_action * action_residual_scale_rad,
    physical_lower,
    physical_upper,
)
previous_action_observation = bounded_action
```

The previous action is neither the unclipped network output nor the physical target.

## Debug interface

For the shared 47-D contract:

```bash
ros2 topic echo /policy_debug/observation
ros2 topic echo /policy_debug/gait_phase
```

`/policy_debug/gait_phase` contains:

```text
[0] phase fraction
[1] policy episode number
[2] successful inference tick count
[3] sin(2*pi*phase)
[4] cos(2*pi*phase)
[5] phase mode code: 1=Stand, 2=Walk, 3=v2.8 legacy compatibility
[6] moving flag
[7] first-swing-left flag
```

For Stand, fields 0, 3, and 4 remain constant for the episode while the successful tick count may increase.

An intentional shadow reset is:

```bash
ros2 service call \
  /policy/reset_gait_phase \
  std_srvs/srv/Trigger '{}'
```

The service begins a new policy episode. It does not command a servo, release a driver pose override, alter torque, or reset physical feedback.

## Bundle and tensor verification

```bash
ros2 run littlegreen_biped_pkg policy_bundle_audit
```

The audit verifies the five-file bundle, checksums, canonical action map, and actual ONNX interface:

```text
obs float32 [1,47] -> actions float32 [1,12]
```

A YAML/ONNX shape mismatch is rejected even when filenames appear correct.

## Legacy v2.8.0 compatibility

The former 45-D policy and older 47-D successful-tick clock are retained only as isolated compatibility paths. The prior paired 45-D artifact is stored under `configs/legacy_v280_45d/`. Neither legacy path may reinterpret the active v2.3.1 export.
