# LittleGreen ROS 2 v2.9.3 — Track-1 learned zero-command policy handoff

## Why this revision exists

Track-1 v10.2 model_10000 has a stable learned zero-command standing region that differs from the protected training `q_default`. The physical robot also leans backward when placed at `q_default`. v2.9.3 therefore separates three concepts that were previously easy to conflate:

```text
model zero / calibration center
        !=
training q_default
        !=
learned zero-command policy-entry pose
```

`q_default` is still used exactly by the 47-D observation and action-contract-v4 transform. It is **not** rewritten. The Track-1 learned zero-command pose is instead used as the default pre-position target whenever live locomotion policy authority is intentionally entered.

The robot still does **not move automatically at power-on**. The ST3215 driver starts by holding the measured current position when writes are enabled. A policy-entry ramp remains an explicit guarded action.

## v10.2 model_10000 handoff identity

The packaged handoff profile is bound to:

```text
task:          Velocity-Lilgreen-Locomotion-ST3215-Loaded-v102
policy_sha256: da7adcaf996809bfb7a413bfec712dd2f3bd2ed984f8f661af2f9cc9c4d7872e
mode:          learned_zero_command
phase:         [0, 1]
command:       [0, 0, 0]
```

It was derived from analyzer schema 2 using late, viable, bilateral main+toe full-plant samples from model_10000. The extracted previous-action observation was cross-checked directly against the action term with zero error.

One source median — right hip yaw — was 1.94e-05 rad beyond the physical lower limit. The Track-2 handoff profile clamps only that value to the exact physical limit. This is far below one ST3215 encoder step and does not alter the learned pose materially.

## Handoff state

The effective joint target is:

```text
L hip roll      -0.0022942114
L hip yaw       +0.0143458154
L hip pitch     -0.3477285504
L knee          +0.8788803220
L ankle pitch   -0.4449054599
L ankle roll    +0.0664796531
R hip roll      -0.0303205941
R hip yaw       -0.0567572892   # physical lower limit clamp
R hip pitch     -0.2813382149
R knee          +0.8014949560
R ankle pitch   -0.4355151951
R ankle roll    +0.0509283803
```

The first live policy observation is seeded with:

```text
obs[33:45] =
[-0.1871184260, +0.9505239725, -0.5016342402, -0.0525087193,
 -0.0837832540, +0.2786795497, -0.2837223411, -0.9598009586,
 -0.2914022803, -0.1211327016, +0.2477010041, +0.0395602249]
```

These are bounded normalized previous actions. They are **not radians** and must never be sent directly to a servo.

## Live authority state machine

For live locomotion, the policy node now starts with policy authority disabled. No `/desired_position` is published until the explicit handoff completes.

```text
current measured pose
   |
   | guarded driver ramp
   v
Track-1 learned zero-command handoff pose
   |  driver pose override still active
   |
   | /policy/arm_handoff
   |   - fresh IMU + joint feedback
   |   - near handoff pose
   |   - low joint/base velocity
   |   - zero effective command
   |   - acceptable body tilt
   |   - seed obs[33:45]
   v
handoff armed / policy output still blocked
   |
   | release driver pose override
   |
   | /policy/enable_authority
   v
first live ONNX inference uses the Track-1 previous-action seed
```

Shadow mode is unchanged and does not require this authority gate.

## IMU discontinuity gate

Hardware characterization showed an intermittent orientation-estimator event: after a roughly 110 ms IMU transport gap, the quaternion briefly became identity while the stationary gyro remained near zero, then reconverged over about a second.

v2.9.3 adds:

```text
startup stability hold          1.0 s
post-discontinuity recovery     1.5 s
transport-gap trigger           > 0.080 s
orientation-jump trigger        > 0.12 rad with low gyro
stationary gyro threshold       < 0.35 rad/s
```

A reset-like discontinuity inhibits inference. In live locomotion it also latches policy authority off, requiring a deliberate new handoff rather than silently resuming locomotion from an unknown physical state.

## First test: pose only

The first v2.9.3 hardware test should not release policy authority. Keep the robot supported, launch the write-enabled runtime-safe driver and live policy node, then run:

```bash
ros2 run littlegreen_biped_pkg policy_handoff_control pose
```

This loads the SHA-bound Track-1 pose into the driver, performs the normal guarded smooth ramp, and leaves the driver's internal pose override active. Measure the physical posture and IMU at this point.

Useful checks:

```bash
ros2 run lgh_imu_tools orientation_audit --pose neutral
ros2 topic echo /joint_states --once
ros2 topic echo /st3215_driver/diagnostics --once
```

Expected: the robot should be appreciably less backward-leaning than at `q_default`. Exact agreement with the simulated ~4.8 degree median tilt is not required.

## First supported closed-loop test

After the pose-only result is accepted:

```bash
ros2 run littlegreen_biped_pkg policy_handoff_control live
```

The helper performs the guarded ramp, waits for the policy handoff state to become armable, seeds the previous action, releases the driver pose override, and then enables policy authority. If authority enable fails after the override is released, the helper immediately requests `hold_current_pose`.

Keep command at zero for the first test. Only after stable supported closed-loop standing should forward command be introduced in small increments, e.g. 0.03, 0.05, 0.08, 0.10 m/s.

To stop policy authority and latch the measured pose:

```bash
ros2 run littlegreen_biped_pkg policy_handoff_control disable
```

## Manual service sequence

The same workflow can be performed manually:

```bash
ros2 service call /st3215_driver/move_to_policy_handoff_pose std_srvs/srv/Trigger '{}'
ros2 service call /policy/arm_handoff std_srvs/srv/Trigger '{}'
ros2 service call /st3215_driver/release_pose_override std_srvs/srv/Trigger '{}'
ros2 service call /policy/enable_authority std_srvs/srv/Trigger '{}'
```

For manual use, the driver `policy_handoff_pose_rad` parameter must first be populated from the active SHA-bound profile; `policy_handoff_control` does this automatically and is the preferred path.

## Installing a future Track-1 handoff extraction

Place the analyzer output `zero_command_handoff_pose.json` beside the Track-1 exported policy bundle and run:

```bash
python3 src/littlegreen_biped_pkg/scripts/install_exported_policy_bundle.py \
  --bundle-dir ~/path/to/exported \
  --dry-run

python3 src/littlegreen_biped_pkg/scripts/install_exported_policy_bundle.py \
  --bundle-dir ~/path/to/exported
```

The installer automatically detects the JSON, verifies task identity, q_default, canonical joint order, physical limits, bounded previous-action semantics, and analyzer cross-check, then generates `policy_handoff.yaml` bound to the installed policy SHA.

## Startup semantics

The recommended default behavior is therefore:

```text
power-on / driver start -> hold measured current physical pose
commissioning reference -> explicit move_to_default_pose (training q_default)
policy entry             -> Track-1 learned zero-command handoff pose
policy control           -> ONNX closed loop
```

This preserves hardware safety and calibration while making the policy's normal startup state inherit from Track 1.
