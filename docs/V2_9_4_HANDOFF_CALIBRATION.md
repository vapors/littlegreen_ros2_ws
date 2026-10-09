# LittleGreen ROS 2 v2.9.4 — hardware-calibrated zero-command handoff

## Purpose

v2.9.4 turns the v10.2 zero-command handoff into an auditable three-layer contract instead of embedding physical robot tuning inside a single pose vector.

The protected Track-1 quantities remain unchanged:

- policy: `Velocity-Lilgreen-Locomotion-ST3215-Loaded-v102`
- policy SHA-256: `da7adcaf996809bfb7a413bfec712dd2f3bd2ed984f8f661af2f9cc9c4d7872e`
- observation: `littlegreen_velocity_47d_phase_v1`, 47-D
- action contract: v4, 12-D
- phase mode: `neutral_static`, `[0.0, 1.0]`
- `q_default`: unchanged and still used by the observation/action transform
- previous-action seed: unchanged Track-1 median bounded action

## Pose provenance

The handoff profile stores these vectors separately in canonical 12-joint order:

```text
source_median_joint_position_rad   # immutable Track-1 analyzer result
hardware_trim_rad                  # robot-specific additive calibration
joint_position_rad                 # effective Track-2 target
physical_limit_clamps              # explicit final clamp records
```

The effective target must satisfy:

```text
joint_position_rad[i]
  = clamp_to_physical_limit(
      source_median_joint_position_rad[i] + hardware_trim_rad[i])
```

Both the offline bundle auditor and the live policy node validate this relationship before live locomotion can be armed.

## Current hardware trim

The calibrated trim is:

```text
L hip roll       0.000 rad
L hip yaw        0.000 rad
L hip pitch     -0.035 rad
L knee           0.000 rad
L ankle pitch   -0.100 rad
L ankle roll     0.000 rad
R hip roll       0.000 rad
R hip yaw        0.000 rad
R hip pitch     -0.035 rad
R knee           0.000 rad
R ankle pitch   -0.100 rad
R ankle roll     0.000 rad
```

The right hip-yaw source median is additionally clamped by `+1.9435604e-05 rad` to the captured physical lower limit. That clamp is recorded independently and is not treated as a hardware trim.

## Effective handoff target

```text
L hip roll      -0.0022942114
L hip yaw       +0.0143458154
L hip pitch     -0.3827285504
L knee          +0.8788803220
L ankle pitch   -0.5449054599
L ankle roll    +0.0664796531
R hip roll      -0.0303205941
R hip yaw       -0.0567572892
R hip pitch     -0.3163382149
R knee          +0.8014949560
R ankle pitch   -0.5355151951
R ankle roll    +0.0509283803
```

## Physical calibration evidence

At this effective handoff pose, the 2026-10-09 orientation audit reported:

```text
median projected gravity (base):
[+0.0178999640, -0.0230282932, -0.9995786694]

median RPY (deg):
[-1.02588, +1.31954, -0.20401]

orientation expectation: PASS
```

This is approximately 1.7 degrees total tilt and was visually/mechanically stable while the robot remained supported.

## Re-install behavior

`install_exported_policy_bundle.py` now preserves an existing `hardware_trim_rad` only when all of the following match:

1. policy task,
2. policy SHA-256,
3. Track-1 source median handoff pose.

Installing a different policy identity resets the hardware trim to twelve zeros and marks it uncalibrated. This prevents a robot-specific correction from silently carrying into a new learned policy.

## Source-policy synchronization

v2.9.4 also synchronizes the active v10.2 five-file deployment bundle into `src/littlegreen_biped_pkg/src/configs/` and `src/checkpoints/policy.onnx`. A normal package rebuild therefore preserves the audited v10.2 policy instead of restoring the older Stand artifact.

## Zero-command closed-loop handoff

Keep the robot mechanically supported and command fixed at `[0,0,0]`.

1. Start the IMU source and verify `/imu/data` is healthy.
2. Launch the runtime-safe ST3215 driver with writes explicitly enabled.
3. Launch `policy_live.launch.py` with `controller_mode:=safety_only`.
4. Run `policy_bundle_audit`; it must pass and report the calibrated hardware trim.
5. Run `policy_handoff_control pose`; verify the robot reaches the effective pose and remains balanced.
6. Run `orientation_audit --pose neutral` if posture has changed or hardware has been disturbed.
7. Run `policy_handoff_control live`.

The `live` stage re-runs the guarded pose ramp, validates fresh joint/IMU data, requires near-zero joint/base velocity and zero command, seeds `obs[33:45]`, releases the driver pose override, and only then enables the first live ONNX inference.

To stop policy authority and hold the measured current pose:

```bash
ros2 run littlegreen_biped_pkg policy_handoff_control disable
```

Do not introduce forward, lateral, or yaw commands until supported zero-command closed-loop standing has been observed without a violent transient, persistent limit clipping, or IMU discontinuity latch.
