# LittleGreen ROS 2 v2.9.4 validation

## Scope

This validation covers the v2.9.4 source revision that separates the Track-1 learned zero-command pose from robot-specific handoff trim and synchronizes the active source policy bundle to v10.2 `model_10000`.

## Static/source validation

Validated without ROS runtime dependencies:

- workspace version `2.9.4` and `littlegreen_biped_pkg` version `0.7.4`;
- all 17 package manifests discovered by the source validator;
- no packaged Python cache/pytest cache artifacts;
- active v10.2 source policy task, role, 47-D observation contract, 12-D action-v4 contract, neutral-static phase, and ONNX SHA;
- source `policy.yaml` / `policy_latest.yaml` byte identity;
- config ONNX / checkpoint ONNX byte identity;
- bundle manifest, deployment contract, and `policy.sha256` consistency;
- exact Track-1 handoff source pose, 12-D hardware trim, effective pose, physical bounds, policy SHA, and orientation-calibration provenance;
- Python handoff/auditor/installer/golden-vector tests;
- built-in ONNX tensor inspection `[1,47] -> [1,12]`.

## v2.9.4 handoff invariants

The packaged handoff must satisfy:

```text
joint_position_rad
  = physical_limit_clamp(source_median_joint_position_rad + hardware_trim_rad)
```

Current calibrated trim:

```text
L hip pitch    -0.035 rad
L ankle pitch  -0.100 rad
R hip pitch    -0.035 rad
R ankle pitch  -0.100 rad
all others      0.000 rad
```

The previous-action seed remains the Track-1 analyzer median and is not transformed into radians.

## Physical evidence recorded

The effective pose records the 2026-10-09 supported orientation audit:

```text
projected gravity base = [+0.0178999640, -0.0230282932, -0.9995786694]
RPY deg                = [-1.02588, +1.31954, -0.20401]
result                 = PASS
```

## Remaining Orange Pi validation

The build environment used to assemble this release does not provide ROS 2 Humble/ONNX Runtime C++ headers, so the authoritative C++ compile and live ROS service validation remain on the Orange Pi.

Required before first zero-command closed-loop test:

1. rebuild `littlegreen_biped_pkg` on the Orange Pi;
2. run `policy_bundle_audit` and require PASS;
3. run `policy_handoff_control pose` and confirm the calibrated posture;
4. keep the robot mechanically supported;
5. start `policy_live.launch.py` with `controller_mode:=safety_only` and zero command;
6. run `policy_handoff_control live`;
7. stop immediately with `policy_handoff_control disable` or servo power disconnect if the transition is not smooth.
