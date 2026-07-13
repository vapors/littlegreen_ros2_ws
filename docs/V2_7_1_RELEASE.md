# LittleGreen ROS 2 v2.7.1

v2.7.1 propagates the latest Track 1 v1.4.5s3 deployment contract and analysis boundary into the hardware workspace without changing the ST3215 communication protocol or introducing aggressive outer-PD control.

## Policy contract

The packaged paired export is:

```text
Task:      Velocity-Lilgreen-Stand-ST3215-Loaded-v5s3
Rate:      50 Hz
Interface: observation[45] -> action[12]
Contract:  v4 bounded default-centered vector residual
Profile:   v1_4_5_stabilized_vector_residual
ONNX SHA:  c0079190bc0bf531a5eb6928541560d2402d1f711e14b641e436cad5d43e854d
```

`littlegreen_biped_node` now supports both action contracts v3 and v4. For v4 it loads the per-joint `action_residual_scale_rad` vector and validates nominal residual bounds, the deployment profile, and the required v4 transform flag in addition to the existing default, physical-bound, action-index, joint-name, normalized-limit, previous-action, and checksum gates.

## Default pose propagation

The current athletic default pose is synchronized across:

```text
policy_latest.yaml
joint_map.yaml
lgh_st3215_driver/config/servo_map.yaml
track1_action_contract_v4.yaml
```

```text
[0.0, 0.0, -0.24, 0.62, -0.22, 0.0,
 0.0, 0.0, -0.24, 0.62, -0.22, 0.0]
```

The v4 per-joint residual vector is:

```text
[0.24, 0.16, 0.42, 0.58, 0.48, 0.26,
 0.24, 0.16, 0.42, 0.58, 0.48, 0.26]
```

## New tools

- `policy_bundle_audit` — offline paired YAML/ONNX and hardware-map audit.
- `policy_runtime_metrics` — read-only shadow/live recorder for Track 1-aligned metrics observable from current ROS topics.

## Metric boundary

v2.7.1 records raw-action excess, normalized saturation, physical target clipping, residual demand, tracking error, velocity-limit use, projected gravity, and the exact command entering the policy observation.

It explicitly does not claim base COM height, COM-forward offset, foot-support state, swing clearance, foot slip, or physical torque without additional sensing or validated estimation.

## Unchanged boundaries

- ST3215 bus timing and packet behavior
- calibrated servo centers and physical endpoints
- driver profile behavior
- IMU source abstraction
- `safety_only` initial live-policy path
- offline read-only maintenance restriction
- no aggressive outer-PD integration
