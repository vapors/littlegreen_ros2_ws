# Policy Observability Topics

When `publish_policy_debug=true`, the policy node publishes the exact data path around ONNX inference.

Numeric debug topics use best-effort keep-last-1 QoS so a slow observer cannot back-pressure the 50 Hz policy timer.

## `/policy_debug/observation`

Type: `std_msgs/msg/Float64MultiArray`

The layout is selected by the audited policy bundle.

Legacy 45-D contract:

```text
0:3    command velocity [vx, vy, wz]
3:6    base angular velocity
6:9    projected gravity
9:21   q - q_default
21:33  qdot
33:45  previous bounded normalized action
```

Phase-guided 47-D contract:

```text
0:45   identical to the legacy contract
45     sin(2*pi*phase)
46     cos(2*pi*phase)
```

A 47-D observation begins at approximately `[sin, cos] = [0, 1]`. The phase advances only after successful inference and output-path update, freezes while readiness is gated, and wraps every 36 successful policy ticks for the current 0.72 s / 0.02 s contract.

## `/policy_debug/gait_phase`

Published only for a 47-D phase-guided policy.

Type: `std_msgs/msg/Float64MultiArray`

```text
[0] phase fraction in [0,1)
[1] logical phase tick
[2] ticks per period
[3] sin(2*pi*phase)
[4] cos(2*pi*phase)
[5] expected half-cycle: 0=left stance/right swing, 1=right stance/left swing
```

This topic reports the policy's expected phase. It is not a measured foot-contact signal.

## `/policy/reset_gait_phase`

Type: `std_srvs/srv/Trigger`

The service exists only for a 47-D policy. It resets the logical clock to phase zero in `shadow` or `disabled` mode. It is refused in `live` mode; stop and restart the guarded live policy instead of creating an abrupt live phase discontinuity.

## `/policy_debug/raw_action`

Raw 12-value ONNX output before deployment post-processing.

## `/policy_debug/clipped_raw_action`

Raw action after action-limit clipping.

## `/policy_debug/target_unclipped`

```text
q_default + action_residual_scale_rad * clipped_raw_action
```

before physical joint-limit clipping.

## `/policy_debug/target_clipped`

Final 12-position target after physical joint-limit clipping.

In live mode this has the same target semantics as `/desired_position`. In shadow mode it has the same target semantics as `/policy_shadow/desired_position`.

## `/policy_debug/saturation_mask`

Type: `std_msgs/msg/UInt8MultiArray`

One byte per joint:

```text
bit 0 / value 1: raw action was clipped
bit 1 / value 2: target was below the lower physical limit
bit 2 / value 4: target was above the upper physical limit
```

A value of zero means no clipping occurred for that joint.

## Runtime metrics recorder

`policy_runtime_metrics` combines these debug topics with `/joint_states` to produce Track 1-aligned real-hardware metrics:

```bash
ros2 run littlegreen_biped_pkg policy_runtime_metrics --duration-sec 30
```

For 47-D policies it also records the expected phase fraction, sine/cosine, unit-circle error, and expected half-cycle. These remain expected policy timing rather than measured contact. The recorder does not claim COM, foot-contact, swing-clearance, slip, or physical-torque metrics because those are not available from the current runtime sensors.
