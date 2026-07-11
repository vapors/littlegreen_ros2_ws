# Policy Observability Topics

When `publish_policy_debug=true`, the policy node publishes the data path around ONNX inference.

Numeric debug topics use best-effort keep-last-1 QoS so a slow observer cannot back-pressure the policy timer.

## `/policy_debug/observation`

Type: `std_msgs/msg/Float64MultiArray`

45 values in the exact ONNX order:

```text
0:3    command velocity [vx, vy, wz]
3:6    base angular velocity
6:9    projected gravity
9:21   q - q_default
21:33  qdot
33:45  previous clipped raw action
```

## `/policy_debug/raw_action`

Raw 12-value ONNX output before deployment post-processing.

## `/policy_debug/clipped_raw_action`

Raw action after action-limit clipping.

## `/policy_debug/target_unclipped`

```text
q_default + action_scale * clipped_raw_action
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
