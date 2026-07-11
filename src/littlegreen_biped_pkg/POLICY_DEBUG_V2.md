# Policy observability topics — v2

The policy node can publish the exact data path around ONNX inference when
`publish_policy_debug: true`.

All numeric debug topics use best-effort keep-last-1 QoS so a slow echo or
logger cannot back-pressure the 25 Hz policy timer.

## Topics

### `/policy_debug/observation`

`std_msgs/msg/Float64MultiArray`, 45 values in the exact order passed to ONNX:

```text
0:3    command velocity [vx, vy, wz]
3:6    base angular velocity
6:9    projected gravity
9:21   q - q_default
21:33  qdot
33:45  previous clipped raw action
```

### `/policy_debug/raw_action`

Raw 12-value ONNX tensor output before deployment post-processing.

### `/policy_debug/clipped_raw_action`

Raw action after `action_limit_lower` / `action_limit_upper`.

### `/policy_debug/target_unclipped`

```text
q_default + action_scale * clipped_raw_action
```

before physical joint-limit clipping.

### `/policy_debug/target_clipped`

Final 12-position target after physical joint-limit clipping. This is the same
semantic target published on `/desired_position`.

### `/policy_debug/saturation_mask`

`std_msgs/msg/UInt8MultiArray`, one byte per action:

```text
bit 0 / value 1: raw action was clipped
bit 1 / value 2: target was below lower physical limit
bit 2 / value 4: target was above upper physical limit
```

A value of zero means no clipping occurred for that joint.
