# Policy Observability Topics

When `publish_policy_debug=true`, the policy node publishes the exact finite data passed through ONNX and action-contract v4. Numeric debug topics use best-effort keep-last-1 QoS so a slow observer cannot back-pressure the 50 Hz policy timer.

## `/policy_debug/observation`

Type: `std_msgs/msg/Float64MultiArray`

The active v2.3.1 Stand bundle uses:

```text
0:3    command velocity [vx, vy, yaw_rate]
3:6    base angular velocity
6:9    projected gravity
9:21   q - q_default
21:33  joint velocity
33:45  previous bounded normalized action
45     sin(2*pi*phase)
46     cos(2*pi*phase)
```

For Stand, the final pair is sampled once at policy-episode start and remains unchanged. Readiness loss, command changes, and successful inference count do not mutate it.

## `/policy_debug/gait_phase`

Type: `std_msgs/msg/Float64MultiArray`

```text
[0] phase fraction in [0,1)
[1] policy episode number
[2] successful policy tick count
[3] sin(2*pi*phase)
[4] cos(2*pi*phase)
[5] mode code: 1=Stand, 2=Walk, 3=v2.8 legacy
[6] moving flag
[7] first-swing-left flag
```

For the current Stand policy, phase/sine/cosine are constant; the tick counter is diagnostic only. This is a software phase input, not measured foot contact.

## `/policy/reset_gait_phase`

Type: `std_srvs/srv/Trigger`

In shadow or disabled mode, this starts a new deployment episode and samples a new Stand phase. It is refused in live mode. The service has no servo-bus authority.

## Action topics

`/policy_debug/raw_action` is the raw 12-value ONNX output.

`/policy_debug/clipped_raw_action` is the normalized action after clipping to `[-1,1]`. This exact vector becomes the next previous-action observation.

`/policy_debug/target_unclipped` is:

```text
q_default + action_residual_scale_rad * clipped_raw_action
```

`/policy_debug/target_clipped` is the final 12-position target after physical clipping.

`/policy_debug/saturation_mask` uses one byte per joint:

```text
bit 0 / value 1: raw action clipped
bit 1 / value 2: target below lower physical limit
bit 2 / value 4: target above upper physical limit
```

## Runtime metrics

```bash
ros2 run littlegreen_biped_pkg policy_runtime_metrics --duration-sec 30
```

The recorder reports software phase and action/target metrics. It does not claim contact, COM, slip, swing clearance, physical torque, or electrical emergency-stop status.
