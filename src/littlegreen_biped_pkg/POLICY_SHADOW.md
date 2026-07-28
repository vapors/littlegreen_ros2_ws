# Policy Shadow Mode

Shadow mode executes the same bundle parser, exact 47-D builder, ONNX session, action-contract-v4 transform, previous-action update, phase state, and target construction as live mode, but publishes proposed targets only on:

```text
/policy_shadow/desired_position
```

It does not create a policy publisher on `/desired_position`.

```bash
ros2 launch littlegreen_biped_pkg policy_shadow.launch.py
```

## v2.3.1 Stand checks

Startup validates the complete five-file bundle, actual ONNX `[1,47] -> [1,12]` interface, canonical joint order, q-default, residual scales, physical limits, previous-action semantics, and `randomized_static_per_episode` phase mode.

Inspect:

```bash
ros2 topic info /desired_position --verbose
ros2 topic info /policy_shadow/desired_position --verbose
ros2 topic echo /policy_status --once
ros2 topic echo /policy_ready --once
ros2 topic echo /policy_debug/gait_phase
ros2 topic echo /policy_debug/observation
```

The Stand phase pair must remain constant for the entire episode, including transient readiness loss.

An intentional new shadow episode is:

```bash
ros2 service call \
  /policy/reset_gait_phase \
  std_srvs/srv/Trigger '{}'
```

## Deterministic replay

A fixed phase is available only for tests and replay:

```bash
ros2 launch littlegreen_biped_pkg policy_shadow.launch.py \
  enable_phase_test_override:=true \
  phase_test_fixed_value:=0.25
```

The same override is refused in live mode.

## Metrics

```bash
ros2 run littlegreen_biped_pkg policy_runtime_metrics \
  --duration-sec 30
```

Outputs are read-only and default under `~/.ros/littlegreen_policy_metrics/`.
