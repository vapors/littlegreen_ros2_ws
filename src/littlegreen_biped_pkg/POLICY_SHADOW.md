# Policy Shadow Mode

Shadow mode executes the same observation builder, ONNX session, bounded action processing, previous-action update, gait-phase lifecycle, and target construction as live mode, but publishes the proposed target only on:

```text
/policy_shadow/desired_position
```

It does not create a policy publisher on `/desired_position`.

```bash
ros2 launch littlegreen_biped_pkg policy_shadow.launch.py
```

## Startup validation

v2.8.0 supports the legacy 45-D observation and the phase-guided 47-D observation. For action contracts v3 and v4, startup verifies the exported residual scale, defaults, physical bounds, joint names, action indices, previous-action semantics, ONNX checksum, and ONNX tensor shapes. Contract v4 also validates the non-uniform residual vector, nominal residual bounds, deployment profile, and required v4 transform flag.

A 47-D bundle additionally requires the exact v1.4.7 phase metadata. The packaged v1.4.5s3 policy remains 45-D; no deployable v1.4.7 pair is included.

## Graph checks

```bash
ros2 topic info /desired_position --verbose
ros2 topic info /policy_shadow/desired_position --verbose
ros2 topic echo /policy_status --once
ros2 topic echo /policy_ready --once
```

For a future 47-D policy:

```bash
ros2 topic echo /policy_debug/gait_phase
ros2 service call /policy/reset_gait_phase std_srvs/srv/Trigger '{}'
```

The reset is safe only because shadow mode has no live policy command authority. The topic reports expected phase timing, not measured contact.

## Runtime metrics

With policy debug enabled:

```bash
ros2 run littlegreen_biped_pkg policy_runtime_metrics \
  --duration-sec 30
```

The recorder is read-only and writes CSV/YAML under `~/.ros/littlegreen_policy_metrics/` by default.
