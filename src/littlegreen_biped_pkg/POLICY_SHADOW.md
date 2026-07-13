# Policy Shadow Mode

Shadow mode executes the same observation builder, ONNX session, bounded action processing, previous-action update, and target construction as live mode, but publishes the proposed target only on:

```text
/policy_shadow/desired_position
```

It does not create a policy publisher on `/desired_position`.

```bash
ros2 launch littlegreen_biped_pkg policy_shadow.launch.py
```

## Startup validation

For action contracts v3 and v4, startup verifies the exported residual scale, defaults, physical bounds, joint names, action indices, previous-action semantics, and ONNX checksum. Contract v4 also validates the non-uniform residual vector, nominal residual bounds, deployment profile, and required v4 transform flag. Any mismatch is fatal.

The packaged v1.4.5s3 policy uses action contract v4.

## Graph checks

```bash
ros2 topic info /desired_position --verbose
ros2 topic info /policy_shadow/desired_position --verbose
ros2 topic echo /policy_status --once
ros2 topic echo /policy_ready --once
```

## Runtime metrics

With policy debug enabled:

```bash
ros2 run littlegreen_biped_pkg policy_runtime_metrics \
  --duration-sec 30
```

The recorder is read-only and writes CSV/YAML under `~/.ros/littlegreen_policy_metrics/` by default.
