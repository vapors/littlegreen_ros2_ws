# Live Policy Mode

`policy_live.launch.py` starts the policy node in `live` mode and starts `pd_controller_node`. It does not start the ST3215 driver, IMU source, joystick, or keyboard.

```bash
ros2 launch littlegreen_biped_pkg policy_live.launch.py \
  controller_mode:=safety_only
```

## Contract gate

v2.8.0 validates observation and action contracts independently:

- legacy 45-D observations with action contracts v3 or v4;
- phase-guided 47-D observations with action contract v4;
- all other observation dimensions are rejected.

The packaged v1.4.5s3 policy remains a known-good 45-D action-contract-v4 bundle. No deployable v1.4.7 policy is included.

Before inference, the node validates observation metadata, ONNX input/output tensor dimensions, exported defaults, physical bounds, joint names, action indices, normalized action limits, previous-action semantics, and the ONNX checksum against `joint_map.yaml`. Contract v4 also validates nominal residual bounds, the deployment profile, and the required v4 transform flag.

Run the offline audit before launch:

```bash
ros2 run littlegreen_biped_pkg policy_bundle_audit
```

A future v1.4.7 bundle is deployable only when its YAML reports 47 observations, includes the exact gait-phase metadata, its ONNX input is actually `[1,47]`, and the paired checksum passes.

## Phase lifecycle in live mode

A 47-D policy begins at phase zero, advances only after a successful policy tick, and freezes while readiness is gated. Zero command velocity does not stop the clock. The phase-reset service is intentionally refused in live mode; stop the policy and restart the guarded live launch to begin a new deployment episode at phase zero.

## Initial hardware rule

Use `controller_mode:=safety_only`, mechanical support, zero command velocity, and immediate access to servo power disconnect. Do not begin with `outer_pd` or `outer_pid`.

Full sequence: `docs/LIVE_POLICY_DEPLOYMENT.md`.
