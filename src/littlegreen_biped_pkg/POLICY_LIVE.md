# Live Policy Mode

`policy_live.launch.py` starts the policy node in `live` mode and starts `pd_controller_node`. It does not start the ST3215 driver, IMU source, joystick, or keyboard.

```bash
ros2 launch littlegreen_biped_pkg policy_live.launch.py \
  controller_mode:=safety_only
```

## Contract gate

The packaged v2.3.1 canonical Stand bundle must pass:

```bash
ros2 run littlegreen_biped_pkg policy_bundle_audit
```

The node verifies the unmodified export schema, exact 47-D layout, Stand phase mode, actual ONNX input/output shapes, action contract v4, q-default, residual scales, physical bounds, canonical joint order, previous-action semantics, and checksum pairing.

## Stand episode phase

A live Stand episode samples one random phase during policy-node startup and holds it unchanged. Readiness loss does not resample. The explicit phase reset service and deterministic phase override are refused in live mode. To begin a new live episode, stop live policy output, re-check command authority and support, then intentionally restart/re-arm.

## Walk gate

Live Walk is blocked unless the exported bundle explicitly pins its deployment stage or exact phase period. The presence of a generic period field is not enough when checkpoint-stage provenance is ambiguous.

## Initial hardware rule

Use `controller_mode:=safety_only`, mechanical support, zero command velocity, and immediate access to servo power disconnect. Do not begin with `outer_pd` or `outer_pid`.

Full sequence: `docs/LIVE_POLICY_DEPLOYMENT.md`.
