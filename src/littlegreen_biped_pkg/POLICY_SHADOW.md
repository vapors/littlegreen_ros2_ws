# Policy Shadow Mode

Shadow mode uses the same observation construction, freshness gates, ONNX inference, action clipping, joint target mapping, and previous-action update behavior as live mode.

It publishes proposed targets on:

```text
/policy_shadow/desired_position
```

It does not create a policy publisher on `/desired_position`.

## Recommended launch

Run the servo driver separately with the runtime-safe publication profile and writes disabled:

```bash
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=runtime_safe \
  enable_writes:=false
```

Then:

```bash
ros2 launch littlegreen_biped_pkg policy_shadow.launch.py
```

The dedicated shadow launch starts only `littlegreen_biped_node`. It does not launch teleop, the PD controller, or the ST3215 driver.

## Validation

```bash
ros2 topic info /desired_position --verbose
ros2 topic info /policy_shadow/desired_position --verbose
ros2 topic echo /policy_status --once
ros2 topic hz /policy_shadow/desired_position
```

For action contract v3, node startup verifies the exported residual scale, defaults, physical bounds, joint names, and action indices against `joint_map.yaml`, then verifies the ONNX checksum. Any mismatch is fatal.

After shadow acceptance, use `policy_live.launch.py` with `controller_mode:=safety_only`. The complete sequence is documented in the workspace page `docs/LIVE_POLICY_DEPLOYMENT.md`.
