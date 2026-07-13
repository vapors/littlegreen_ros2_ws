# Live Policy Mode

`policy_live.launch.py` starts the policy node in `live` mode and starts `pd_controller_node`. It does not start the ST3215 driver, IMU source, joystick, or keyboard.

```bash
ros2 launch littlegreen_biped_pkg policy_live.launch.py \
  controller_mode:=safety_only
```

## Contract gate

Action contracts v3 and v4 are validated at node startup. The packaged v1.4.5s3 policy uses contract v4 with a per-joint residual vector.

Before inference, the node validates exported defaults, physical bounds, joint names, action indices, normalized action limits, previous-action semantics, and the ONNX checksum against `joint_map.yaml`. Contract v4 also validates nominal residual bounds, the deployment profile, and the required v4 transform flag.

Run the offline audit before launch:

```bash
ros2 run littlegreen_biped_pkg policy_bundle_audit
```

## Initial hardware rule

Use `controller_mode:=safety_only`, mechanical support, zero command velocity, and immediate access to servo power disconnect. Do not begin with `outer_pd` or `outer_pid`.

Full sequence: `docs/LIVE_POLICY_DEPLOYMENT.md`.
