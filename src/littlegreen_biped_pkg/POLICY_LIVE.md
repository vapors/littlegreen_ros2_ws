# Policy Live Mode

The recommended live launch is:

```bash
ros2 launch littlegreen_biped_pkg policy_live.launch.py \
  controller_mode:=safety_only
```

This launch starts `littlegreen_biped_node` in `live` mode and starts `pd_controller_node` in the selected controller mode. It does not launch the ST3215 driver, IMU source, or teleop nodes.

Before live mode:

1. deploy a paired `policy_latest.yaml` and `policy.onnx`;
2. rebuild `littlegreen_biped_pkg`;
3. pass driver and IMU preflight with writes disabled;
4. pass policy shadow validation;
5. restart the driver with `profile:=runtime_safe enable_writes:=true`;
6. rerun runtime preflight with `--expect-writes true`.

Action contract v3 is validated at node startup. The exported defaults, physical bounds, joint names, and action indices must match `joint_map.yaml`; the ONNX checksum must match `policy_sha256`.

Initial live deployment must use `controller_mode:=safety_only`. See the workspace page `docs/LIVE_POLICY_DEPLOYMENT.md` for the complete sequence.
