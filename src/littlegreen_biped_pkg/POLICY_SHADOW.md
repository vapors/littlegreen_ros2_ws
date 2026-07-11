# Policy shadow mode — v2.6.0

Shadow mode uses the same observation construction, freshness gates, ONNX inference, action clipping, joint target mapping, and previous-action update as live mode, but does not create a policy publisher on `/desired_position`.

```bash
ros2 launch littlegreen_biped_pkg policy_shadow.launch.py
```

Output:

```text
/policy_shadow/desired_position
```

The dedicated launch starts only `littlegreen_biped_node`. It does not launch teleop, the PD controller, or the ST3215 driver.

Recommended hardware-side companion launch:

```bash
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=runtime_safe enable_writes:=false
```

Validation:

```bash
ros2 topic info /desired_position --verbose
ros2 topic info /policy_shadow/desired_position --verbose
ros2 topic echo /policy_status --once
ros2 topic hz /policy_shadow/desired_position
```

The current packaged policy remains a workspace snapshot. Track 1 deployment-bundle pairing is deferred and must be audited before using shadow results to authorize live hardware commands.
