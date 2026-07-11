# pd_controller_pkg

Safety envelope, reference shaper, and configurable outer-loop controller for
LittleGreen ST3215 position servos.

## Modes

`controller_mode` is set in `config/pd_config.yaml`:

- `safety_only`: preserve the previous sanitize/filter/rate/accel limiting path;
- `outer_pd`: measured position/velocity feedback influences the position target;
- `outer_pid`: adds integral action with clamp and conditional anti-windup.

The controller is intentionally velocity-form:

```text
error      = q_ref - q
vel_error  = qdot_ref - qdot
qdot_cmd   = Kp*error + Kd*vel_error + Ki*integral(error)
q_cmd_next = q_cmd_prev + bounded(qdot_cmd) * dt
```

The output remains `/servo_target_radians`; ST3215 step conversion remains in
`lgh_st3215_driver`.

The default mode is `safety_only`. This allows policy diagnostics and physical
pose commissioning without changing the previous downstream command behavior.

## Reset state to feedback

After an external pose override or manual repositioning, align the shaper and
controller state to current measured feedback:

```bash
ros2 service call \
  /pd_controller/reset_to_feedback \
  std_srvs/srv/Trigger '{}'
```

## Debug topics

```text
/outer_controller/velocity_command
/outer_controller/position_error
/outer_controller/integral_error
/outer_controller/status
```

`/pd_torque_debug` remains available for compatibility but is debug-only and is
not connected to the ST3215 command path.
