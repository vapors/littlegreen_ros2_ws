# Runtime Integration

The native ST3215 driver preserves the current policy and controller topic contract.

## Normal control graph

```text
/imu/data ---------------------------> littlegreen_biped_node
/joint_states -----------------------> littlegreen_biped_node
/joint_feedback_age_ms -------------> littlegreen_biped_node
/command_velocity ------------------> littlegreen_biped_node

littlegreen_biped_node
  /desired_position
        |
        v
pd_controller_node
  /servo_target_radians
        |
        v
lgh_st3215_driver
        |
        +--> /joint_states
        +--> /joint_feedback_age_ms
        +--> /st3215_driver/diagnostics
```

## Commissioning integration

Start the driver separately so the active profile and write state remain explicit:

```bash
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=commissioning \
  enable_writes:=false
```

Validate:

```bash
ros2 run lgh_st3215_tools st3215_preflight \
  --mode commissioning \
  --expect-writes false
```

## Policy shadow integration

Use the runtime-safe publication surface and keep writes disabled:

```bash
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=runtime_safe \
  enable_writes:=false
```

Then start the dedicated policy-only shadow launch:

```bash
ros2 launch littlegreen_biped_pkg policy_shadow.launch.py
```

Do not use `override_imu:=true` as a hardware commissioning substitute. Shadow validation should use the real `/imu/data` stream after the IMU preflight passes.

## Live integration boundary

The broader `littlegreen_biped_launch.py` starts joystick, teleop, the policy node, the file bridge, and `pd_controller_pkg`. Keep the servo driver in a separate launch during early deployment so `profile` and `enable_writes` remain visible and independently controllable.

Live motion remains gated by the Track 1 deployment bundle and hardware-side contract audit.
