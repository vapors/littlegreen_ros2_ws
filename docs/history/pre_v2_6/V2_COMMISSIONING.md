# v2 commissioning sequence

The workspace defaults are intentionally conservative:

- `bhl_st3215_driver` writes disabled unless launch override enables them;
- `pd_controller_pkg` starts in `safety_only` mode;
- policy debug topics are enabled;
- the training-default pose move is explicit and service-gated.

## 1. Feedback-only + policy observation audit

Run the native driver with writes disabled, then launch the biped stack:

```bash
ros2 launch bhl_st3215_driver \
  bhl_st3215_driver.launch.py \
  enable_writes:=false
```

```bash
ros2 launch berkeley_biped_pkg \
  berkeley_biped_launch.py \
  controller_mode:=safety_only
```

Inspect:

```bash
ros2 topic echo /policy_debug/observation --once
ros2 topic echo /policy_debug/raw_action --once
ros2 topic echo /policy_debug/clipped_raw_action --once
ros2 topic echo /policy_debug/target_unclipped --once
ros2 topic echo /policy_debug/target_clipped --once
ros2 topic echo /policy_debug/saturation_mask --once
```

The saturation-mask byte uses:

```text
1 = raw action clipped
2 = lower joint limit
4 = upper joint limit
```

## 2. Explicit move to training default pose

Use the secure support/hanging apparatus. The simplest safe sequence is to start
the native driver with writes enabled before starting the policy stack, allowing
the driver's startup current-pose hold behavior to take effect first.

```bash
ros2 launch bhl_st3215_driver \
  bhl_st3215_driver.launch.py \
  enable_writes:=true
```

After complete feedback is healthy:

```bash
ros2 service call \
  /st3215_driver/move_to_default_pose \
  std_srvs/srv/Trigger '{}'
```

The driver will ramp smoothly from measured physical position to:

```text
[0.0, 0.0, -0.1, 0.4, -0.3, 0.0,
 0.0, 0.0, -0.1, 0.4, -0.3, 0.0]
```

and hold an internal override by default.

## 3. Align the controller before releasing the driver override

With the policy stack running in `safety_only` first:

```bash
ros2 service call \
  /pd_controller/reset_to_feedback \
  std_srvs/srv/Trigger '{}'
```

Then explicitly release the driver override:

```bash
ros2 service call \
  /st3215_driver/release_pose_override \
  std_srvs/srv/Trigger '{}'
```

## 4. Outer-PD dry-run and tuning

Before physical writes from the policy path, use writes-disabled dry runs and inspect:

```text
/outer_controller/position_error
/outer_controller/velocity_command
/outer_controller/status
/servo_target_radians
```

Launch the outer feedback mode with:

```bash
ros2 launch berkeley_biped_pkg \
  berkeley_biped_launch.py \
  controller_mode:=outer_pd
```

The first-pass gain values are deliberately conservative starting values, not
final physical tuning values and not copies of Isaac torque-domain stiffness or
damping.
