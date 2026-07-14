# ST3215 Model-Zero Calibration Workflow

This workflow calibrates the mapping between LittleGreen model joint angles and ST3215 raw encoder steps.

The key distinction is:

| Term | Meaning |
|---|---|
| **Model zero** | Physical calibration pose where each actuated joint is at `joint_zero_rad`—currently 0 rad. |
| **Policy default** | The Track 1 standing stance stored as `training_default_rad`. It is a commanded pose, not the zero-calibration fixture. |
| **Physical limits** | Durable safe joint range in model-space radians: `min_rad` / `max_rad`. |
| **Raw limits** | ST3215 step endpoints derived from the calibrated center and model-space limits. |

For the current robot:

```text
model zero:
  all 12 actuated joints = 0 rad

policy default:
  hip pitch   = -0.24 rad
  knee pitch  = +0.62 rad
  ankle pitch = -0.22 rad
  roll/yaw    =  0.00 rad
```

Keep the robot securely supported and keep the physical servo-power disconnect reachable.

## 1. Start the commissioning driver feedback-only

```bash
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=commissioning \
  enable_writes:=false
```

Verify the driver and command graph:

```bash
ros2 run lgh_st3215_tools st3215_preflight \
  --mode commissioning \
  --expect-writes false

ros2 topic info /servo_target_radians --verbose
```

For calibration, require `Publisher count: 0`. A policy, PD controller, identification tool, or standing tool must not be publishing during center capture.

## 2. Put the robot in model zero

With torque disabled and the robot supported, align the links to the straight model-zero fixture pose.

Print the reference:

```bash
ros2 run lgh_st3215_tools print_model_zero \
  --servo-map ~/littlegreen_ros2_ws/src/lgh_st3215_driver/config/servo_map.yaml
```

At model zero, `center_step` is simply the measured raw servo position. It does not need to equal 2048.

Use an external physical reference—fixture, digital angle gauge, or alignment marks. ROS feedback alone cannot prove horn alignment because the same center is used for command and feedback conversion.

## 3. Capture model-zero centers

All joints:

```bash
ros2 run lgh_st3215_tools capture_calibration \
  --reference model-zero \
  --servo-map ~/littlegreen_ros2_ws/src/lgh_st3215_driver/config/servo_map.yaml
```

One replacement servo only:

```bash
ros2 run lgh_st3215_tools capture_calibration \
  --reference model-zero \
  --joint leg_left_knee_pitch_joint \
  --servo-map ~/littlegreen_ros2_ws/src/lgh_st3215_driver/config/servo_map.yaml
```

The tool writes a review proposal and does not edit source files.

Correction categories:

```text
NO_CHANGE                       no update required
FINE_SOFTWARE_CORRECTION        <= 25 steps
INSPECT_MECHANICAL_ALIGNMENT    26..100 steps
MECHANICAL_REINDEX_RECOMMENDED  > 100 steps
RAW_RANGE_OUT_OF_BOUNDS         derived endpoint falls outside 0..4095
UNSTABLE_CAPTURE                joint moved during sampling
```

Small center changes are no longer mislabeled as physical-limit conflicts. The tool preserves `min_rad` / `max_rad` and derives new `min_step` / `max_step` values from the proposed center.

## 4. Dry-run the update

```bash
ros2 run lgh_st3215_tools apply_calibration \
  calibration_reports/<timestamp>/center_step_proposal.yaml \
  --source-servo-map ~/littlegreen_ros2_ws/src/lgh_st3215_driver/config/servo_map.yaml
```

When the standard source-tree layout is used, the tool automatically finds and synchronizes:

```text
src/littlegreen_biped_pkg/src/configs/joint_map.yaml
```

It updates only these deployment fields:

```text
servo_map.yaml:
  center_step
  min_step
  max_step

joint_map.yaml mirror:
  servo_center_step
  servo_min_step
  servo_max_step
```

It does not change the policy default or model-space radian limits.

## 5. Apply after review

Normal reviewed proposal:

```bash
ros2 run lgh_st3215_tools apply_calibration \
  calibration_reports/<timestamp>/center_step_proposal.yaml \
  --source-servo-map ~/littlegreen_ros2_ws/src/lgh_st3215_driver/config/servo_map.yaml \
  --apply
```

A proposal containing reviewed `MECHANICAL_REINDEX_RECOMMENDED` entries requires an explicit acknowledgement:

```bash
ros2 run lgh_st3215_tools apply_calibration \
  calibration_reports/<timestamp>/center_step_proposal.yaml \
  --source-servo-map ~/littlegreen_ros2_ws/src/lgh_st3215_driver/config/servo_map.yaml \
  --allow-large-corrections \
  --apply
```

The flag does not prove the physical pose was correct; it records that the operator reviewed and accepted the large correction.

Timestamped backups are created before both source files are modified.

## 6. Rebuild the affected packages

```bash
cd ~/littlegreen_ros2_ws

colcon build --symlink-install \
  --packages-select \
    lgh_st3215_driver \
    littlegreen_biped_pkg

source install/setup.bash
```

## 7. Verify model zero

Restart the commissioning driver feedback-only, leave the robot physically aligned at model zero, then run:

```bash
ros2 run lgh_st3215_tools verify_model_zero \
  --servo-map ~/littlegreen_ros2_ws/src/lgh_st3215_driver/config/servo_map.yaml
```

Default raw-step thresholds:

```text
PASS  <= 8 steps
WARN  <= 16 steps
FAIL  > 16 steps
```

This checks encoder steps against the calibrated centers. External physical alignment remains the proof that the pose is truly model zero.

## 8. Command the policy-default stance

Stop the feedback-only driver. Confirm there is no stale command publisher:

```bash
ros2 topic info /servo_target_radians --verbose
```

Then relaunch with writes enabled:

```bash
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=commissioning \
  enable_writes:=true \
  default_pose_move_duration_sec:=8.0
```

Then run:

```bash
ros2 run lgh_st3215_tools assume_policy_default
```

`pose_console` remains as a compatibility alias:

```bash
ros2 run lgh_st3215_tools pose_console
```

The underlying service name remains `/st3215_driver/move_to_default_pose` for compatibility, but the pose it commands is the **policy-default stance**, not model zero.

## 9. Verify the policy-default stance

After the guarded ramp completes:

```bash
ros2 run lgh_st3215_tools verify_policy_default \
  --allow-writes-enabled \
  --servo-map ~/littlegreen_ros2_ws/src/lgh_st3215_driver/config/servo_map.yaml
```

This compares the measured raw steps with the expected policy-default raw targets. It does not propose new centers.

## When to repeat the hardware-limit capture

A replacement servo normally does **not** require a new physical-limit capture when the robot geometry and horn relationship are restored.

Repeat the standalone limit tool only when:

- the linkage or mechanical stops changed;
- the replacement horn cannot be installed in the equivalent orientation;
- the derived raw range falls outside 0..4095;
- the joint contacts the mechanism before the known radian limit;
- the previous limit capture is uncertain.

After a center-only calibration, the saved hardware-limit capture can be re-rendered against the new centers without moving the joints through their endpoints again. See [`HARDWARE_CONTRACT.md`](HARDWARE_CONTRACT.md).
