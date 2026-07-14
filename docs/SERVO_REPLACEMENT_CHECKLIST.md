# ST3215 Servo Replacement Checklist

Use this checklist when replacing one ST3215 servo without changing the LittleGreen joint geometry.

## Before removal

- [ ] Record the joint name and servo ID.
- [ ] Support the robot mechanically.
- [ ] Keep the servo-power disconnect reachable.
- [ ] Stop policy, PD, identification, and maintenance processes.
- [ ] Save the current `servo_map.yaml` and recent preflight/hardware snapshot.

## Install the replacement

- [ ] Configure the replacement servo with the required ID and 1,000,000-baud bus setting.
- [ ] Install the servo and horn so the linkage can be aligned to model zero.
- [ ] Do not assume raw step 2048 is the physical joint zero.
- [ ] Confirm cable routing and mechanical freedom before energizing torque.

## Capture model zero

Start feedback-only commissioning:

```bash
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=commissioning \
  enable_writes:=false
```

- [ ] Place the replaced joint at exact model zero (`joint_zero_rad`, currently 0 rad).
- [ ] Use an external fixture, angle gauge, or alignment mark.
- [ ] Capture only the replaced joint:

```bash
ros2 run lgh_st3215_tools capture_calibration \
  --reference model-zero \
  --joint <joint_name> \
  --servo-map ~/littlegreen_ros2_ws/src/lgh_st3215_driver/config/servo_map.yaml
```

- [ ] Review `correction_steps` and the status.
- [ ] Re-index the horn and repeat when `MECHANICAL_REINDEX_RECOMMENDED` is reported.
- [ ] Confirm derived raw endpoints remain inside 0..4095.

## Apply and rebuild

Dry-run:

```bash
ros2 run lgh_st3215_tools apply_calibration \
  calibration_reports/<timestamp>/center_step_proposal.yaml \
  --source-servo-map ~/littlegreen_ros2_ws/src/lgh_st3215_driver/config/servo_map.yaml
```

Apply:

```bash
ros2 run lgh_st3215_tools apply_calibration \
  calibration_reports/<timestamp>/center_step_proposal.yaml \
  --source-servo-map ~/littlegreen_ros2_ws/src/lgh_st3215_driver/config/servo_map.yaml \
  --apply
```

- [ ] Confirm the diff changes only the selected joint’s center/raw deployment endpoints and the joint-map mirror.
- [ ] Rebuild:

```bash
cd ~/littlegreen_ros2_ws
colcon build --symlink-install \
  --packages-select lgh_st3215_driver littlegreen_biped_pkg
source install/setup.bash
```

## Verify model zero

- [ ] Restart feedback-only commissioning.
- [ ] Hold the replaced joint at exact model zero.
- [ ] Run:

```bash
ros2 run lgh_st3215_tools verify_model_zero \
  --joint <joint_name>
```

- [ ] Require PASS, or investigate before continuing.

## Assume and verify policy default

Restart commissioning with writes enabled and a slow pose ramp:

```bash
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=commissioning \
  enable_writes:=true \
  default_pose_move_duration_sec:=8.0
```

- [ ] Run the guarded move:

```bash
ros2 run lgh_st3215_tools assume_policy_default
```

- [ ] Verify the selected joint:

```bash
ros2 run lgh_st3215_tools verify_policy_default \
  --allow-writes-enabled \
  --joint <joint_name>
```

- [ ] Visually confirm the physical joint matches the simulator’s policy-default stance.
- [ ] Return to feedback-only mode before policy shadow.

## Physical-limit decision

A new endpoint capture is **not normally required** for a like-for-like replacement.

Run the LittleGreen Hardware Limit Tool only when:

- [ ] linkage or stop geometry changed;
- [ ] horn indexing cannot reproduce the original model-zero relationship;
- [ ] derived raw endpoints fall outside 0..4095;
- [ ] physical contact occurs before the known model-space limit;
- [ ] previous endpoint data is not trusted.

## Final records

- [ ] center-step proposal and report
- [ ] source-map backups
- [ ] updated `servo_map.yaml`
- [ ] synchronized `joint_map.yaml` mirror
- [ ] model-zero verification report
- [ ] policy-default verification report
- [ ] operator note with replacement servo ID/date
