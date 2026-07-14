# lgh_st3215_tools

Guarded LittleGreen ST3215 calibration, characterization, preflight, auditing, and dataset tools.

## Pose terminology

- **model zero**: physical calibration fixture pose at `joint_zero_rad` (currently 0 rad)
- **policy default**: Track 1 standing stance stored as `training_default_rad`
- **physical limits**: durable `min_rad` / `max_rad`
- **raw limits**: `min_step` / `max_step` derived from center + radian limits

## Calibration commands

```bash
ros2 run lgh_st3215_tools print_model_zero
ros2 run lgh_st3215_tools print_policy_default

ros2 run lgh_st3215_tools capture_calibration \
  --reference model-zero

ros2 run lgh_st3215_tools capture_calibration \
  --reference model-zero \
  --joint leg_left_knee_pitch_joint

ros2 run lgh_st3215_tools apply_calibration \
  calibration_reports/<timestamp>/center_step_proposal.yaml \
  --source-servo-map ~/littlegreen_ros2_ws/src/lgh_st3215_driver/config/servo_map.yaml

ros2 run lgh_st3215_tools verify_model_zero
ros2 run lgh_st3215_tools assume_policy_default
ros2 run lgh_st3215_tools verify_policy_default --allow-writes-enabled
```

Compatibility aliases remain available:

```text
print_default_pose     -> print_policy_default
pose_console           -> assume_policy_default
verify_calibration     -> verify_model_zero
```

## Other tools

```text
servo_identification
standing_characterization
st3215_preflight
hardware_snapshot
dataset_manifest
```

The normal ROS tools operate above `lgh_st3215_driver`; they do not independently open the servo UART. The standalone physical endpoint tool lives at:

```text
tools/lgh_hardware_limit_tool/lgh_hardware_limit_tool.py
```
