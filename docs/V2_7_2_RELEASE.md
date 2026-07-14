# LittleGreen ROS 2 v2.7.2

v2.7.2 simplifies servo calibration and replacement by separating **model zero** from the Track 1 **policy-default stance**.

## Corrected calibration model

```text
model zero
  physical fixture pose at joint_zero_rad
  center_step is captured here

policy default
  Track 1 standing stance at training_default_rad
  commanded after calibration

physical limits
  durable min_rad/max_rad

raw limits
  derived min_step/max_step from center + radian limits
```

A center-only calibration no longer appears to invalidate the physical endpoint capture. The calibrated center may move by a few steps while the same model-space physical limits remain authoritative.

## New and clarified commands

```text
print_model_zero
print_policy_default
capture_calibration --reference model-zero
verify_model_zero
assume_policy_default
verify_policy_default
```

Compatibility aliases:

```text
print_default_pose -> print_policy_default
pose_console       -> assume_policy_default
verify_calibration -> verify_model_zero
```

## Replacement-servo support

`capture_calibration` now accepts one or more `--joint` arguments. A one-servo replacement can be calibrated without creating an all-joint proposal.

`apply_calibration` updates:

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

The radian limits and policy-default values are not changed.

## LittleGreen Hardware Limit Tool v1.1.0

The standalone tool is included at:

```text
tools/lgh_hardware_limit_tool/
```

It uses current LittleGreen paths and stores physical limits in model-space radians. After center calibration, `render` can regenerate raw endpoints from the existing physical capture and current centers without moving every joint through its limits again.

## Documentation

- [`CALIBRATION_WORKFLOW.md`](CALIBRATION_WORKFLOW.md)
- [`SERVO_REPLACEMENT_CHECKLIST.md`](SERVO_REPLACEMENT_CHECKLIST.md)
- [`HARDWARE_CONTRACT.md`](HARDWARE_CONTRACT.md)
- [`COMMAND_CHEATSHEET.md`](COMMAND_CHEATSHEET.md)

## Scope lock

Unchanged in v2.7.2:

- Track 1 v1.4.5s3 policy YAML and ONNX pair;
- action contract v4;
- policy-default joint vector;
- physical radian limits;
- servo IDs and signs;
- driver bus timing and feedback behavior;
- IMU and controller behavior.

## Apply over v2.7.1

Extract the v2.7.2 hotfix at the workspace root, then run:

```bash
cd ~/littlegreen_ros2_ws
./scripts/apply_v2_7_2_hotfix.sh
```

The script validates the updated tree and prints the focused rebuild command.
