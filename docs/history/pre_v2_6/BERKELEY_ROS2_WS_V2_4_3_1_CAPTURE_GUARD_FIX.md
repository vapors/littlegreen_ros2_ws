# Berkeley ROS 2 Workspace v2.4.3.1 — Torque-off capture guard fix

This patch fixes a capture-mode workflow issue in `standing_load_characterization_runner.py`.

## Problem

v2.4.3 used the same guarded command-range validation for:

1. manually observed, torque-off pose capture, and
2. torque-on pose evaluation.

A physically observed pose outside the current software command guard therefore aborted before the pose library could be saved. This prevented the captured data from being used to review whether the configured limits or calibration were stale/conservative.

## v2.4.3.1 behavior

- `capture_pose` is observation-first. It saves a stable torque-off measured pose even when one or more joints are outside the current guarded command range.
- The saved capture metadata contains:
  - `command_guard_valid`
  - `command_guard_violations`
- A prominent warning is printed for every violation.
- `evaluate` mode remains strict and refuses to command any pose outside the current guarded range.
- `--capture-require-commandable` restores the old strict capture behavior when desired.

## Safety rationale

Capturing a torque-off observation does not command motion. Evaluation does. The two operations therefore need different validation behavior.

This patch deliberately does **not** widen `servo_map.yaml` joint limits. The observed pose should first be captured and reviewed. Any limit change should be based on verified mechanical travel/calibration, because the native driver converts commanded radians to steps with software joint-limit clamping.

## Important after an aborted v2.4.3 capture

The error occurs after `disable_torque_all` has been called. The runner does not automatically re-enable torque on this error path. Keep the robot mechanically supported and verify `/st3215_driver/telemetry.torque_enabled_state` before any further action.
