# Berkeley-Humanoid-Lite ST3215 Calibration Workflow — v2.2

This workflow keeps robot-specific calibration in `servo_map.yaml`. It does not
rewrite hidden ST3215 center offsets in servo EEPROM.

## Safety assumptions

- Robot is securely supported.
- Physical power disconnect is immediately reachable.
- Calibration capture and verification are performed with driver writes disabled.
- Only the guarded default-pose move is performed with writes enabled.
- The default-pose keyboard abort is a software motion abort/hold, not a torque-off E-stop.

## 0. Build the updated package

```bash
cd ~/berkeley_ros2_ws
colcon build --packages-select bhl_st3215_driver --symlink-install
source install/setup.bash
```

## 1. Mechanically position the robot in the known training-default pose

Print the exact reference table from the active servo map:

```bash
ros2 run bhl_st3215_tools print_default_pose \
  --servo-map ~/berkeley_ros2_ws/src/bhl_st3215_driver/config/servo_map.yaml
```

The training-default joint pose is:

```text
[0.0, 0.0, -0.1, 0.4, -0.3, 0.0,
 0.0, 0.0, -0.1, 0.4, -0.3, 0.0]
```

With the uncalibrated all-2048 center map, the nominal servo-step references are:

```text
[2048, 2048, 2113, 1787, 1852, 2048,
 2048, 2048, 1983, 2309, 2244, 2048]
```

Use those values for coarse horn indexing. Align one joint at a time. Do not
remove all horns simultaneously.

## 2. Launch feedback-only and capture a calibration proposal

```bash
ros2 launch bhl_st3215_driver \
  bhl_st3215_driver.launch.py \
  enable_writes:=false
```

Confirm raw state is live:

```bash
ros2 topic hz /st3215_driver/raw_position_steps
ros2 topic echo /st3215_driver/raw_position_steps --once
```

Capture 250 samples (about five seconds at 50 Hz):

```bash
ros2 run bhl_st3215_tools capture_calibration \
  --servo-map ~/berkeley_ros2_ws/src/bhl_st3215_driver/config/servo_map.yaml
```

Type `CAPTURE` only after the robot is physically aligned and stable.

The tool creates a timestamped directory containing:

```text
center_step_proposal.yaml
servo_map.proposed.yaml
calibration_summary.csv
calibration_report.txt
```

The capture tool never modifies servo EEPROM and never modifies the source map.

## 3. Review the proposal and mechanical alignment flags

Statuses:

```text
FINE_SOFTWARE_CORRECTION
INSPECT_MECHANICAL_ALIGNMENT
MECHANICAL_REINDEX_RECOMMENDED
RANGE_CONFLICT
UNSTABLE_CAPTURE
```

Default thresholds:

```text
|correction| <= 25 steps      fine software correction
26..100 steps                 inspect horn alignment
>100 steps                    mechanical re-index recommended
```

`RANGE_CONFLICT` and `UNSTABLE_CAPTURE` are blocking. Large mechanical corrections
are also refused by the apply tool unless explicitly overridden after review.

## 4. Dry-run and apply the center_step update

Dry-run first:

```bash
ros2 run bhl_st3215_tools apply_calibration \
  calibration_reports/<timestamp>/center_step_proposal.yaml \
  --source-servo-map ~/berkeley_ros2_ws/src/bhl_st3215_driver/config/servo_map.yaml
```

Review the unified diff. Then apply:

```bash
ros2 run bhl_st3215_tools apply_calibration \
  calibration_reports/<timestamp>/center_step_proposal.yaml \
  --source-servo-map ~/berkeley_ros2_ws/src/bhl_st3215_driver/config/servo_map.yaml \
  --apply
```

Before editing, the tool verifies the captured source-map SHA-256 and joint
identity fields. On apply, it creates a timestamped backup next to the source
map and atomically replaces only `center_step` values.

## 5. Rebuild/relaunch and verify feedback-only

```bash
cd ~/berkeley_ros2_ws
colcon build --packages-select bhl_st3215_driver --symlink-install
source install/setup.bash
```

Relaunch feedback-only:

```bash
ros2 launch bhl_st3215_driver \
  bhl_st3215_driver.launch.py \
  enable_writes:=false
```

With the robot still physically in the known default pose:

```bash
ros2 run bhl_st3215_tools verify_calibration \
  --servo-map ~/berkeley_ros2_ws/src/bhl_st3215_driver/config/servo_map.yaml
```

Default verification thresholds:

```text
PASS  |error| <= 0.02 rad
WARN  |error| <= 0.05 rad
FAIL  |error| >  0.05 rad
```

Expected `/joint_states.position` baseline:

```text
[0.0, 0.0, -0.1, 0.4, -0.3, 0.0,
 0.0, 0.0, -0.1, 0.4, -0.3, 0.0]
```

## 6. Enable writes and verify startup hold

Stop the feedback-only process and launch:

```bash
ros2 launch bhl_st3215_driver \
  bhl_st3215_driver.launch.py \
  enable_writes:=true \
  default_pose_move_duration_sec:=8.0
```

Before any policy stack is started, inspect:

```bash
ros2 topic echo /st3215_driver/diagnostics --once
```

The driver should be holding the measured startup pose with full fresh feedback.

## 7. Guarded move to the calibrated default pose

Use the keyboard-abort console:

```bash
ros2 run bhl_st3215_tools pose_console
```

Type `MOVE` to start. During the ramp:

```text
SPACE   abort and hold latest measured pose
q/Q     abort and hold latest measured pose
a/A     abort and hold latest measured pose
ESC     abort and hold latest measured pose
Ctrl+C  request abort, then exit
```

After completion, the internal pose override remains active. Policy/PD commands
are ignored until explicitly released.

## 8. Continue policy diagnostics and outer-PD tuning before policy release

While the driver still holds the calibrated default pose, start the policy stack
with:

```bash
ros2 launch berkeley_biped_pkg \
  berkeley_biped_launch.py \
  controller_mode:=safety_only
```

Collect:

```text
/policy_debug/observation
/policy_debug/raw_action
/policy_debug/clipped_raw_action
/policy_debug/target_unclipped
/policy_debug/target_clipped
/policy_debug/saturation_mask
```

The next controller-development sequence is:

```text
calibrated default-pose baseline
        ↓
policy startup diagnostics
        ↓
manual/reference outer-PD tuning
        ↓
small controlled joint motions
        ↓
whole-body supported tests
        ↓
only then release policy commands to the servo driver
```

Do not release the pose override merely because the policy readiness gate is open.
The policy debug data must first show acceptable behavior from the calibrated baseline.
