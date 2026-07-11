# ST3215 Calibration Workflow

This workflow updates servo `center_step` values from a known physical reference pose while preserving the driver as the sole normal UART owner.

Keep the robot securely supported and keep the physical servo-power disconnect reachable.

## 1. Start the commissioning driver with writes disabled

```bash
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=commissioning \
  enable_writes:=false
```

Verify:

```bash
ros2 run lgh_st3215_tools st3215_preflight \
  --mode commissioning \
  --expect-writes false
```

## 2. Place the robot in the known reference pose

Use the same physical reference pose represented by the calibrated default joint values. Do not capture centers from an approximate unsupported pose.

Print the current default reference:

```bash
ros2 run lgh_st3215_tools print_default_pose \
  --servo-map ~/littlegreen_ros2_ws/src/lgh_st3215_driver/config/servo_map.yaml
```

## 3. Capture a center proposal

```bash
ros2 run lgh_st3215_tools capture_calibration \
  --servo-map ~/littlegreen_ros2_ws/src/lgh_st3215_driver/config/servo_map.yaml
```

The tool requires fresh raw feedback and refuses incompatible write states unless explicitly overridden.

Review:

```text
center_step_proposal.yaml
capture_summary.yaml
capture_summary.txt
```

Default correction categories:

```text
|correction| <= 25 steps      fine software correction
26..100 steps                 inspect horn alignment
>100 steps                    mechanical re-index recommended
```

`RANGE_CONFLICT`, `UNSTABLE_CAPTURE`, and unreviewed large corrections are blocking conditions.

## 4. Dry-run the map update

```bash
ros2 run lgh_st3215_tools apply_calibration \
  calibration_reports/<timestamp>/center_step_proposal.yaml \
  --source-servo-map ~/littlegreen_ros2_ws/src/lgh_st3215_driver/config/servo_map.yaml
```

Review the unified diff. The modifying path is intentionally explicit; the tool will not silently edit the installed package-share copy.

## 5. Apply after review

```bash
ros2 run lgh_st3215_tools apply_calibration \
  calibration_reports/<timestamp>/center_step_proposal.yaml \
  --source-servo-map ~/littlegreen_ros2_ws/src/lgh_st3215_driver/config/servo_map.yaml \
  --apply
```

The tool checks the captured source-map identity, creates a timestamped backup, and atomically replaces the approved `center_step` values.

## 6. Rebuild and relaunch feedback-only

```bash
cd ~/littlegreen_ros2_ws
colcon build --packages-select lgh_st3215_driver --symlink-install
source install/setup.bash
```

Restart:

```bash
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=commissioning \
  enable_writes:=false
```

Verify with the robot still in the reference pose:

```bash
ros2 run lgh_st3215_tools verify_calibration \
  --servo-map ~/littlegreen_ros2_ws/src/lgh_st3215_driver/config/servo_map.yaml
```

Default thresholds:

```text
PASS  |error| <= 0.02 rad
WARN  |error| <= 0.05 rad
FAIL  |error| >  0.05 rad
```

Expected default joint vector:

```text
[0.0, 0.0, -0.1, 0.4, -0.3, 0.0,
 0.0, 0.0, -0.1, 0.4, -0.3, 0.0]
```

## 7. Guarded write-enabled verification

Stop the feedback-only driver. With the robot still supported:

```bash
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=commissioning \
  enable_writes:=true \
  default_pose_move_duration_sec:=8.0
```

Run the keyboard-abort console:

```bash
ros2 run lgh_st3215_tools pose_console
```

During the ramp:

```text
SPACE, q/Q, a/A, ESC   abort and hold the latest measured pose
Ctrl+C                 request abort, then exit
```

After a successful move, the internal pose override remains active until explicitly released.

## 8. Preserve the result

Keep together:

- the preflight report;
- hardware snapshot;
- calibration capture;
- source-map backup;
- updated `servo_map.yaml`;
- verification report;
- operator notes about the physical reference pose.

For policy shadow after calibration, stop the write-enabled driver and return to:

```bash
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=runtime_safe \
  enable_writes:=false
```
