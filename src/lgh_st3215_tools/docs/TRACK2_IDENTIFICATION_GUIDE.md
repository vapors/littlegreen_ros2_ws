# Track 2 Servo Identification Guide

Use this workflow for supported-robot, one-joint ST3215 identification with the learned policy and downstream controller stopped.

The authoritative time series is `/st3215_driver/telemetry`, which provides one coherent snapshot per completed native bus cycle.

## Safety boundary

- Securely support the robot.
- Keep the physical servo-power disconnect immediately reachable.
- Test one joint at a time.
- Keep `littlegreen_biped_node` and `pd_controller_node` stopped for direct identification.
- Treat `/st3215_driver/hold_current_pose` as a software position hold, not an electrical emergency stop.

## 1. Build and source

```bash
cd ~/littlegreen_ros2_ws
colcon build --packages-select \
  lgh_st3215_driver \
  lgh_st3215_tools \
  --symlink-install

source install/setup.bash
```

## 2. Feedback-only validation

Start the commissioning profile with writes disabled:

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

Inspect timing and telemetry:

```bash
ros2 topic hz /st3215_driver/telemetry
ros2 topic echo /st3215_driver/telemetry --once
ros2 topic echo /st3215_driver/diagnostics --once
```

Do not proceed if the bus cannot sustain a stable cycle, feedback is stale, protocol/error counters are growing, or telemetry snapshots are dropping.

## 3. Start the direct-identification driver

Stop the feedback-only process and relaunch:

```bash
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=commissioning \
  enable_writes:=true
```

Run preflight:

```bash
ros2 run lgh_st3215_tools st3215_preflight \
  --mode commissioning \
  --expect-writes true
```

Release a prior pose override only after confirming the robot is supported and the test anchor is correct:

```bash
ros2 service call \
  /st3215_driver/release_pose_override \
  std_srvs/srv/Trigger '{}'
```

## 4. First conservative test

```bash
ros2 run lgh_st3215_tools servo_identification \
  --joint leg_left_ankle_pitch_joint \
  --mode step \
  --direction both \
  --amplitude-rad 0.02 \
  --support-condition "securely suspended, feet unloaded"
```

Abort controls:

```text
SPACE
q / Q
ESC
Ctrl+C
```

On abort, the runner requests the driver's current-pose hold. The pose override remains active until explicitly released.

## 5. Recommended progression

1. Left ankle pitch, ±0.02 rad.
2. Right ankle pitch, ±0.02 rad.
3. Compare bilateral timing, velocity, load, current, and steady-state error.
4. Run 0.02 / 0.05 / 0.10 rad step sweeps when the small tests are clean.
5. Run deadband staircases.
6. Run slow triangle tests for hysteresis/backlash visualization.
7. Run known-load static holds when force and lever-arm data are available.
8. Move to knee pitch, then hip pitch.
9. Evaluate outer-loop behavior only as a separate, controlled experiment.

## 6. Step sweep

```bash
ros2 run lgh_st3215_tools servo_identification \
  --joint leg_left_ankle_pitch_joint \
  --mode step_sweep \
  --direction both \
  --amplitudes-rad 0.02,0.05,0.10 \
  --support-condition "securely suspended, feet unloaded"
```

Do not extend to larger amplitudes until the smaller sweep is reviewed.

## 7. Available modes

```text
step
step_sweep
deadband_staircase
triangle
hold_under_load
```

Use the executable help for the current full option list:

```bash
ros2 run lgh_st3215_tools servo_identification --help
```

## 8. Output

Each run creates a dataset containing at least:

```text
timeseries.csv
metadata.yaml
summary.yaml
summary.txt
```

Preserve the associated preflight report and hardware snapshot with the dataset.
