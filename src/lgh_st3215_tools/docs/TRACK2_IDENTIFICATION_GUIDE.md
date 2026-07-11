# Track 2 Servo Identification Guide — v2.4 telemetry foundation

This workspace is for supported-robot, one-joint identification with the learned
policy stopped. It adds cycle-synchronous native-driver telemetry so transient and
load measurements are based on actual bus-cycle timing instead of reconstructing
rows from several independently scheduled ROS topics.

## v2.4 changes

- `/st3215_driver/telemetry` publishes one coherent snapshot for each completed
  native bus cycle, nominally 50 Hz.
- The driver reads the STS3215 feedback window `0x38..0x46` as one contiguous
  15-byte block per servo. Each successful sample contains:
  - position;
  - speed;
  - signed load/duty-cycle proxy;
  - voltage;
  - temperature;
  - servo status;
  - moving flag;
  - current.
- Telemetry includes command receipt, SyncWrite, feedback sample, and cycle timing
  in the Linux steady/monotonic clock domain.
- `servo_identification` logs telemetry as the authoritative time-series
  source. `/joint_states` and diagnostics remain safety/compatibility inputs.
- Step summaries now decompose end-to-end lag into:
  - runner publish -> driver receipt;
  - driver receipt -> first matching SyncWrite start;
  - SyncWrite duration;
  - first matching SyncWrite end -> first sustained motion.
- The CSV also logs load, voltage, temperature, status/moving, raw current, and
  decoded amperes for the selected joint.

The current-pose latch remains a **software position hold**, not a torque-off
E-stop. Keep the hardware power disconnect immediately accessible.

## Why load and current are both logged

The STS3215 `Current load` register is a signed drive-output duty-cycle proxy with
0.001 scaling. It is useful for control-effort and asymmetry analysis, but it is
not a current measurement. The `Current current` register is a separate direct
measurement with 6.5 mA/count scaling.

For v2.4, no current is inferred from load. The runner logs both signals and reports
an empirical `abs(load)` versus `abs(current)` Pearson correlation for each run.
A later calibrated current/torque proxy can be fitted from measured data if useful,
but it should be conditioned on direction, speed, supply voltage, temperature,
and operating point rather than treated as a universal conversion.

## Build

```bash
cd ~/littlegreen_ros2_ws
colcon build --packages-select lgh_st3215_driver --symlink-install
source install/setup.bash
```

The package now generates `lgh_st3215_driver/msg/ServoTelemetry`, so rebuild and
source the workspace before running the identification tool.

## Feedback-only validation before motion

Start the driver without writes:

```bash
ros2 launch lgh_st3215_driver \
  lgh_st3215_driver.launch.py \
  enable_writes:=false
```

Check the telemetry rate and one sample:

```bash
ros2 topic hz /st3215_driver/telemetry
ros2 topic echo /st3215_driver/telemetry --once
ros2 topic echo /st3215_driver/diagnostics --once
```

Before write-enabled testing, compare the new 15-byte feedback sweep timing against
the previous baseline:

```text
cycle_rate_hz
cycle_work_us_mean / p99 / max
feedback_sweep_us_mean / p99 / max
read_rtt_us_mean / p99 / max
read_timeout_count
checksum_error_count
cycles_over_period_count
telemetry_dropped_count
```

Do not proceed to motion if the bus no longer maintains a stable ~50 Hz cycle or
if telemetry snapshots are dropping during the armed test.

## Required launch state for direct identification

Launch only the native driver. Do not launch `littlegreen_biped_pkg` or the outer PD
controller for direct servo identification.

```bash
ros2 launch lgh_st3215_driver \
  lgh_st3215_driver.launch.py \
  enable_writes:=true
```

Release a prior pose override deliberately:

```bash
ros2 service call \
  /st3215_driver/release_pose_override \
  std_srvs/srv/Trigger '{}'
```

## First recommended test: left ankle pitch ±0.02 rad

```bash
ros2 run lgh_st3215_tools servo_identification \
  --joint leg_left_ankle_pitch_joint \
  --mode step \
  --direction both \
  --amplitude-rad 0.02 \
  --support-condition "securely suspended, feet unloaded"
```

Abort keys during motion:

```text
SPACE  q/Q  ESC  Ctrl+C
```

On abort the runner requests `/st3215_driver/hold_current_pose`; the driver captures
the latest measured 12-joint pose and asserts its internal override. Press `x` to
exit the abort-hold console. The driver override remains active until explicitly
released.

## Recommended test progression

1. Left ankle pitch ±0.02 rad.
2. Right ankle pitch ±0.02 rad.
3. Compare bilateral timing, speed, load, current, and steady-state error.
4. Repeat 0.02 / 0.05 / 0.10 rad step sweeps if the small tests are clean.
5. Run deadband staircases.
6. Run slow triangle tests for hysteresis/backlash visualization.
7. Run known-load static holds for effective stiffness.
8. Move to knee pitch and hip pitch.
9. Only then begin outer-PD manual-reference tuning.

## Velocity-saturation sweep

```bash
ros2 run lgh_st3215_tools servo_identification \
  --joint leg_left_ankle_pitch_joint \
  --mode step_sweep \
  --direction both \
  --amplitudes-rad "0.02 0.05 0.10"
```

Do not extend to 0.15 or 0.20 rad until smaller sweeps are clean.

## Deadband staircase

```bash
ros2 run lgh_st3215_tools servo_identification \
  --joint leg_left_ankle_pitch_joint \
  --mode deadband_staircase \
  --deadband-offsets-rad "0.002 0.005 0.010 0.020"
```

The sequence is:

```text
0
+0.002 +0.005 +0.010 +0.020
0
-0.002 -0.005 -0.010 -0.020
0
```

## Slow triangle test

```bash
ros2 run lgh_st3215_tools servo_identification \
  --joint leg_left_ankle_pitch_joint \
  --mode triangle \
  --triangle-amplitude-rad 0.02 \
  --triangle-frequency-hz 0.10 \
  --triangle-cycles 2
```

## Loaded stiffness hold

A stiffness estimate requires known applied torque. Example:

```bash
ros2 run lgh_st3215_tools servo_identification \
  --joint leg_left_ankle_pitch_joint \
  --mode hold_under_load \
  --load-force-n 5.0 \
  --lever-arm-m 0.10 \
  --load-prepare-sec 5.0 \
  --load-hold-sec 10.0 \
  --notes "force applied approximately perpendicular to lever arm"
```

The summary reports local effective stiffness only when measured deflection exceeds
the configured minimum. It represents the combined servo controller, transmission,
joint mechanics, and supported load condition at that pose.

## Outer-PD manual-reference path

Launch the native driver and `pd_controller_node` without the policy node, then use:

```bash
--command-path outer
```

The runner publishes manual references to `/desired_position`, expects the PD node
to own `/servo_target_radians`, and still rejects a running policy node or competing
publisher.

## v2.4 output schema highlights

`timeseries.csv` is driven by `/st3215_driver/telemetry` and includes:

```text
telemetry_cycle_index
cycle_start_monotonic_ns
cycle_end_monotonic_ns
telemetry_callback_delay_ms
cycle_work_us
feedback_sweep_us
read_start_index
runner_command_sequence
driver_command_sequence
driver_written_command_sequence
driver_command_receipt_monotonic_ns
write_due / write_attempted / write_ok
sync_write_start_monotonic_ns
sync_write_end_monotonic_ns
sync_write_us
q_ref_runner_rad
q_ref_driver_rad
target_step
q_meas_rad
qdot_meas_rad_s
sample_monotonic_ns
feedback_age_ms
raw_position_step
raw_speed
raw_load
load_ratio
voltage_v
temperature_c
servo_status
moving
raw_current
current_a
read_ok
status_error
telemetry_dropped_count
```

The summary reports, where applicable:

```text
runner publish -> driver receipt
receiver -> first matching SyncWrite start
SyncWrite duration
first matching SyncWrite end -> first sustained motion
end-to-end publish -> motion lag
10–90% rise time
63.2% response time
settling time
overshoot
steady-state error
peak velocity
static gain
damping-ratio estimate
natural-frequency estimate
velocity plateau candidate
deadband candidate
effective static stiffness under known load
peak/median absolute current
peak/median absolute load ratio
voltage and temperature range
abs(load) vs abs(current) empirical correlation
```

## v2.4.2 Track 1 maximum-envelope baseline

The shipped v2.4.2 servo map uses a fixed ST3215 position-mode profile of:

```text
speed = 0
acceleration = 0
```

These values are included in every SyncWrite and are not dynamically changed by the
policy or outer controller. The driver reports the resolved profile in diagnostics and
telemetry. The guarded runner refuses to arm against a different profile unless
`--allow-nonmax-motion-profile` is explicitly supplied for an intentional profile study.

Before collecting the full fresh baseline, validate the installed hardware with a
conservative suspended ankle sweep:

```bash
ros2 run lgh_st3215_tools servo_identification \
  --joint leg_left_ankle_pitch_joint \
  --mode step_sweep \
  --direction both \
  --amplitudes-rad "0.02,0.05" \
  --test-center-offset-rad 0.05 \
  --support-condition "securely suspended, feet unloaded"
```

After confirming expected behavior, collect the planned larger-amplitude baseline under
the same unchanged profile.
