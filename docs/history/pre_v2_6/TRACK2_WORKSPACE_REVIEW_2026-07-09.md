# Track 2 Workspace Review — 2026-07-09

Reviewed inputs:

- `berkeley_ros2_ws_v2_2_src.zip`
- `Berkeley-Humanoid-Lite_v1_2_2.zip`
- `TRACK2_SERVO_IDENTIFICATION_HANDOFF.md`
- ST3215 protocol/manual references supplied with the project

## 1. Native driver is the correct base for identification

The `bhl_st3215_driver` architecture already has the important pieces needed for
repeatable one-joint tests:

- 50 Hz bus cycle and 50 Hz command path;
- `/joint_states` plus per-joint feedback age;
- canonical-order raw position steps and raw speed;
- write-disabled default at launch;
- startup hold from measured physical pose;
- explicit guarded move-to-default service;
- internal pose override that blocks external targets;
- driver diagnostics containing command counts, command age, feedback readiness,
  write state, timing statistics, and bus error counters.

The identification runner therefore uses the ROS command/feedback interface and
does not open `/dev/ttyS3` itself. This preserves a single bus owner.

## 2. Calibration-source mismatch found and corrected

The Track 2 handoff says calibration was applied successfully with centers:

```text
[2043, 2029, 2120, 2059, 2022, 2019,
 2037, 2046, 2120, 2048, 2074, 2057]
```

The supplied v2.2 workspace archive still contained `center_step: 2048` for all
12 joints in `bhl_st3215_driver/config/servo_map.yaml`.

This Track 2 update places the handoff's calibrated values into the native servo
map. The runner also refuses an all-2048 map by default to catch accidental use
of an un-applied calibration map.

## 3. Direct servo identification and outer-PD tuning must be separate paths

For servo/joint identification, use:

```text
servo_identification_runner
  -> /servo_target_radians
  -> native ST3215 driver
```

Do not run `pd_controller_node` during those direct tests because it also
publishes `/servo_target_radians`.

For outer-PD tuning, use:

```text
servo_identification_runner
  -> /desired_position
  -> pd_controller_node
  -> /servo_target_radians
  -> native ST3215 driver
```

The runner enforces these graph ownership rules and refuses a running
`berkeley_biped_node`.

## 4. Abort handling needed a driver-level latch

The existing `abort_pose_move` service only applies when a default-pose move or
pose override is active. A free-running manual identification test therefore had
no native service that could capture current feedback and assert the internal
override.

This update adds:

```text
/st3215_driver/hold_current_pose    std_srvs/srv/Trigger
```

Behavior:

1. stop/join any active guarded pose ramp;
2. copy the latest complete fresh measured 12-joint pose into the command buffer;
3. assert the internal pose override;
4. block external `/servo_target_radians` commands until explicit release.

If fresh complete feedback is unavailable, the service still asserts the
internal override and leaves the last bus command held. This is a software
position hold, not a torque-off E-stop.

## 5. Policy workspace metadata is older than Track 1 v1.2.2

The ROS workspace's current `policy_latest.yaml` / `joint_map.yaml` still describe
an earlier deployment contract with:

```text
policy_dt: 0.04
control_dt: 0.04
action_scale: 0.25
```

The supplied Track 1 v1.2.2 source config uses 200 Hz physics with decimation 4,
which is a 50 Hz policy/action update. Its v1.2 action term is also a bounded
normalized action mapped asymmetrically from default pose toward the full
hardware limits.

This mismatch does not block Track 2 because the policy must remain disconnected
for identification. Do not use the current ROS policy metadata to infer Track 1
v1.2.2 action timing or action semantics. Before later policy-on-hardware work,
re-export/sync the deployment policy YAML and ONNX artifact deliberately.

## 6. Timing measurements available now vs later instrumentation

The runner can measure:

```text
reference publication -> first sustained encoder motion
```

This includes ROS publish/transport, driver scheduling, bus-cycle phase, servo
internal response, and encoder detection threshold.

The current driver does not expose a per-command receipt timestamp/sequence ACK,
so this first version cannot separately measure:

```text
ROS publication -> driver callback receipt
```

and:

```text
driver receipt/command cycle -> physical motion
```

A later Phase G driver instrumentation update can add a compact command receipt
sequence/timestamp topic if that decomposition becomes important.

## 7. Velocity telemetry interpretation

Use both signals, but do not treat them as identical:

- `/joint_states.velocity` is derived from position delta and filtered by the
  native driver;
- `/st3215_driver/raw_speed` is the raw servo speed field.

The initial summary uses filtered `qdot` for response metrics and retains raw
speed in the CSV for saturation cross-checks and later scaling validation.

## 8. Loaded stiffness can start before current telemetry is added

Known-force / known-lever-arm tests can estimate local static effective stiffness
without servo current telemetry. The current runner supports those tests.

Current/load/voltage/temperature topics are still absent from the native driver.
They should be added later at low rates after the first motion datasets are clean,
not before the basic command/feedback identification path is validated.

## Recommended first hardware sequence

1. Build only `bhl_st3215_driver`.
2. Launch driver with writes enabled; do not launch policy or PD controller.
3. Confirm diagnostics and release any old pose override deliberately.
4. Run left ankle pitch `step`, ±0.02 rad.
5. Review CSV/summary before increasing amplitude.
6. Run right ankle pitch ±0.02 rad with identical support condition.
7. Compare lag, rise time, overshoot, settling, steady error, and peak velocity.
8. Only then run the conservative 0.02/0.05/0.10 rad sweep.
