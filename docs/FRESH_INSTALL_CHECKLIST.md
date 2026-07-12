# Fresh Install and Hardware Commissioning Checklist

Use this checklist after completing [`INSTALL_ORANGE_PI.md`](INSTALL_ORANGE_PI.md).

Keep the robot mechanically supported throughout initial bring-up and keep the physical servo-power disconnect immediately reachable.

## Gate 0 — clean environment

Open a new terminal and confirm the workspace is active:

```bash
ros2 pkg prefix lgh_st3215_driver
```

**Pass:** the path is under `~/littlegreen_ros2_ws/install/`.

Check that no legacy project overlay is active:

```bash
ros2 pkg list | grep -E '^(bhl_|berkeley_biped|lilgreen_)'
```

**Pass:** no output.

## Gate 1 — software verification

```bash
cd ~/littlegreen_ros2_ws
./scripts/verify_install.sh --software-only
```

**Pass:** exit code `0`, or exit code `2` only for an understood warning.

Record:

```bash
cat VERSION
printf 'workspace=%s\n' "$LITTLEGREEN_ROS2_WS"
printf 'onnxruntime=%s\n' "$ONNXRUNTIME_DIR"
ros2 pkg prefix lgh_st3215_driver
ros2 pkg prefix littlegreen_biped_pkg
```

## Gate 2 — UART and permissions

With servo power off:

```bash
ls -l /dev/ttyS3
id -nG | tr ' ' '\n' | grep '^dialout$'
test -r /dev/ttyS3 && test -w /dev/ttyS3 && echo UART_ACCESS_OK
```

**Pass:** `/dev/ttyS3` exists, the current login includes `dialout`, and access is readable/writable.

## Gate 3 — offline read-only maintenance

Keep the runtime driver stopped. Apply servo bus power only when the robot is safely supported.

```bash
ros2 run lgh_st3215_maintenance bus_scan --first-id 1 --last-id 12
ros2 run lgh_st3215_maintenance verify_ids
```

**Pass:** all expected IDs respond once and no duplicate or unexpected IDs are reported.

Optional backup:

```bash
ros2 run lgh_st3215_maintenance backup_control_tables
```

The maintenance package is read-only. It performs no EEPROM writes.

## Gate 4 — feedback-only commissioning driver

Start the driver with writes disabled:

```bash
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=commissioning \
  enable_writes:=false
```

In another terminal:

```bash
ros2 node list
ros2 topic list
```

Expected commissioning topics include:

```text
/joint_states
/joint_feedback_age_ms
/st3215_driver/diagnostics
/st3215_driver/raw_position_steps
/st3215_driver/raw_speed
/st3215_driver/telemetry
/servo_target_steps_debug
```

## Gate 5 — feedback preflight

```bash
ros2 run lgh_st3215_tools st3215_preflight \
  --mode feedback \
  --expect-writes false
```

**Pass:** `ST3215 PREFLIGHT: PASS`, exit code `0`.

Preserve the report path printed by the tool.

## Gate 6 — commissioning profile audit

```bash
ros2 run lgh_st3215_tools st3215_preflight \
  --mode commissioning \
  --expect-writes false
```

**Pass:** the profile is `commissioning`, laboratory topics are present, the policy node is absent, error counters do not grow, and the command authority count is acceptable.

Capture a hardware snapshot:

```bash
ros2 run lgh_st3215_tools hardware_snapshot
```

## Gate 7 — feedback quality

Check rates:

```bash
ros2 topic hz /joint_states
ros2 topic hz /joint_feedback_age_ms
ros2 topic hz /st3215_driver/telemetry
```

Expected nominal rate: approximately `50 Hz`.

Inspect one diagnostic message:

```bash
ros2 topic echo /st3215_driver/diagnostics --once
```

Confirm:

- `feedback_ready=true`;
- `writes_enabled=false`;
- cycle rate is near 50 Hz;
- cycle-work p99 remains below the configured preflight limit;
- no timeout, checksum, malformed-frame, wrong-ID, I/O, or servo-status error counter is increasing;
- feedback ages are bounded.

## Gate 8 — IMU boundary

With the current IMU source publishing `/imu/data`:

```bash
ros2 run lgh_imu_tools imu_preflight
ros2 run lgh_imu_tools stationary_characterization --duration-sec 20
```

**Pass:** rate, timestamps, freshness, quaternion norm, finite values, and stationary measurements satisfy the configured contract.

Repeat the orientation audit whenever the sensor, mounting, bus, or driver changes.

## Gate 9 — runtime-safe profile

Stop the commissioning driver and relaunch:

```bash
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=runtime_safe \
  enable_writes:=false
```

Run:

```bash
ros2 run lgh_st3215_tools st3215_preflight \
  --mode runtime \
  --expect-writes false
```

**Pass:** `/joint_states`, `/joint_feedback_age_ms`, and diagnostics remain available while raw position, raw speed, cycle telemetry, and target debug publishers are absent.

## Gate 10 — policy shadow

Keep the driver in `runtime_safe` with writes disabled:

```bash
ros2 launch littlegreen_biped_pkg policy_shadow.launch.py
```

Verify:

```bash
ros2 topic info /desired_position --verbose
ros2 topic hz /policy_shadow/desired_position
ros2 topic echo /policy_status --once
```

**Pass:**

- the policy publishes `/policy_shadow/desired_position`;
- it does not publish `/desired_position`;
- policy readiness/status is observable;
- no servo command authority is created by shadow mode.

For a deployed action-contract-v3 bundle, startup must report successful contract validation against `joint_map.yaml` and successful ONNX checksum verification. Complete the live sequence only through [`LIVE_POLICY_DEPLOYMENT.md`](LIVE_POLICY_DEPLOYMENT.md).

## Gate 11 — guarded write-enabled work

Only after Gates 0–10 pass, return to the `commissioning` profile and explicitly enable writes for a planned guarded operation:

```bash
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=commissioning \
  enable_writes:=true
```

Before any motion:

- confirm mechanical support;
- confirm the power disconnect is reachable;
- confirm exactly one intended command publisher;
- confirm fresh complete feedback;
- confirm the intended tool and motion amplitude;
- keep the policy disconnected.

Write-enabled identification, calibration verification, and pose operations must follow their dedicated workflow documentation.
