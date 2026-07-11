# Fresh Install and Hardware Commissioning Checklist — v2.6.4

Use this checklist after completing [`INSTALL_ORANGE_PI.md`](INSTALL_ORANGE_PI.md).

The robot must be securely supported throughout initial bring-up. Keep the physical servo-power disconnect immediately reachable.

## Gate 0 — no old overlay

Open a new shell. The installer-managed `~/.bashrc` entry loads the environment automatically. For the current shell only, `source ~/.bashrc` is sufficient.

```bash
ros2 pkg list | grep -E '^(bhl_|berkeley_biped|lilgreen_)'
```

**Pass:** no output.

```bash
ros2 pkg list | grep -E '^(lgh_|littlegreen_)'
```

**Pass:** the six renamed packages are present.

## Gate 1 — software verification

```bash
cd ~/littlegreen_ros2_ws
./scripts/verify_install.sh --software-only
```

**Pass:** exit code 0, or exit code 2 only for an understood warning.

Record:

```bash
printf 'workspace=%s\n' "$LITTLEGREEN_ROS2_WS"
printf 'onnxruntime=%s\n' "$ONNXRUNTIME_DIR"
ros2 pkg prefix lgh_st3215_driver
ros2 pkg prefix littlegreen_biped_pkg
```

## Gate 2 — host UART and permissions

With servo power off:

```bash
ls -l /dev/ttyS3
id -nG | tr ' ' '\n' | grep '^dialout$'
```

**Pass:** `/dev/ttyS3` exists and the current login includes `dialout`.

Do not run the runtime driver and maintenance tools at the same time. Both use the same local exclusive lock.

## Gate 3 — source/configuration snapshot

```bash
ros2 run lgh_st3215_tools hardware_snapshot
```

Record the generated report location and preserve it with the commissioning log.

Verify that the source map is the intended calibrated map:

```bash
ros2 run lgh_st3215_tools print_default_pose \
  --servo-map ~/littlegreen_ros2_ws/src/lgh_st3215_driver/config/servo_map.yaml
```

Do not replace the calibrated map with an all-2048 placeholder.

## Gate 4 — offline read-only maintenance

Keep the runtime driver stopped. Power the servo bus only when mechanically safe.

```bash
ros2 run lgh_st3215_maintenance verify_ids
```

Optional full backup:

```bash
ros2 run lgh_st3215_maintenance backup_control_tables
```

**Pass:** expected IDs are found without duplicates or transaction errors.

The v2.6.4 maintenance package performs no EEPROM writes.

## Gate 5 — feedback-only runtime driver

Start only the native driver:

```bash
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=commissioning \
  enable_writes:=false
```

Do not launch:

```text
littlegreen_biped_node
pd_controller_node
servo identification runner
standing characterization runner
maintenance commands
```

In another shell:

```bash
ros2 topic hz /joint_states
ros2 topic hz /joint_feedback_age_ms
ros2 topic hz /st3215_driver/telemetry
ros2 topic echo /st3215_driver/diagnostics --once
```

Expected commissioning behavior:

```text
joint states                  approximately 50 Hz
feedback age                  approximately 50 Hz
cycle telemetry               approximately 50 Hz
diagnostics                   approximately 1 Hz
writes enabled                false
complete feedback             true after startup
```

## Gate 6 — ST3215 preflight

With the feedback-only driver still running:

```bash
ros2 run lgh_st3215_tools st3215_preflight \
  --mode feedback \
  --expect-writes false
```

**Pass:** process exit code 0 and YAML report status `PASS`.

A refusal or failure must be resolved rather than bypassed.

Common exit meanings:

```text
2  test completed but acceptance criteria failed
3  refused safety/precondition
4  ROS resource or feedback unavailable
5  configuration error
6  hardware/I/O error
7  operator abort
```

## Gate 7 — IMU source and contract

Bring up the current micro-ROS IMU path, or the future direct I2C/SPI driver, so it publishes `/imu/data`.

```bash
ros2 topic hz /imu/data
ros2 topic echo /imu/data --once
ros2 run lgh_imu_tools imu_preflight
```

Then collect a supported, stationary sample:

```bash
ros2 run lgh_imu_tools stationary_characterization --duration-sec 20
```

**Pass:** rate, freshness, finite values, quaternion norm, timestamp progression, and configured frame contract pass.

The future direct I2C/SPI migration must pass these same tools before it replaces the micro-ROS source.

## Gate 8 — runtime-safe profile audit

Stop the commissioning launch, then:

```bash
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=runtime_safe \
  enable_writes:=false
```

Confirm:

```bash
ros2 topic list | grep '^/st3215_driver/'
```

Expected runtime-safe surface:

```text
/st3215_driver/diagnostics       present
/st3215_driver/telemetry         absent
/st3215_driver/raw_position_steps absent
/st3215_driver/raw_speed         absent
```

`/joint_states` and `/joint_feedback_age_ms` remain present. The underlying full feedback read is unchanged.

## Gate 9 — policy shadow only

The Track 1 deployment bundle is not yet the v2.6.4 hardware authorization boundary. Use the packaged policy only for software/shadow validation.

Keep servo writes disabled:

```bash
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=runtime_safe \
  enable_writes:=false
```

In another shell:

```bash
ros2 launch littlegreen_biped_pkg policy_shadow.launch.py
```

Verify:

```bash
ros2 topic info /desired_position --verbose
ros2 topic info /policy_shadow/desired_position --verbose
ros2 topic hz /policy_shadow/desired_position
ros2 topic echo /policy_status --once
```

**Pass:** the policy publishes the shadow topic and does not create a policy publisher on `/desired_position`.

## Gate 10 — guarded write-enabled driver

Proceed only after feedback, configuration, and mechanical support are verified.

Stop all prior driver processes. Launch:

```bash
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=commissioning \
  enable_writes:=true \
  default_pose_move_duration_sec:=8.0
```

The driver must initialize its command from complete fresh measured feedback and hold the measured startup pose.

Inspect:

```bash
ros2 topic echo /st3215_driver/diagnostics --once
```

Do not release the pose override or connect a live policy in this gate.

## Gate 11 — guarded pose test

With the robot securely supported:

```bash
ros2 run lgh_st3215_tools pose_console
```

Use the documented exact arming phrase. During motion:

```text
SPACE / q / Q / a / A / ESC  abort and hold latest measured pose
Ctrl+C                         request abort and exit
```

Remember: this is a software position hold, not an electrical emergency stop.

## Gate 12 — release record

Preserve together:

```text
installer/terminal log
verify_install output
hardware snapshot
maintenance ID verification
ST3215 preflight report
IMU preflight report
stationary IMU characterization
driver diagnostics sample
policy shadow sample
```

Only after this baseline is archived should further Track 2 testing continue.
