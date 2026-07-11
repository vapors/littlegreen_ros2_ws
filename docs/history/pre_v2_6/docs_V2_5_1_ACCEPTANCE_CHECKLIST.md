# v2.5.1 Orange Pi acceptance checklist

Use the robot's mechanical support and keep the hardware power disconnect reachable throughout commissioning.

## 1. Build

```bash
cd ~/berkeley_ros2_ws
rm -rf build/bhl_st3215_driver build/bhl_st3215_tools \
       build/bhl_st3215_maintenance build/bhl_imu_tools \
       build/berkeley_biped_pkg
colcon build --packages-select \
  bhl_st3215_driver bhl_st3215_tools bhl_st3215_maintenance \
  bhl_imu_tools berkeley_biped_pkg \
  --symlink-install
source install/setup.bash
```

Expected: all five packages build without warnings that indicate missing installed files or unresolved driver-core symbols.

## 2. Commissioning profile, writes disabled

```bash
ros2 launch bhl_st3215_driver bhl_st3215_driver.launch.py \
  profile:=commissioning enable_writes:=false
```

In another terminal:

```bash
ros2 run bhl_st3215_tools st3215_preflight \
  --mode feedback --expect-writes false
```

Expected exit code: `0`.

Verify topic surface:

```bash
ros2 topic list | grep -E 'joint_states|joint_feedback_age|st3215_driver'
ros2 topic hz /joint_states
ros2 topic hz /st3215_driver/telemetry
```

Expected: `/joint_states` and telemetry remain near 50 Hz, with raw position/speed topics present.

## 3. Runtime-safe profile, writes disabled

Stop the driver and relaunch:

```bash
ros2 launch bhl_st3215_driver bhl_st3215_driver.launch.py \
  profile:=runtime_safe enable_writes:=false
```

```bash
ros2 run bhl_st3215_tools st3215_preflight \
  --mode runtime --expect-writes false
```

Expected exit code: `0`.

Expected topics present:

```text
/joint_states
/joint_feedback_age_ms
/st3215_driver/diagnostics
```

Expected topics absent:

```text
/st3215_driver/telemetry
/st3215_driver/raw_position_steps
/st3215_driver/raw_speed
/servo_target_steps_debug
```

Confirm the driver diagnostic still reports full feedback readiness, voltage/current/temperature state, and error counters.

## 4. Maintenance exclusion

With the driver still running:

```bash
ros2 run bhl_st3215_maintenance bus_scan --first-id 1 --last-id 12
printf 'exit=%s\n' "$?"
```

Expected: refusal and exit code `3` because the driver owns the UART lock.

Stop the driver, keep the robot bus powered, then run:

```bash
ros2 run bhl_st3215_maintenance verify_ids
```

Expected: all configured IDs reply and exit code `0`.

Read-only backup:

```bash
ros2 run bhl_st3215_maintenance backup_control_tables
```

Expected: one YAML record per configured servo and no writes to EEPROM.

## 5. Existing tool equivalence

Relaunch commissioning feedback-only and verify the migrated tools:

```bash
ros2 run bhl_st3215_tools print_default_pose
ros2 run bhl_st3215_tools verify_calibration
ros2 run bhl_st3215_tools servo_identification --help
ros2 run bhl_st3215_tools standing_characterization --help
```

For the first motion regression, use the already validated supported one-joint workflow and compare generated summaries to the v2.5 baseline.

## 6. IMU tools against current micro-ROS source

```bash
ros2 run bhl_imu_tools imu_preflight
ros2 run bhl_imu_tools stationary_characterization --duration-sec 20
ros2 run bhl_imu_tools orientation_audit --pose neutral
```

Expected: canonical topic/frame/timestamp checks pass before any policy shadow work. Preserve the result directories as the baseline for the future direct I2C/SPI driver.

## 7. Policy shadow

Run the driver separately in `runtime_safe`, writes disabled. Then:

```bash
ros2 launch berkeley_biped_pkg policy_shadow.launch.py
```

Verify:

```bash
ros2 topic info /desired_position --verbose
ros2 topic info /policy_shadow/desired_position --verbose
ros2 topic echo /policy_status --once
```

Expected:

- no publisher from `berkeley_biped_node` on `/desired_position`;
- one policy publisher on `/policy_shadow/desired_position`;
- readiness/debug behavior identical to live policy processing;
- no PD controller launched by `policy_shadow.launch.py`;
- servo writes remain disabled.

## 8. Write-enabled regression

Only after the preceding checks pass, repeat the existing guarded current-pose hold/default-pose workflows with `profile:=commissioning enable_writes:=true`. v2.5.1 does not authorize policy-live or aggressive outer-PD testing by itself.
