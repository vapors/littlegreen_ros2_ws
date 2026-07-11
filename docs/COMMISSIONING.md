# Commissioning Sequence

This page is the concise commissioning path. Use [`FRESH_INSTALL_CHECKLIST.md`](FRESH_INSTALL_CHECKLIST.md) for the full acceptance record.

## 1. Verify software and host access

```bash
cd ~/littlegreen_ros2_ws
./scripts/verify_install.sh
```

Confirm `/dev/ttyS3`, `dialout`, and the active workspace overlay.

## 2. Inspect the bus offline

Stop the runtime driver:

```bash
ros2 run lgh_st3215_maintenance bus_scan --first-id 1 --last-id 12
ros2 run lgh_st3215_maintenance verify_ids
```

Maintenance directly owns the UART and is read-only.

## 3. Launch feedback-only commissioning

```bash
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=commissioning \
  enable_writes:=false
```

Run both preflight views:

```bash
ros2 run lgh_st3215_tools st3215_preflight \
  --mode feedback \
  --expect-writes false

ros2 run lgh_st3215_tools st3215_preflight \
  --mode commissioning \
  --expect-writes false
```

Capture:

```bash
ros2 run lgh_st3215_tools hardware_snapshot
```

## 4. Validate the IMU boundary

```bash
ros2 run lgh_imu_tools imu_preflight
ros2 run lgh_imu_tools stationary_characterization --duration-sec 20
```

Perform the orientation audit after any sensor, mounting, transport, or driver change.

## 5. Validate the runtime-safe topic surface

Stop the commissioning driver and relaunch:

```bash
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=runtime_safe \
  enable_writes:=false
```

```bash
ros2 run lgh_st3215_tools st3215_preflight \
  --mode runtime \
  --expect-writes false
```

## 6. Run policy shadow

Keep writes disabled:

```bash
ros2 launch littlegreen_biped_pkg policy_shadow.launch.py
```

Confirm `/policy_shadow/desired_position` is active and the policy is not a publisher on `/desired_position`.

## 7. Plan any write-enabled test explicitly

Return to the `commissioning` profile only for a defined guarded operation:

```bash
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=commissioning \
  enable_writes:=true
```

The policy must remain disconnected during servo identification, calibration motion, and standing characterization.

## Current release gates

Passing installation, driver, IMU, and shadow checks does not by itself authorize live policy motion. The next live-policy gate is a paired Track 1 deployment bundle and a hardware-side contract audit. Aggressive outer-PD tuning remains deferred.
