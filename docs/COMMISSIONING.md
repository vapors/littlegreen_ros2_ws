# LittleGreen v2.6.0 Commissioning Sequence

This is the abbreviated sequence. The authoritative fresh-install gate checklist is [`FRESH_INSTALL_CHECKLIST.md`](FRESH_INSTALL_CHECKLIST.md).

## 1. Verify the install and active overlay

```bash
source ~/.config/littlegreen/ros2_env.sh
cd ~/littlegreen_ros2_ws
./scripts/verify_install.sh --software-only
```
*The installer automatically adds the LittleGreen environment script to ~/.bashrc. New interactive Bash terminals will load it automatically. Manual sourcing is only needed in the current terminal before reopening it, or in scripts and services that do not load ~/.bashrc.

Old package names should not be visible in `ros2 pkg list`.

## 2. Read-only bus verification

With the normal driver stopped and the robot securely supported:

```bash
ros2 run lgh_st3215_maintenance verify_ids
```

The v2.6.0 maintenance package is read-only.

## 3. Feedback-only commissioning profile

```bash
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=commissioning \
  enable_writes:=false
```

Inspect:

```bash
ros2 topic hz /joint_states
ros2 topic hz /joint_feedback_age_ms
ros2 topic hz /st3215_driver/telemetry
ros2 topic echo /st3215_driver/diagnostics --once
```

Run the domain preflight:

```bash
ros2 run lgh_st3215_tools st3215_preflight \
  --mode feedback \
  --expect-writes false
```

Do not continue after a failed or refused preflight.

## 4. IMU contract

Bring up the current micro-ROS IMU source or the future direct driver, then:

```bash
ros2 run lgh_imu_tools imu_preflight
ros2 run lgh_imu_tools stationary_characterization --duration-sec 20
```

Repeat orientation/extrinsic testing whenever the sensor mounting or transport changes.

## 5. Runtime-safe publication profile

Stop the commissioning driver and launch:

```bash
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=runtime_safe \
  enable_writes:=false
```

This keeps joint state, feedback age, and diagnostics while suppressing high-rate laboratory publications. It does not change the underlying feedback read.

## 6. Policy shadow

Until Track 1 supplies the audited deployment bundle, use the current policy only in shadow mode:

```bash
ros2 launch littlegreen_biped_pkg policy_shadow.launch.py
```

Verify that the policy publishes `/policy_shadow/desired_position` and does not publish `/desired_position`.

## 7. Guarded write-enabled startup hold

Only after the preceding gates pass, with the robot securely supported:

```bash
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=commissioning \
  enable_writes:=true \
  default_pose_move_duration_sec:=8.0
```

The driver must begin from complete fresh measured feedback and hold the measured current pose.

## 8. Guarded pose motion

```bash
ros2 run lgh_st3215_tools pose_console
```

The software abort holds the latest measured pose; the physical power disconnect remains the emergency action.

## 9. Deferred live-policy and outer-loop work

Do not release live policy commands solely because the software and shadow checks pass. The next gate is Track 1 deployment-bundle pairing and golden-vector contract validation. Aggressive outer-PD tuning is outside v2.6.0.
