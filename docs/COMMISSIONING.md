# Commissioning Sequence

This is the concise bring-up path. Use [`FRESH_INSTALL_CHECKLIST.md`](FRESH_INSTALL_CHECKLIST.md) when recording full acceptance evidence.

## 1. Verify software and host access

```bash
cd ~/littlegreen_ros2_ws
./scripts/verify_install.sh
```

Confirm:

```bash
ros2 pkg prefix lgh_st3215_driver
ls -l /dev/ttyS3
id -nG | tr ' ' '\n' | grep dialout
```

## 2. Inspect the ST3215 bus offline

The runtime driver must be stopped because maintenance owns `/dev/ttyS3` directly.

```bash
ros2 run lgh_st3215_maintenance bus_scan --first-id 1 --last-id 12
ros2 run lgh_st3215_maintenance verify_ids
```

## 3. Launch feedback-only commissioning

```bash
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=commissioning \
  enable_writes:=false
```

In another terminal:

```bash
ros2 run lgh_st3215_tools st3215_preflight \
  --mode feedback \
  --expect-writes false

ros2 run lgh_st3215_tools st3215_preflight \
  --mode commissioning \
  --expect-writes false

ros2 run lgh_st3215_tools hardware_snapshot
```

Inspect commissioning topics as needed:

```bash
ros2 topic hz /joint_states
ros2 topic echo /st3215_driver/raw_position_steps --once
ros2 topic echo /st3215_driver/diagnostics --once
```

## 4. Start and validate the IMU

Use a dedicated terminal for the current micro-ROS source:

```bash
ros2 run micro_ros_agent micro_ros_agent serial \
  --dev /dev/ttyACM0 \
  -b 115200 \
  -v0
```

If `/dev/ttyACM0` is absent, inspect:

```bash
ls -l /dev/ttyACM*
ls -l /dev/serial/by-id/
```

Validate:

```bash
ros2 topic hz /imu/data
ros2 topic echo /imu/data --once
ros2 run lgh_imu_tools imu_preflight
ros2 run lgh_imu_tools stationary_characterization --duration-sec 10
```

Repeat the orientation audit after any sensor, mount, transport, or firmware change:

```bash
ros2 run lgh_imu_tools orientation_audit --pose neutral
```

## 5. Validate the runtime-safe publication surface

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

The raw position, raw speed, telemetry, and target-debug publishers should be absent; joint state, feedback age, and diagnostics remain available.

## 6. Audit and run policy shadow

```bash
ros2 run littlegreen_biped_pkg policy_bundle_audit
ros2 launch littlegreen_biped_pkg policy_shadow.launch.py
```

The installed audit must inspect ONNX tensor dimensions. The packaged default remains 45-D. Do not relabel it as 47-D. A future phase-guided bundle must report `[1,47] -> [1,12]` and include the exact phase metadata.

Confirm shadow has no live desired-position authority:

```bash
ros2 topic info /desired_position --verbose
ros2 topic info /policy_shadow/desired_position --verbose
ros2 topic echo /policy_status --once
```

Expected:

```text
/desired_position publisher count: 0
/policy_shadow/desired_position publisher count: 1
```

For a 47-D policy, inspect `/policy_debug/gait_phase` and confirm phase zero `[sin,cos]≈[0,1]`, a 36-successful-tick wrap, and phase freeze while the readiness gate is closed.

## 7. Plan write-enabled work explicitly

Before starting a write-enabled driver, stop shadow/policy/controller tools and inspect the command topic:

```bash
ros2 node list
ros2 topic info /servo_target_radians --verbose
```

For calibration or manual guarded pose work, require `Publisher count: 0`.

Start the profile appropriate to the planned task:

```bash
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=commissioning \
  enable_writes:=true
```

Examples of explicit guarded authority:

```bash
# Hold current measured pose and block external commands
ros2 service call \
  /st3215_driver/hold_current_pose \
  std_srvs/srv/Trigger '{}'

# Move to the policy-default stance
ros2 run lgh_st3215_tools assume_policy_default
```

The policy remains disconnected during servo identification, model-zero calibration, and standing characterization.

## 8. Live-policy gate

Passing installation, driver, and IMU checks does not by itself authorize live policy motion.

Required sequence:

```text
paired YAML + ONNX audit
→ runtime-safe feedback-only preflight
→ real IMU preflight
→ policy shadow acceptance
→ stop shadow and inspect command publishers
→ runtime-safe write-enabled preflight
→ live launch with controller_mode=safety_only
→ verify command chain
→ release driver pose override only when intentional
```

Follow [`LIVE_POLICY_DEPLOYMENT.md`](LIVE_POLICY_DEPLOYMENT.md) and [`ROS_GRAPH_AND_AUTHORITY.md`](ROS_GRAPH_AND_AUTHORITY.md).
