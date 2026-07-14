# Recommended Workflows

## 1. Fresh installation

```text
validate source
→ install software
→ log out/in
→ verify overlay
→ run the staged commissioning checklist
```

Use:

- [`INSTALL_ORANGE_PI.md`](INSTALL_ORANGE_PI.md)
- [`FRESH_INSTALL_CHECKLIST.md`](FRESH_INSTALL_CHECKLIST.md)

## 2. Feedback-only hardware observation

```bash
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=commissioning \
  enable_writes:=false
```

Then:

```bash
ros2 run lgh_st3215_tools st3215_preflight \
  --mode feedback \
  --expect-writes false
```

Use this before any write-enabled test.

## 3. Hardware snapshot

With the commissioning driver running:

```bash
ros2 run lgh_st3215_tools hardware_snapshot
```

Preserve the generated report with the related calibration, identification, or standing dataset.

## 4. Offline bus maintenance

```text
stop runtime driver
→ acquire UART through maintenance package
→ scan/verify/read/backup
→ stop maintenance
→ restart runtime driver
```

Commands:

```bash
ros2 run lgh_st3215_maintenance bus_scan --first-id 1 --last-id 12
ros2 run lgh_st3215_maintenance verify_ids
ros2 run lgh_st3215_maintenance register_dump --id 1 --address 0x00 --length 0x47
ros2 run lgh_st3215_maintenance backup_control_tables
```

Do not run maintenance and the driver at the same time.

## 5. Model-zero calibration and servo replacement

```text
commissioning profile, writes disabled
→ physically place selected joint(s) at model zero
→ capture center_step
→ preserve min_rad/max_rad
→ derive raw min_step/max_step from the new center
→ dry-run/apply source maps
→ rebuild driver and biped package
→ verify model zero
→ guarded move to policy default
→ verify policy-default raw targets
```

Use:

- [`CALIBRATION_WORKFLOW.md`](CALIBRATION_WORKFLOW.md)
- [`SERVO_REPLACEMENT_CHECKLIST.md`](SERVO_REPLACEMENT_CHECKLIST.md)

A like-for-like replacement servo normally does not require a new physical endpoint capture.

## 6. Servo identification

```text
securely support robot
→ disconnect policy
→ commissioning profile
→ preflight
→ enable writes explicitly
→ run one joint at a time
→ preserve dataset and manifest
```

Show the current options:

```bash
ros2 run lgh_st3215_tools servo_identification --help
```

## 7. Standing characterization

Use the guarded standing tool only after suspended identification and calibration are confirmed. Keep the policy disconnected and record the mechanical support condition.

```bash
ros2 run lgh_st3215_tools standing_characterization --help
```

## 8. IMU validation

```text
source publishes /imu/data
→ imu_preflight
→ stationary characterization
→ known-orientation audit
→ repeat after any source or mounting change
```

Commands:

```bash
ros2 run lgh_imu_tools imu_preflight
ros2 run lgh_imu_tools stationary_characterization --duration-sec 20
ros2 run lgh_imu_tools orientation_audit --pose neutral
```

## 9. Policy shadow

```text
runtime_safe driver
+ writes disabled
+ real joint feedback
+ real IMU
→ policy shadow
→ log proposed targets
→ no live desired-position authority
```

Launch:

```bash
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=runtime_safe \
  enable_writes:=false
```

```bash
ros2 launch littlegreen_biped_pkg policy_shadow.launch.py
```

Verify the policy does not publish `/desired_position`.

## 10. Guarded live policy

```text
paired action-contract-v3/v4 YAML + ONNX
→ policy_bundle_audit PASS
→ package-only rebuild
→ runtime-safe driver and IMU preflight with writes disabled
→ policy shadow acceptance
→ runtime-safe driver preflight with writes enabled
→ policy_live.launch.py with safety_only
→ guarded zero-command standing
→ short supervised run windows
```

Launch the explicit live stack with:

```bash
ros2 launch littlegreen_biped_pkg policy_live.launch.py \
  controller_mode:=safety_only
```

The complete deployment and stop sequence is documented in [`LIVE_POLICY_DEPLOYMENT.md`](LIVE_POLICY_DEPLOYMENT.md).

Do not begin with `outer_pd` or `outer_pid` tuning.
