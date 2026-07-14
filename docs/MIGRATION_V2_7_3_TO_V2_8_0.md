# Migration: v2.7.3 to v2.8.0

v2.8.0 is an observation-contract compatibility release. It preserves the packaged 45-D policy, action contract v4, driver behavior, and calibration files from the source baseline.

## 1. Back up the live Orange Pi calibration

Before replacing the workspace source:

```bash
mkdir -p ~/littlegreen_backup_v2_7_3

cp ~/littlegreen_ros2_ws/src/lgh_st3215_driver/config/servo_map.yaml \
  ~/littlegreen_backup_v2_7_3/servo_map.yaml

cp ~/littlegreen_ros2_ws/src/littlegreen_biped_pkg/src/configs/joint_map.yaml \
  ~/littlegreen_backup_v2_7_3/joint_map.yaml

sha256sum \
  ~/littlegreen_backup_v2_7_3/servo_map.yaml \
  ~/littlegreen_backup_v2_7_3/joint_map.yaml
```

Do not assume a release archive contains calibration changes made locally after v2.7.3 was packaged.

## 2. Replace or extract the source workspace

Stop all LittleGreen ROS processes first. Extract v2.8.0 to the intended workspace path.

Compare maps before building:

```bash
diff -u \
  ~/littlegreen_backup_v2_7_3/servo_map.yaml \
  ~/littlegreen_ros2_ws/src/lgh_st3215_driver/config/servo_map.yaml

diff -u \
  ~/littlegreen_backup_v2_7_3/joint_map.yaml \
  ~/littlegreen_ros2_ws/src/littlegreen_biped_pkg/src/configs/joint_map.yaml
```

When the live maps contain newer verified calibration, copy those values into the v2.8.0 source tree before building and record their provenance.

## 3. Validate and build

```bash
cd ~/littlegreen_ros2_ws
./scripts/validate_source_tree.py
./scripts/build_workspace.sh --clean
source install/setup.bash
./scripts/verify_install.sh --software-only
```

The installed executable list now includes:

```text
littlegreen_biped_node
policy_onnx_contract_probe
policy_bundle_audit
policy_runtime_metrics
annotate_phase_guided_policy
```

## 4. Confirm the packaged 45-D policy remains unchanged

```bash
ros2 run littlegreen_biped_pkg policy_bundle_audit
```

Expected interface:

```text
obs[45] -> actions[12]
```

The audit should warn that the older 45-D bundle lacks explicit observation metadata, but it should pass checksum, tensor shape, action contract, and hardware-map checks.

## 5. Feedback-only regression

Keep the robot supported and writes disabled:

```bash
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=runtime_safe \
  enable_writes:=false
```

Run the normal servo and IMU preflights, then launch the packaged policy in shadow mode. The 45-D observation stream and proposed targets should behave as before.

## 6. Do not install a phase-guided policy yet

v2.8.0 is runtime-ready for a genuine 47-D export, but the release itself contains no deployable v1.4.7 pair. Keep the packaged 45-D pair until the future YAML/ONNX bundle passes the complete audit.

## 7. Rollback

To roll back, stop all ROS processes, restore the v2.7.3 workspace source or archive, restore the backed-up calibration maps when needed, clean-build, and re-run the 45-D policy bundle audit. No servo register or firmware migration is performed by v2.8.0.
