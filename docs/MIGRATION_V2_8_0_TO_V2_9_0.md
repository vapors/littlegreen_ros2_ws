# Migration: v2.8.0 to v2.9.0

v2.9.0 is a policy-bundle and observation-builder release. It does not rewrite the ST3215 driver, calibration, hardware maps, IMU interface, maintenance tools, or commissioning workflows.

## 1. Preserve the live calibration first

Before replacing a deployed workspace, compare the live Orange Pi maps with the new source archive:

```bash
sha256sum \
  ~/littlegreen_ros2_ws/src/lgh_st3215_driver/config/servo_map.yaml \
  ~/littlegreen_ros2_ws/src/littlegreen_biped_pkg/src/configs/joint_map.yaml
```

Back up locally verified maps before extracting a new release:

```bash
mkdir -p ~/littlegreen_calibration_backup
cp -a \
  ~/littlegreen_ros2_ws/src/lgh_st3215_driver/config/servo_map.yaml \
  ~/littlegreen_ros2_ws/src/littlegreen_biped_pkg/src/configs/joint_map.yaml \
  ~/littlegreen_calibration_backup/
```

Do not replace a newer robot-specific calibration with archive values without comparison.

## 2. Validate the source tree

```bash
cd ~/littlegreen_ros2_ws
python3 scripts/validate_source_tree.py
```

## 3. Build the changed policy package

```bash
source /opt/ros/humble/setup.bash
cd ~/littlegreen_ros2_ws
colcon build \
  --packages-select littlegreen_biped_pkg \
  --symlink-install
source install/setup.bash
```

A full clean build is appropriate for a fresh installation:

```bash
source /opt/ros/humble/setup.bash
cd ~/littlegreen_ros2_ws
rm -rf build install log
colcon build --symlink-install
source install/setup.bash
```

## 4. Audit the installed five-file bundle

```bash
ros2 run littlegreen_biped_pkg policy_bundle_audit
```

Expected contract summary:

```text
observation_contract: v1 littlegreen_velocity_47d_phase_v1
phase_mode: randomized_static_per_episode
interface: obs[47] -> actions[12]
action_contract: v4
```

The audit must pass without editing `policy.yaml`.

## 5. Run tests

```bash
cd ~/littlegreen_ros2_ws
colcon test --packages-select littlegreen_biped_pkg
colcon test-result --verbose
```

Run the transport-neutral golden observation comparison:

```bash
ros2 run littlegreen_biped_pkg policy_golden_vector_compare \
  --fixture "$(ros2 pkg prefix littlegreen_biped_pkg)/share/littlegreen_biped_pkg/golden/v231_stand_observation_vectors.yaml" \
  --policy-yaml "$(ros2 pkg prefix littlegreen_biped_pkg)/share/littlegreen_biped_pkg/configs/policy_latest.yaml" \
  --skip-onnx
```

A Track 1 fixture containing raw actor outputs and q-targets can use the same command without `--skip-onnx` on a host with Python ONNX Runtime installed.

## 6. Validate in shadow mode

Keep servo writes disabled:

```bash
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=runtime_safe \
  enable_writes:=false
```

Start the current IMU source when required:

```bash
ros2 run micro_ros_agent micro_ros_agent serial \
  --dev /dev/ttyACM0 \
  -b 115200 \
  -v0
```

Launch the policy in shadow mode:

```bash
ros2 launch littlegreen_biped_pkg policy_shadow.launch.py
```

Verify that the final two observation values remain constant throughout the episode:

```bash
ros2 topic echo /policy_debug/gait_phase
ros2 topic echo /policy_debug/observation
```

A transient readiness outage must not resample the Stand phase. An intentional shadow reset begins a new episode:

```bash
ros2 service call \
  /policy/reset_gait_phase \
  std_srvs/srv/Trigger '{}'
```

## Rollback

The prior 45-D v2.8.0 policy pair is retained under:

```text
share/littlegreen_biped_pkg/configs/legacy_v280_45d/
```

Rollback requires selecting the paired legacy YAML and ONNX together. Never pair the v2.3.1 YAML with the legacy model or vice versa.
