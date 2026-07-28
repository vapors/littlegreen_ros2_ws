# Validation

Current release-specific validation is recorded in [`V2_9_0_VALIDATION.md`](V2_9_0_VALIDATION.md).

## Source and offline checks

```bash
cd ~/littlegreen_ros2_ws
python3 scripts/validate_source_tree.py
python3 src/littlegreen_biped_pkg/scripts/policy_bundle_audit.py \
  --policy-yaml src/littlegreen_biped_pkg/src/configs/policy_latest.yaml \
  --joint-map src/littlegreen_biped_pkg/src/configs/joint_map.yaml \
  --onnx src/littlegreen_biped_pkg/src/configs/policy.onnx \
  --deployment-contract src/littlegreen_biped_pkg/src/configs/deployment_contract.yaml \
  --policy-sha256-file src/littlegreen_biped_pkg/src/configs/policy.sha256 \
  --bundle-manifest src/littlegreen_biped_pkg/src/configs/bundle_manifest.yaml
```

The source validator checks workspace/package versions, naming, syntax, the complete v2.3.1 bundle, preserved action/hardware contracts, and generated-file cleanliness.

## ROS 2 host checks

```bash
source /opt/ros/humble/setup.bash
cd ~/littlegreen_ros2_ws
colcon build --packages-select littlegreen_biped_pkg --symlink-install
source install/setup.bash
colcon test --packages-select littlegreen_biped_pkg
colcon test-result --verbose
ros2 run littlegreen_biped_pkg policy_bundle_audit
```

## Shadow acceptance

```bash
ros2 launch littlegreen_biped_pkg policy_shadow.launch.py
ros2 topic echo /policy_debug/gait_phase
ros2 topic echo /policy_debug/observation
```

The Stand phase/sine/cosine must remain constant for the full deployment episode and across transient readiness loss. Shadow must not publish `/desired_position`.

No Orange Pi, ROS 2 runtime, C++ ONNX inference, or hardware result is claimed until those commands are run on the deployment host.
