# LittleGreen ROS 2 v2.9.1 Hotfix

## Scope

v2.9.1 is a narrow Orange Pi compatibility and lint-cleanup hotfix for v2.9.0. It does not change the v2.3.1 Stand observation contract, randomized-static phase semantics, action contract v4, joint map, servo calibration, driver behavior, IMU transform, or runtime safety gates.

## ONNX Runtime fix

The v2.9.0 node and `policy_onnx_contract_probe` stored the non-owning result of `GetTensorTypeAndShapeInfo()` after calling it on a temporary `Ort::TypeInfo`. Once that temporary was destroyed, later `GetShape()` calls used an invalid metadata view. On the Orange Pi this appeared as:

```text
cannot create std::vector larger than max_size()
```

v2.9.1 keeps the owning input and output `Ort::TypeInfo` objects alive until all element-type and shape queries are complete.

## Orange Pi rebuild

```bash
cd ~/littlegreen_ros2_ws
rm -rf build/littlegreen_biped_pkg install/littlegreen_biped_pkg log
source /opt/ros/humble/setup.bash
export ONNXRUNTIME_DIR="$HOME/libs/onnxruntime-linux-aarch64-1.22.0"
export LD_LIBRARY_PATH="$ONNXRUNTIME_DIR/lib:${LD_LIBRARY_PATH:-}"
colcon build --packages-select littlegreen_biped_pkg --symlink-install
source install/setup.bash
colcon test --packages-select littlegreen_biped_pkg
colcon test-result --verbose
ros2 run littlegreen_biped_pkg policy_bundle_audit
```

Continue with feedback-only driver and policy shadow only after the audit passes.
