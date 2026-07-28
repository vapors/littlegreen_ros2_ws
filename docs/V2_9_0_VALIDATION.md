# LittleGreen ROS 2 v2.9.0 Validation Record

## Completed off-target validation

The release construction environment completed the following checks:

- source-tree naming and generated-file audit;
- YAML and XML parsing;
- Python syntax compilation;
- exact bundle SHA-256 verification;
- dependency-free ONNX protobuf interface inspection;
- offline bundle audit against the unmodified v2.3.1 files;
- Python bundle-audit tests;
- golden observation-vector tests;
- standalone C++17 observation and phase-state harness;
- preservation comparison for `servo_map.yaml` and `joint_map.yaml`;
- release ZIP integrity and checksum generation.

The expected policy interface is:

```text
obs float32 [1,47] -> actions float32 [1,12]
```

## Stand acceptance checks

The phase-state tests cover:

- phase 0.00 -> `[0, 1]`;
- phase 0.25 -> `[1, 0]` within floating-point tolerance;
- phase 0.50 -> `[0, -1]`;
- phase 0.75 -> `[-1, 0]`;
- unit-circle norm;
- unchanged pair over at least 10,000 successful ticks;
- no resampling on readiness loss;
- intentional new-episode behavior;
- exact observation indices and unchanged first 45 values when only phase changes.

## Future Walk contract checks

The phase abstraction tests cover:

- command thresholds;
- standing phase zero;
- alternating first-swing onset;
- continuous `policy_dt / period` advancement;
- modulo wrap;
- no contact or COM dependency;
- new-episode reset rather than stale mid-gait resume;
- live deployment rejection without explicit stage/period pinning.

These tests validate software semantics only. They do not authorize live Walk deployment.

## Required Orange Pi validation

Run on the Orange Pi 5 Max with ROS 2 Humble and ONNX Runtime C++ 1.22.0:

```bash
cd ~/littlegreen_ros2_ws
python3 scripts/validate_source_tree.py
source /opt/ros/humble/setup.bash
colcon build --packages-select littlegreen_biped_pkg --symlink-install
source install/setup.bash
colcon test --packages-select littlegreen_biped_pkg
colcon test-result --verbose
ros2 run littlegreen_biped_pkg policy_bundle_audit
```

Then perform feedback-only shadow validation with the robot supported:

```bash
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  profile:=runtime_safe \
  enable_writes:=false
```

```bash
ros2 launch littlegreen_biped_pkg policy_shadow.launch.py
```

Observe:

```bash
ros2 topic hz /policy_shadow/desired_position
ros2 topic echo /policy_status --once
ros2 topic echo /policy_ready --once
ros2 topic echo /policy_debug/gait_phase
ros2 topic echo /policy_debug/observation
```

Acceptance requires a constant Stand phase pair for the full episode, no `/desired_position` publisher from the policy, correct freshness behavior, and no contract warnings.

## Not claimed by this record

This record does not claim:

- a successful ROS 2/colcon build in the construction environment;
- Orange Pi execution;
- C++ ONNX Runtime inference;
- live servo writes;
- standing balance or fall safety;
- hardware timing, current, temperature, or actuator response;
- a deployable Walk policy.
