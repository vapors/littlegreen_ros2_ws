# LittleGreen Biped Policy Integration Safety Update v1.1

This update hardens the ROS 2 policy inference boundary without changing the 45-observation / 12-action policy contract.

## Policy artifact pairing

`policy_latest.yaml` and `policy.onnx` now live together in `src/configs/` for deployment. The policy node resolves the relative ONNX path beside the selected YAML first and fails closed if the paired model is missing. It no longer silently falls back to `src/checkpoints/policy.onnx`.

The updated training-side `export_policy.py` writes the normal export artifacts and also copies an atomic deployment pair to:

```text
~/littlegreen_ros2_ws/src/littlegreen_biped_pkg/src/configs/
  policy_latest.yaml
  policy.onnx
```

The deployment copy verifies the ONNX SHA-256 after copying, and the ROS policy node verifies the same checksum before creating the ONNX Runtime session.

## IMU extrinsic

The default physical transform is configurable in `src/configs/policy_runtime.yaml`:

```text
x_base =  y_imu
y_base = -x_imu
z_base =  z_imu
```

The transform is applied to both angular velocity and projected gravity. The IMU quaternion is validated and normalized before use.

## Readiness and freshness gates

Policy inference is blocked until:

- a valid IMU sample has been received, unless `override_imu=true`;
- all 12 joint positions have been received;
- all 12 joint velocities have been received when `require_joint_velocity=true`;
- IMU receive age is within `imu_timeout_sec`;
- every required joint field has a recent ROS receive update within `joint_state_timeout_sec`;
- the complete observation vector is finite.

Default thresholds:

```text
imu_timeout_sec         0.050 s
joint_state_timeout_sec 0.150 s
command_timeout_sec     0.500 s
```

A stale command stream zeros the command observation rather than stopping balance inference.

## ONNX finite protection

The node now rejects:

- non-finite IMU values;
- invalid quaternion norms;
- non-finite JointState fields;
- non-finite command velocities;
- non-finite observation vectors;
- null or wrong-sized ONNX outputs;
- non-finite policy actions;
- non-finite post-processed joint targets.

When an inference cycle is rejected, `/desired_position` is not published and `prev_actions` is not advanced.

## Status topics

```text
/policy_ready   std_msgs/Bool
/policy_status  std_msgs/String
```

Both use reliable, transient-local QoS so late subscribers can see the current state.

## Runtime configuration

`launch/littlegreen_biped_launch.py` now accepts:

```text
policy_config
policy_runtime_config
onnx_model_path
joint_map
use_sim
override_imu
```

The normal launch path loads `policy_runtime.yaml` and the paired `policy_latest.yaml` + `policy.onnx` bundle.

## Build note

The C++ source and launch Python were structurally/static checked in the update environment. A full ROS 2/ONNX Runtime compile was not possible there because the environment did not contain the project's ROS 2 Humble and ONNX Runtime toolchain. Build and run the package on the Orange Pi before enabling servo torque.


## v1.2 hardware feedback freshness

See `FRESHNESS_GATE_v1_2.md`. The host now distinguishes ROS message transport freshness from the age of each servo's last successful physical feedback read using `/joint_feedback_age_ms`.
