# Measured Hardware Joint Contract

## Purpose

The active ROS 2 control path uses the measured 12-joint LittleGreen hardware envelope. Physical endpoint capture and model-zero center calibration are separate operations.

## Authority model

| Field | Authority | Meaning |
|---|---|---|
| `joint_zero_rad` | robot model | model-zero angle, currently 0 rad |
| `center_step` | model-zero calibration | raw ST3215 position at `joint_zero_rad` |
| `training_default_rad` | paired Track 1 policy | policy-default standing stance |
| `min_rad` / `max_rad` | measured physical-limit contract | durable model-space safety limits |
| `min_step` / `max_step` | generated deployment values | raw endpoints derived from center + radian limits |

A center calibration may change `center_step`, `min_step`, and `max_step` without changing the physical/model-space limits. A physical-limit capture changes `min_rad` / `max_rad` and must be propagated to Track 1.

## Runtime authority by layer

1. `src/lgh_st3215_driver/config/servo_map.yaml`
   - ST3215 IDs, signs, calibrated model-zero centers, model-space safe limits, and derived raw limits
   - final native-driver radian-to-step conversion and clamping
2. `src/littlegreen_biped_pkg/src/configs/joint_map.yaml`
   - canonical action/joint ordering, policy-default pose, physical radian limits, and servo calibration mirror
3. `src/lgh_st3215_tools/config/track1_action_contract_v4.yaml`
   - ROS-side mirror of the paired Track 1 deployment contract
4. `src/littlegreen_biped_pkg/src/configs/joint_limits.yaml`
   - compatibility/documentation mirror; not the final runtime authority

The URDF is not the final servo safety clamp.

## Model zero and policy default

Model zero is the calibration pose:

```text
all 12 actuated joints = joint_zero_rad = 0 rad
```

The current policy-default stance is:

```text
[0.0, 0.0, -0.24, 0.62, -0.22, 0.0,
 0.0, 0.0, -0.24, 0.62, -0.22, 0.0]
```

The policy-default raw target is computed from the model-zero calibration:

```text
step = center_step
     + servo_sign * (policy_default_rad - joint_zero_rad) * 4096 / (2*pi)
```

## Canonical joint order and safe limits

| Index | Joint | Lower rad | Upper rad |
|---:|---|---:|---:|
| 0 | `leg_left_hip_roll_joint` | -0.694893297 | 0.780796221 |
| 1 | `leg_left_hip_yaw_joint` | -0.088970886 | 0.644271931 |
| 2 | `leg_left_hip_pitch_joint` | -1.922077927 | 0.681087470 |
| 3 | `leg_left_knee_pitch_joint` | 0.134990309 | 2.235010008 |
| 4 | `leg_left_ankle_pitch_joint` | -0.809941856 | 0.710233105 |
| 5 | `leg_left_ankle_roll_joint` | -0.513883564 | 0.912718569 |
| 6 | `leg_right_hip_roll_joint` | -0.874369049 | 0.642737950 |
| 7 | `leg_right_hip_yaw_joint` | -0.056757289 | 0.701029220 |
| 8 | `leg_right_hip_pitch_joint` | -1.991107063 | 0.546097160 |
| 9 | `leg_right_knee_pitch_joint` | 0.171805848 | 2.241145931 |
| 10 | `leg_right_ankle_pitch_joint` | -0.845223414 | 0.819145741 |
| 11 | `leg_right_ankle_roll_joint` | -0.443320448 | 1.061514705 |

## LittleGreen Hardware Limit Tool

The standalone tool is installed in the repository at:

```text
tools/lgh_hardware_limit_tool/lgh_hardware_limit_tool.py
```

Capture requires the ROS driver to be stopped because the tool directly owns `/dev/ttyS3` and disables torque.

```bash
python3 tools/lgh_hardware_limit_tool/lgh_hardware_limit_tool.py capture \
  --device /dev/ttyS3 \
  --margin-steps 10 \
  --output-dir ~/lgh_limit_capture
```

After a center-only calibration, reuse the saved physical capture and regenerate the center-dependent raw endpoints:

```bash
python3 tools/lgh_hardware_limit_tool/lgh_hardware_limit_tool.py render \
  --capture ~/lgh_limit_capture/physical_limit_capture.yaml \
  --margin-steps 10 \
  --output-dir ~/lgh_limit_capture/rendered_after_zero_calibration
```

The authoritative contract stores physical and safe limits in radians. Capture-time raw endpoints remain provenance; generated raw endpoints use the current `center_step` values.

## Action-contract v3/v4 deployment check

For a v3 or v4 policy bundle, `littlegreen_biped_node` compares exported action defaults and physical target bounds against `joint_map.yaml` before loading ONNX. It also validates action indices, selected simulation joints, normalized action bounds, previous-action semantics, and the ONNX checksum. Contract v4 additionally validates the non-uniform residual vector and deployment profile.

## Track 1 propagation

A model-zero center calibration does not change the Track 1 policy contract.

A physical-limit change does. Update the Track 1 hardware-limit arrays while preserving the surrounding API and constants, then export a newly paired policy YAML and ONNX artifact.

## Deployment caution

Keep every exported policy paired with the exact joint order, policy-default pose, residual mapping, physical radian limits, timing, IMU transform, and ONNX model used to create it.
