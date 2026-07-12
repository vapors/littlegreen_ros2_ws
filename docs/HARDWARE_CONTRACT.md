# Measured Hardware Joint Contract

## Purpose

The active ROS 2 control path uses the measured 12-joint LittleGreen hardware envelope. The source endpoint measurement used a physical endpoint shim and a fixed 10-step inward margin at each endpoint.

## Runtime authority by layer

1. `src/lgh_st3215_driver/config/servo_map.yaml`
   - ST3215 IDs, signs, calibrated centers, safe radian limits, and raw-step limits
   - final native-driver radian-to-step conversion and clamping
2. `src/littlegreen_biped_pkg/src/configs/joint_map.yaml`
   - canonical action/joint ordering, default pose, and policy/controller clipping
3. `src/lgh_st3215_tools/config/track1_action_contract_v3.yaml`
   - ROS-side mirror used by standing-characterization contract auditing
4. `src/littlegreen_biped_pkg/src/configs/joint_limits.yaml`
   - compatibility/documentation mirror; not the final runtime authority

The URDF is not the final servo safety clamp.

## Action-contract-v3 deployment check

For a v3 policy bundle, `littlegreen_biped_node` compares the exported action defaults and physical target bounds against the `joints[]` section of `joint_map.yaml` before loading the ONNX session. It also checks action indices and selected simulation joint names. Any mismatch is fatal.

Policy timing and `action_residual_scale_rad` are supplied by the paired policy YAML; they are intentionally not duplicated as runtime authority in `joint_map.yaml`.

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

## Default pose

Canonical default joint vector:

```text
[0.0, 0.0, -0.1, 0.4, -0.3, 0.0,
 0.0, 0.0, -0.1, 0.4, -0.3, 0.0]
```

## Track 1 propagation

For LittleGreen training, update the hardware-limit arrays in the Track 1 hardware contract while preserving the surrounding API and constants. Hardware-aligned tasks should use the same lower/upper arrays for action processing and simulation startup limit writes.

## Deployment caution

Changing the hardware contract changes clipping behavior. Keep every exported policy paired with the exact joint order, default pose, action mapping, limits, timing, IMU transform, and ONNX model used to create it.
