# Berkeley ROS 2 Workspace v2.5 — Measured Hardware Joint Contract

## Purpose

v2.5 propagates the measured 12-joint hardware envelope into the active ROS 2 control path without adding a new runtime adapter. The source measurement used a physical endpoint shim and a fixed **10-step inward margin per endpoint**.

## Runtime authority by layer

1. `bhl_st3215_driver/config/servo_map.yaml`
   - native ST3215 IDs, signs, calibrated centers, safe radian limits, safe raw-step limits
   - final native driver conversion and clamping
2. `berkeley_biped_pkg/src/configs/joint_map.yaml`
   - canonical action/joint ordering and default pose
   - policy-node target clipping
   - `pd_controller_pkg` safety/reference clipping
3. `bhl_st3215_driver/config/track1_action_contract_v3.yaml`
   - ROS-side mirror used only by the standing-load runner contract audit
4. `berkeley_biped_pkg/src/configs/joint_limits.yaml`
   - compatibility/documentation mirror; not loaded by current runtime nodes

No additional runtime adapter or limit service was added.

## Authoritative safe limits

| Joint | Lower rad | Upper rad |
|---|---:|---:|
| `leg_left_hip_roll_joint` | -0.694893297 | 0.780796221 |
| `leg_left_hip_yaw_joint` | -0.088970886 | 0.644271931 |
| `leg_left_hip_pitch_joint` | -1.922077927 | 0.681087470 |
| `leg_left_knee_pitch_joint` | 0.134990309 | 2.235010008 |
| `leg_left_ankle_pitch_joint` | -0.809941856 | 0.710233105 |
| `leg_left_ankle_roll_joint` | -0.513883564 | 0.912718569 |
| `leg_right_hip_roll_joint` | -0.874369049 | 0.642737950 |
| `leg_right_hip_yaw_joint` | -0.056757289 | 0.701029220 |
| `leg_right_hip_pitch_joint` | -1.991107063 | 0.546097160 |
| `leg_right_knee_pitch_joint` | 0.171805848 | 2.241145931 |
| `leg_right_ankle_pitch_joint` | -0.845223414 | 0.819145741 |
| `leg_right_ankle_roll_joint` | -0.443320448 | 1.061514705 |

## Track 1 propagation

For Berkeley-Humanoid-Lite, update only `HARDWARE_LOWER_LIMIT_RAD` and `HARDWARE_UPPER_LIMIT_RAD` in the existing `hardware_contract.py`, preserving the rest of that file's API and constants. Hardware-aligned tasks consume those arrays for action processing and for startup simulation joint-limit writes.

## Deployment caution

Changing the hardware limit contract can change command clipping for a previously trained policy. Keep old policies paired with their original contract for reproducibility; use the v2.5 measured contract for new hardware-aligned training and validation before deploying a new policy.
