# Berkeley ROS 2 workspace source v2 changes

This source snapshot builds on `berkeley_ros2_ws_v1_0` and focuses on control
observability, a real position-servo outer loop, and an explicit training-default
pose baseline.

## `berkeley_biped_pkg`

Added best-effort keep-last-1 policy debug topics at policy rate:

- `/policy_debug/observation` — exact 45-float vector sent to ONNX.
- `/policy_debug/raw_action` — raw 12-float ONNX output.
- `/policy_debug/clipped_raw_action` — action after configured raw-action clip.
- `/policy_debug/target_unclipped` — `q_default + scale * action` before joint limits.
- `/policy_debug/target_clipped` — final joint target published to `/desired_position`.
- `/policy_debug/saturation_mask` — UInt8 bit mask per joint:
  - bit 0 (`1`) raw-action clipping occurred;
  - bit 1 (`2`) target hit the lower physical joint limit;
  - bit 2 (`4`) target hit the upper physical joint limit.

`publish_policy_debug` is enabled in `policy_runtime.yaml` and can be disabled
without changing policy behavior.

## `pd_controller_pkg`

The old safety shaper has been refactored into three explicit layers:

1. command sanitization, timeout handling, and physical joint clipping;
2. low-pass/reference velocity/reference acceleration shaping;
3. optional measured-feedback outer-loop position control.

Supported modes:

- `safety_only` — legacy shaping behavior and default v2 bring-up mode;
- `outer_pd` — velocity-form position controller with measured q/qdot feedback;
- `outer_pid` — adds bounded integral action and anti-windup.

The ST3215 command remains a position target. The real outer loop computes a
bounded velocity command from tracking error and integrates that into the next
position command. Per-joint Kp/Kd/Ki and controller velocity/acceleration limits
are configured in `pd_config.yaml`.

The launch file now correctly loads `pd_config.yaml` as a ROS parameter file.
The service `/pd_controller/reset_to_feedback` aligns the reference shaper and
outer-loop states with current measured feedback.

Existing interfaces are preserved:

- inputs `/desired_position` and `/desired_joint_position`;
- feedback `/joint_states` plus legacy split topics;
- output `/servo_target_radians`;
- debug `/safe_joint_targets` and `/pd_torque_debug`.

New controller debug topics:

- `/outer_controller/velocity_command`;
- `/outer_controller/position_error`;
- `/outer_controller/integral_error`;
- `/outer_controller/status`.

## `bhl_st3215_driver`

Added `training_default_rad` to the native servo map and a guarded smooth move to
that pose. The move is explicit and never starts automatically.

Services:

- `/st3215_driver/move_to_default_pose`;
- `/st3215_driver/release_pose_override`.

The move requires writes to be enabled and full feedback to be ready. It ramps
from measured physical position, blocks external servo targets during the move,
and holds the default pose by default until the override is explicitly released.

## First commissioning sequence

Keep physical writes disabled while inspecting policy debug data. After the
policy observation/action path is understood and the robot is securely supported:

1. enable native-driver writes;
2. call `move_to_default_pose`;
3. inspect joint state, feedback age, and diagnostics;
4. call `pd_controller/reset_to_feedback`;
5. select `outer_pd` only after dry-run tuning looks safe;
6. release the driver pose override explicitly.
