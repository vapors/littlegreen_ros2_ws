# Recommended Workflows — v2.6.0

## 1. Runtime authority hierarchy

Keep these responsibilities separate:

```text
Track 1 training
  hardware_contract.py
      ↓ matching canonical radians
ROS policy/control
  littlegreen_biped_pkg/src/configs/joint_map.yaml
      ↓
Native hardware
  lgh_st3215_driver/config/servo_map.yaml
      ↓
ST3215 raw steps
```

In v2.6.0:

- `joint_map.yaml` is authoritative for policy-node and PD-controller target clipping;
- `servo_map.yaml` is authoritative for physical sign, center calibration, radian conversion, final radian clamp, and raw-step clamp;
- `joint_limits.yaml` is a compatibility/documentation mirror only;
- the URDF/Xacro is not the active hardware safety authority.

## 2. Daily bring-up

1. Mechanically support the robot.
2. Source ROS and the workspace.
3. Start the native driver feedback-only with the `commissioning` profile.
4. Verify all 12 joint reads, feedback ages, telemetry, diagnostics, and ST3215 preflight.
5. Verify the current `/imu/data` source with `lgh_imu_tools`.
6. Stop the commissioning driver and use `runtime_safe`, still with writes disabled, for policy shadow work.
7. Run the dedicated policy shadow launch and inspect observations, actions, and targets.
8. Relaunch write-enabled only for an explicitly planned guarded hardware test.
9. Keep policy live output disconnected until the Track 1 deployment bundle and hardware contract are audited.

## 3. Calibration workflow

```text
mechanically align q_default
        ↓
feedback-only native driver
        ↓
lgh_st3215_tools capture_calibration
        ↓
review proposal and status flags
        ↓
lgh_st3215_tools apply_calibration dry run
        ↓
apply center_step changes
        ↓
rebuild/relaunch driver
        ↓
lgh_st3215_tools verify_calibration
```

Calibration changes `center_step` in software. It does not rewrite hidden servo EEPROM center offsets.

## 4. Policy bring-up workflow

Current v2.6.0 boundary:

```text
runtime_safe driver + writes disabled
        ↓
canonical real joint state and IMU
        ↓
dedicated policy_shadow.launch.py
        ↓
/policy_ready and shadow target stream
        ↓
inspect observation/raw action/clipped target/saturation mask
        ↓
Track 1 paired deployment bundle arrives
        ↓
golden-vector and hardware contract audit
        ↓
only then plan guarded live safety_only tests
```

Aggressive `outer_pd` or `outer_pid` tuning is deferred. It must be treated as a separate measured plant change, not an automatic part of first policy deployment.

## 5. Actuator identification workflow

For the Track 1 nominal actuator model:

- keep policy off;
- keep downstream outer-loop feedback off;
- use `command_path=direct`;
- use the fixed max-envelope profile (`speed=0`, `acceleration=0`);
- support the robot so the tested joint condition matches the intended experiment;
- treat timing, actuator onset, velocity response, lag proxy, residual error, and hysteresis as distinct effects.

Use standing-load experiments as a **loaded-response extension and validation layer**, not as a replacement for suspended single-joint identification.

## 6. Standing-pose capture workflow

Capture mode intentionally uses manual torque-off positioning:

```text
preflight
  ↓
explicit TORQUE OFF confirmation phrase
  ↓
torque disable + pose override
  ↓
operator positions full robot pose
  ↓
press Enter
  ↓
2 s median capture (default)
  ↓
q-std stability check
  ↓
Track 1 / servo-map contract check
  ↓
replace only that named pose in the pose library
```

Default library location:

```text
~/.ros/lgh_standing_poses.yaml
```

Capture audits default to:

```text
~/.ros/lgh_standing_pose_capture_audits/
```

## 7. Standing-load evaluation workflow

Recommended ladder:

```text
normal
→ shallow
→ medium
→ deep
→ medium
→ shallow
→ normal
```

Use `--no-return-between-poses` with the explicit down/up ladder to avoid direct normal→deep jumps.

For controlled speed characterization:

- use the same requested speed in crouch and stand-return directions;
- use measured/logged `q_ref` peak velocity as the analysis independent variable;
- keep `--min-transition-sec 0.25` for the aggressive loaded sweep unless intentionally designing a step-like test;
- preserve the exact pose-library hash in metadata.

## 8. Data locations

| Data | Default location |
|---|---|
| standing pose library | `~/.ros/lgh_standing_poses.yaml` |
| pose capture audits | `~/.ros/lgh_standing_pose_capture_audits/` |
| standing-load reports | `~/littlegreen_ros2_ws/track2_standing_reports/<timestamp>_standing_load/` |
| actuator identification reports | relative `identification_reports/<timestamp>.../` unless overridden |
| calibration reports | relative `calibration_reports/` unless overridden |
| joystick command mirror | `/tmp/joystick_cmd.txt` |

## 9. Current v2.6.0 joint contract

| Idx | Joint | ID | Sign | Center | q_default | Lower | Upper | Safe steps |
|---|---|---|---|---|---|---|---|---|
| 0 | leg_left_hip_roll_joint | 1 | -1 | 2041 | 0.000 | -0.695 | 0.781 | 1532..2494 |
| 1 | leg_left_hip_yaw_joint | 2 | -1 | 2027 | 0.000 | -0.089 | 0.644 | 1607..2085 |
| 2 | leg_left_hip_pitch_joint | 3 | -1 | 2110 | -0.100 | -1.922 | 0.681 | 1666..3363 |
| 3 | leg_left_knee_pitch_joint | 4 | -1 | 2051 | 0.400 | 0.135 | 2.235 | 594..1963 |
| 4 | leg_left_ankle_pitch_joint | 5 | 1 | 2024 | -0.300 | -0.810 | 0.710 | 1496..2487 |
| 5 | leg_left_ankle_roll_joint | 6 | 1 | 2021 | 0.000 | -0.514 | 0.913 | 1686..2616 |
| 6 | leg_right_hip_roll_joint | 7 | 1 | 2038 | 0.000 | -0.874 | 0.643 | 1468..2457 |
| 7 | leg_right_hip_yaw_joint | 8 | 1 | 2040 | 0.000 | -0.057 | 0.701 | 2003..2497 |
| 8 | leg_right_hip_pitch_joint | 9 | 1 | 2123 | -0.100 | -1.991 | 0.546 | 825..2479 |
| 9 | leg_right_knee_pitch_joint | 10 | 1 | 2051 | 0.400 | 0.172 | 2.241 | 2163..3512 |
| 10 | leg_right_ankle_pitch_joint | 11 | -1 | 2077 | -0.300 | -0.845 | 0.819 | 1543..2628 |
| 11 | leg_right_ankle_roll_joint | 12 | -1 | 2058 | 0.000 | -0.443 | 1.062 | 1366..2347 |
