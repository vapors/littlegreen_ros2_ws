# Berkeley-Humanoid-Lite ROS 2 Workspace v2.5 — Operator and Developer Reference

Source archive SHA-256: `f2326c6d8da475e6b6b26c3ad1ac7199a30249f28a63b1895a4edbc84311af93`

## Purpose

This workspace is the Orange Pi 5 Max ROS 2 deployment stack for Berkeley-Humanoid-Lite. It joins four major layers:

1. joystick/velocity command input;
2. ONNX policy inference and observation readiness gates;
3. downstream target shaping or optional velocity-form outer PD/PID;
4. native direct-UART ST3215 bus control, feedback, calibration, and actuator characterization.

## Architecture

```text
                    ┌───────────────────────┐
                    │       /imu/data       │
                    └──────────┬────────────┘
                               │
/joy → teleop_twist_joy → /command_velocity
                               │
                               ▼
                    ┌───────────────────────┐
                    │ berkeley_biped_node   │
                    │ ONNX + freshness gate │
                    └──────────┬────────────┘
                               │ /desired_position
                               ▼
                    ┌───────────────────────┐
                    │ pd_controller_node    │
                    │ safety/PD/PID shaping │
                    └──────────┬────────────┘
                               │ /servo_target_radians
                               ▼
                    ┌───────────────────────┐
                    │ bhl_st3215_driver     │
                    │ final clamp + UART    │
                    └──────┬─────────┬──────┘
                           │         │
                           │         ├─ /joint_states
                           │         ├─ /joint_feedback_age_ms
                           │         ├─ telemetry
                           │         └─ diagnostics
                           ▼
                /dev/ttyS3 @ 1 Mbps
                           ▼
                ST3215 servo IDs 1..12
```

## Package summary

| Package | Role | Primary status |
|---|---|---|
| `bhl_st3215_driver` | Native UART/bus owner, conversion, clamp, feedback, telemetry, calibration and test runners | primary hardware authority |
| `berkeley_biped_pkg` | ONNX inference, observation construction, readiness gates, policy target generation | primary policy layer |
| `pd_controller_pkg` | safety shaping and optional outer PD/PID | primary downstream controller |
| `joystick_bridge` | mirrors velocity command to `/tmp/joystick_cmd.txt` | auxiliary |
| `lilgreen_description` | URDF/Xacro, meshes, RViz/Gazebo launch | visualization/simulation description |
| `servo_test_pkg` | old pose/sweep utility | legacy, not recommended for v2.5 native path |
| `teleop_twist_joy` | Joy→Twist command translator | active input path |
| `joy` | SDL joystick driver | active input path |
| other joystick drivers | optional/vendor source | not in primary biped launch |

## Control authority and limit propagation

```text
Track 1 hardware_contract.py
        │ matching canonical hardware limits
        ▼
berkeley_biped_pkg/src/configs/joint_map.yaml
        │ policy + PD clamp
        ▼
bhl_st3215_driver/config/servo_map.yaml
        │ calibrated signs/centers + final rad and raw-step clamp
        ▼
physical ST3215 bus
```

The runtime path does not use `joint_limits.yaml` as an active authority, and the URDF/Xacro limits are not the final hardware safety contract.

## Default v2.5 rates

| Function | Rate |
|---|---:|
| native bus loop | 50 Hz |
| command tick | 50 Hz |
| `/joint_states` | 50 Hz |
| telemetry | 50 Hz |
| diagnostics | 1 Hz |
| PD controller | 50 Hz |
| current packaged ONNX policy | 25 Hz (`policy_dt=0.04`) |

## Primary launch commands

Feedback only:

```bash
ros2 launch bhl_st3215_driver bhl_st3215_driver.launch.py enable_writes:=false
```

Write enabled:

```bash
ros2 launch bhl_st3215_driver bhl_st3215_driver.launch.py enable_writes:=true
```

Policy/control stack:

```bash
ros2 launch berkeley_biped_pkg berkeley_biped_launch.py controller_mode:=safety_only
```

## Joint contract

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

## Recommended commissioning sequence

```text
feedback-only native driver
  ↓
verify all 12 joints + ages + diagnostics
  ↓
write-enabled native driver
  ↓
guarded default-pose move or explicit current-pose hold
  ↓
start policy stack in safety_only
  ↓
inspect /policy_ready and debug targets
  ↓
reset PD state to current feedback
  ↓
release native driver pose override
  ↓
small supported motions
  ↓
outer_pd / outer_pid tuning only after safety-only path is understood
```

## Driver safety state model

The native driver has two concepts that should not be confused:

- **torque state** — whether servos are commanded torque-enabled or torque-disabled;
- **pose override state** — whether external `/servo_target_radians` messages are accepted or ignored.

Typical manual capture state:

```text
torque_enabled_state = 0
pose_override_active = true
```

Typical hold-after-enable state:

```text
torque_enabled_state = 1
pose_override_active = true
```

Normal external control requires:

```text
torque_enabled_state = 1
pose_override_active = false
```

## Readiness gates

The policy node can require:

- valid fresh IMU sample;
- complete position feedback;
- joint velocities when `require_joint_velocity=true`;
- recent `/joint_states` transport;
- recent `/joint_feedback_age_ms` topic;
- every joint's physical feedback age under the configured maximum.

A stale command stream can be configured to zero the command velocity instead of stopping policy inference.

## Standing-load research path

Pose capture and evaluation are intentionally separate:

- capture mode defines full-body quasi-static waypoint poses;
- evaluation mode executes deterministic smoothstep trajectories and records loaded tracking, current, load proxy, voltage, temperature, and transition summaries.

For the current Track 2→Track 1 handoff, the preferred ladder is:

```text
normal → shallow → medium → deep → medium → shallow → normal
```

and speed-sweep analysis should use actual logged reference peak velocity, especially when `--min-transition-sec` limits the requested motion.

## Where to look next

- Full package descriptions: `PACKAGE_REFERENCE.md`
- All switches and parameters: `INTERFACES_AND_PARAMETERS.md`
- Copy/paste commands: `COMMAND_CHEATSHEET.md`
- End-to-end procedures: `WORKFLOWS.md`
- Known legacy pieces and warnings: `KNOWN_LEGACY_AND_CAVEATS.md`
