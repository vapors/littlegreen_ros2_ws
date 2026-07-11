# Berkeley-Humanoid-Lite ROS 2 Workspace v2.5.1 Reference

## Architecture boundaries

```text
CANONICAL RUNTIME

/imu/data ───────────────┐
                        ▼
/command_velocity → berkeley_biped_pkg
                        │ /desired_position (live only)
                        ▼
                 pd_controller_pkg
                        │ /servo_target_radians
                        ▼
                 bhl_st3215_driver
                        │ exclusive /dev/ttyS3
                        ▼
                    ST3215 bus

ROS LABORATORY

bhl_st3215_tools → driver topics/services → bhl_st3215_driver → ST3215 bus

OFFLINE MAINTENANCE

bhl_st3215_driver STOPPED
bhl_st3215_maintenance → exclusive /dev/ttyS3 → ST3215 bus

IMU VALIDATION

micro-ROS | future I2C | future SPI → /imu/data → bhl_imu_tools

POLICY SHADOW

real observations → exact policy pipeline → /policy_shadow/desired_position
                                      no /desired_position policy publisher
```

## Safety authority hierarchy

1. Physical support and power disconnect.
2. Exclusive UART ownership lock.
3. Driver full-feedback and command watchdog gates.
4. Final driver radian/raw-step clamp.
5. Software pose override/hold.
6. Controller and policy-side clipping.
7. Tool-level graph, freshness, and operator arming guards.

No software service is a safety-rated E-stop.

## Driver profile matrix

| Capability | commissioning | runtime_safe |
|---|:---:|:---:|
| Full 12-servo feedback read | yes | yes |
| 50 Hz bus worker | yes | yes |
| `/joint_states` | yes | yes |
| `/joint_feedback_age_ms` | yes | yes |
| diagnostics | yes | yes |
| raw position/speed topics | yes | no |
| cycle telemetry | yes | no |
| target-step debug | yes | no |
| writes automatically enabled | no | no |

## Tool matrix

| Need | Command/package | Opens UART? | Motion capable? |
|---|---|:---:|:---:|
| Driver health gate | `bhl_st3215_tools st3215_preflight` | no | no |
| Snapshot | `bhl_st3215_tools hardware_snapshot` | no | no |
| Calibration capture/apply/verify | `bhl_st3215_tools` | no | apply edits source YAML only |
| One-joint identification | `bhl_st3215_tools servo_identification` | no | yes, guarded |
| Standing/crouch characterization | `bhl_st3215_tools standing_characterization` | no | yes, guarded |
| Bus scan/register dump/backup | `bhl_st3215_maintenance` | yes | no in v2.5.1 |
| IMU contract validation | `bhl_imu_tools` | no | no |
| Policy inference without authority | `policy_shadow.launch.py` | no | no servo output |

## Preflight philosophy

Preflight is intentionally domain-specific rather than one large orchestration system:

- `st3215_preflight`: servo driver and ROS topic surface;
- `imu_preflight`: canonical IMU message contract;
- future policy deployment audit: added when Track 1 freezes the bundle.

Preflight never launches nodes, enables writes, enables torque, releases overrides, moves joints, or edits configuration.

## Policy modes

| Mode | ONNX inference | Previous action advances | Output |
|---|:---:|:---:|---|
| live | yes | yes | `/desired_position` |
| shadow | yes | yes | `/policy_shadow/desired_position` only |
| disabled | no | no | readiness/status only |

Use the dedicated policy-only shadow launch for the first hardware campaign.

## Deferred work

- Track 1 deployment-bundle pairing and golden-vector audit;
- direct Orange Pi I2C/SPI IMU driver;
- runtime-fast staggered health polling;
- any write-capable maintenance command;
- measured outer-PD experiments;
- live policy standing authorization.
