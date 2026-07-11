# Known Legacy Components and Caveats — v2.6.0

## 1. Historical documents preserve original names

Pre-v2.6 records live under:

```text
docs/history/pre_v2_6/
```

They intentionally retain archive names such as `berkeley_ros2_ws`, `bhl_*`, and `berkeley_biped_pkg` because those names identify real historical artifacts. Use current top-level documentation for active commands.

## 2. `servo_test_pkg` is removed

The old unguarded test package is not part of v2.6.0. Its functionality is superseded by `lgh_st3215_tools` and the guarded driver services.

## 3. The current packaged policy is a snapshot, not the future Track 1 deployment bundle

The policy node and shadow mode are operational, but the current YAML/ONNX snapshot is not the formal v1.4 hardware-deployment package. Use it for software and shadow validation only until Track 1 supplies and audits the paired deployment bundle.

The metadata still contains the historical task identifier:

```text
Velocity-Lilgreen-Humanoid-v0
```

This is intentional. Renaming it locally could break provenance or pairing with the current model.

## 4. Maintenance is offline and read-only

`lgh_st3215_maintenance` directly opens the UART and must not run with the runtime driver. The shared advisory lock is a local process-ownership guard, not an electrical safety mechanism. v2.6.0 contains no EEPROM-write utilities.

## 5. Driver profiles are publication profiles

`commissioning` and `runtime_safe` do not alter servo register reads, command rate, joint mapping, or motion profile. `enable_writes` remains a separate explicit switch. A reduced-read `runtime_fast` profile is intentionally deferred.

## 6. Runtime limits are not sourced from URDF

```text
policy/controller clamp: littlegreen_biped_pkg/src/configs/joint_map.yaml
final hardware clamp:    lgh_st3215_driver/config/servo_map.yaml
```

The Xacro/URDF and compatibility `joint_limits.yaml` are not the final runtime safety authorities.

## 7. Main launch still starts downstream nodes

Use `policy_shadow.launch.py` for the first shadow campaign. It starts only `littlegreen_biped_node`. Passing `policy_output_mode:=shadow` to the broader `littlegreen_biped_launch.py` suppresses the policy’s live target publication but does not remove its joystick or PD-controller nodes.

## 8. No aggressive outer PD in v2.6.0 commissioning

`pd_controller_pkg` remains available for compatibility and controlled future experiments, but v2.6.0 does not introduce or authorize aggressive outer-loop tuning. Initial policy comparison should use shadow mode, then the existing `safety_only` path after the Track 1 deployment bundle is validated.

## 9. Software holds are not E-stops

`hold_current_pose` and `abort_pose_move` latch a software position hold. They are not torque-off circuits, physical power disconnects, or safety-rated emergency stops.

## 10. IMU tools validate the ROS boundary, not sensor internals

Passing `imu_preflight` establishes topic, frame, timestamp, freshness, and numeric consistency at `/imu/data`. It does not prove magnetic calibration, mounting rigidity, absolute heading accuracy, or policy suitability. Repeat the complete orientation audit after moving from micro-ROS to direct I2C or SPI.

## 11. The installer does not configure board-specific hardware

The complete installer handles ROS, rosdep, ONNX Runtime, the source build, and shell environment. It does not change:

```text
Orange Pi boot overlays or UART pinmux
/dev/ttyS3 device-tree configuration
servo power wiring
micro-ROS firmware or agent services
future direct IMU bus configuration
systemd deployment units
```

## 12. Static validation is not hardware acceptance

The archive was validated without an Orange Pi ROS/hardware runtime. Complete `FRESH_INSTALL_CHECKLIST.md` before live motion.
