# Safety and Current Limitations

## Runtime driver and maintenance are mutually exclusive

`lgh_st3215_driver` is the sole normal UART owner. `lgh_st3215_maintenance` opens the UART directly and must be used only after the driver is stopped.

The shared advisory lock prevents two local processes from opening the same device. It is not an electrical safety mechanism.

## Maintenance is read-only

Current maintenance commands support bus scanning, ID verification, register reads, and control-table backups. They do not change IDs, baud rates, EEPROM values, motion profiles, or factory settings.

## Driver profiles do not control writes

`commissioning` and `runtime_safe` select publications only. They do not change servo reads, command rate, joint mapping, speed, acceleration, or torque state.

`enable_writes` remains a separate explicit launch argument.

## Software holds are not E-stops

`hold_current_pose`, `abort_pose_move`, and the pose override are software position controls. They do not replace:

- a physical servo-power disconnect;
- mechanical support;
- a safety-rated emergency stop;
- torque-off hardware.

## Runtime limits are not sourced from the URDF

Final authorities:

```text
policy/controller clamp:
  src/littlegreen_biped_pkg/src/configs/joint_map.yaml

hardware conversion and final clamp:
  src/lgh_st3215_driver/config/servo_map.yaml
```

## Policy shadow and live deployment are different gates

The dedicated shadow launch starts only the policy node and publishes proposed targets on:

```text
/policy_shadow/desired_position
```

It does not create a policy publisher on `/desired_position`.

Live policy motion requires a paired Track 1 YAML/ONNX bundle, successful action-contract-v3/v4 validation against `joint_map.yaml`, checksum verification, driver and IMU preflight, and an accepted shadow run. Use [`LIVE_POLICY_DEPLOYMENT.md`](LIVE_POLICY_DEPLOYMENT.md).

## Use the dedicated shadow launch

Passing `policy_output_mode:=shadow` to the broader `littlegreen_biped_launch.py` suppresses live policy output, but that launch still starts joystick, teleop, the file bridge, and `pd_controller_pkg`.

For initial hardware observation, use:

```bash
ros2 launch littlegreen_biped_pkg policy_shadow.launch.py
```

## Aggressive outer PD is deferred

`pd_controller_pkg` supports `safety_only`, `outer_pd`, and `outer_pid`. Initial live deployment must use shadow mode followed by the `safety_only` path after the policy bundle is validated.

Do not interpret the current outer-loop gains as identified physical stiffness or damping.

## IMU tools validate the ROS boundary

Passing `imu_preflight` establishes message rate, frame, timestamps, freshness, quaternion norm, and finite numeric values at `/imu/data`.

It does not by itself prove:

- mounting rigidity;
- magnetic calibration;
- absolute heading accuracy;
- policy suitability;
- correct sign conventions for every physical pose.

Repeat the orientation audit after moving from micro-ROS to direct I2C or SPI.

## Direct Orange Pi IMU transport is not included yet

The workspace contains transport-independent IMU tools. A current micro-ROS source or a future direct I2C/SPI driver must publish the canonical `/imu/data` topic.

## The installer does not configure board-specific hardware

The installer does not change:

```text
Orange Pi boot overlays or UART pinmux
/dev/ttyS3 device-tree configuration
servo power wiring
micro-ROS firmware or agent services
future direct IMU bus configuration
systemd deployment units
external shell scripts
```

## Static checks are not hardware acceptance

Source validation and a successful build do not authorize motion. Complete the staged hardware checklist before enabling writes.
