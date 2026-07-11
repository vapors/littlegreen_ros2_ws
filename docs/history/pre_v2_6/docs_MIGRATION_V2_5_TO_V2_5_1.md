# Migration from v2.5 to v2.5.1

## Command mapping

| v2.5 | v2.5.1 |
|---|---|
| `bhl_st3215_driver print_default_pose_reference.py` | `bhl_st3215_tools print_default_pose` |
| `bhl_st3215_driver capture_default_pose_calibration.py` | `bhl_st3215_tools capture_calibration` |
| `bhl_st3215_driver apply_servo_calibration.py` | `bhl_st3215_tools apply_calibration` |
| `bhl_st3215_driver verify_default_pose_calibration.py` | `bhl_st3215_tools verify_calibration` |
| `bhl_st3215_driver default_pose_move_console.py` | `bhl_st3215_tools pose_console` |
| `bhl_st3215_driver servo_identification_runner.py` | `bhl_st3215_tools servo_identification` |
| `bhl_st3215_driver standing_load_characterization_runner.py` | `bhl_st3215_tools standing_characterization` |

The `apply_calibration` command now requires the source map to be explicit:

```bash
ros2 run bhl_st3215_tools apply_calibration \
  <proposal.yaml> \
  --source-servo-map \
  ~/berkeley_ros2_ws/src/bhl_st3215_driver/config/servo_map.yaml
```

This avoids silently editing an installed package-share copy or a hard-coded source path.

## Launch mapping

The old launch remains valid and defaults to the commissioning publication surface:

```bash
ros2 launch bhl_st3215_driver bhl_st3215_driver.launch.py \
  enable_writes:=false
```

The explicit v2.5.1 form is preferred:

```bash
ros2 launch bhl_st3215_driver bhl_st3215_driver.launch.py \
  profile:=commissioning enable_writes:=false
```

For deployment observation and shadow work:

```bash
ros2 launch bhl_st3215_driver bhl_st3215_driver.launch.py \
  profile:=runtime_safe enable_writes:=false
```

## Configuration ownership

| Configuration | Owner |
|---|---|
| `servo_driver.yaml`, `servo_map.yaml`, profiles | `bhl_st3215_driver` |
| standing pose library and temporary Track 1 audit contract | `bhl_st3215_tools` |
| canonical IMU validation contract | `bhl_imu_tools` |
| policy runtime/deployment metadata | `berkeley_biped_pkg` / future Track 1 bundle |

## UART ownership

Normal commissioning tools no longer open `/dev/ttyS3`; they communicate through the driver. Offline maintenance opens the UART only after acquiring the same advisory lock used by the driver. Stop the driver before running maintenance.
