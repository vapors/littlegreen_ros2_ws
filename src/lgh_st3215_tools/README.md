# lgh_st3215_tools

Guarded laboratory and commissioning tools for LittleGreen. This package never opens `/dev/ttyS3`; the normal `lgh_st3215_driver` remains the sole UART owner.

## Commands

```bash
ros2 run lgh_st3215_tools st3215_preflight --mode feedback
ros2 run lgh_st3215_tools hardware_snapshot
ros2 run lgh_st3215_tools print_default_pose
ros2 run lgh_st3215_tools capture_calibration
ros2 run lgh_st3215_tools apply_calibration proposal.yaml --source-servo-map ~/littlegreen_ros2_ws/src/lgh_st3215_driver/config/servo_map.yaml
ros2 run lgh_st3215_tools verify_calibration
ros2 run lgh_st3215_tools pose_console
ros2 run lgh_st3215_tools servo_identification --help
ros2 run lgh_st3215_tools standing_characterization --help
```

## Exit codes

| Code | Meaning |
|---:|---|
| 0 | PASS |
| 2 | test ran but acceptance criteria failed |
| 3 | refused safety/precondition |
| 4 | timeout or ROS resource unavailable |
| 5 | configuration error |
| 6 | hardware/I/O error |
| 7 | operator abort |
| 70 | internal software error |
| 130 | SIGINT |

Preflight and snapshot commands always emit a YAML report plus a text summary under `~/.ros/lgh_reports/` unless an output root is supplied.
