# LittleGreen ROS 2 Workspace v2.6.4

## Purpose

v2.6.4 is a cumulative Orange Pi clean-install and ROS 2 Humble compatibility hotfix. It includes all v2.6.1–v2.6.3 corrections and does not change servo calibration, joint limits, driver timing, command semantics, IMU behavior, policy behavior, or the ONNX model.

## Humble diagnostic compatibility

`diagnostic_msgs/msg/DiagnosticStatus.level` is declared as ROS `byte`. ROS 2 Humble's Python bindings expose that field as a one-byte `bytes` object such as `b'\x00'`, rather than an integer. The original preflight and hardware snapshot code attempted `int(status.level)`, which raises:

```text
ValueError: invalid literal for int() with base 10: b'\x00'
```

v2.6.4 adds one shared `diagnostic_level_to_int()` compatibility helper and uses it in:

- `st3215_preflight`;
- `hardware_snapshot`;
- `servo_identification`;
- `standing_characterization`.

The helper accepts both Humble's one-byte representation and integer-like representations.

## Installation cleanup

This release also makes permanent the installation fixes discovered during first Orange Pi commissioning:

- removes the unnecessary `gazebo_ros` rosdep from the hardware workspace manifest while retaining the legacy Gazebo launch resources as optional files;
- detects and disables a legacy `ros2.list` when the modern package-managed `ros2.sources` exists, preventing APT `Signed-By` conflicts;
- documents direct sourcing of `~/.config/littlegreen/ros2_env.sh` as optional because the installer already adds it to `~/.bashrc`.

## Package version

`lgh_st3215_tools` advances from `0.2.0` to `0.2.1`.
