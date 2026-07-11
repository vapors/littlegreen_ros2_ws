# LittleGreen ROS 2 Workspace v2.6.3

## Purpose

v2.6.3 is a shell/install robustness hotfix. It is cumulative with the v2.6.1 ST3215 CMake export fix and the v2.6.2 `littlegreen_biped_pkg` empty-include fix.

No servo behavior, calibration, joint limits, driver profiles, ROS interfaces, IMU behavior, policy behavior, or ONNX model content changed.

## Corrected failure

A fresh install could stop immediately before `rosdep`/`colcon` with:

```text
/opt/ros/humble/setup.bash: line 8: AMENT_TRACE_SETUP_FILES: unbound variable
```

The LittleGreen build and verification scripts intentionally use Bash strict mode (`set -u`). ROS-generated setup files are not guaranteed to be nounset-safe and may inspect variables such as `AMENT_TRACE_SETUP_FILES` before assigning them.

## Correction

v2.6.3 temporarily disables nounset only while sourcing:

- `/opt/ros/humble/setup.bash`;
- `littlegreen_ros2_ws/install/setup.bash`.

The original nounset state is restored immediately afterward. The generated `~/.config/littlegreen/ros2_env.sh` uses the same guarded behavior without changing the option state of an interactive shell.

The installer now reads its displayed release number from the workspace `VERSION` file rather than embedding an older patch number.

## `--skip-onnx`

`--skip-onnx` means “reuse an existing ONNX Runtime installation”; it does not remove the ONNX Runtime build dependency. The default expected location remains:

```text
~/libs/onnxruntime-linux-aarch64-1.22.0
```
