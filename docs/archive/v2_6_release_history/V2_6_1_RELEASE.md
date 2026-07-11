# LittleGreen ROS 2 Workspace v2.6.1

## Purpose

v2.6.1 is a build-system hotfix for the complete v2.6.0 LittleGreen workspace. It does not change servo behavior, calibration, joint limits, topic names, driver profiles, IMU tools, or policy shadow behavior.

## Fixed

`lgh_st3215_maintenance` consumes the ST3215 packet and serial implementation from `lgh_st3215_driver`. The v2.6.0 driver installed its headers and library but did not export a modern CMake target for downstream packages. On a clean overlay, maintenance compilation could therefore fail with:

```text
fatal error: lgh_st3215_driver/protocol.hpp: No such file or directory
```

v2.6.1 exports `lgh_st3215_driver_core` through the CMake target:

```text
lgh_st3215_driver::lgh_st3215_driver_core
```

and maintenance tools link that target directly. The target propagates the installed driver include directory and core library to every maintenance executable.

## Required rebuild

Because package export metadata changed, remove the old build products before rebuilding:

```bash
cd ~/littlegreen_ros2_ws
rm -rf build install log
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
colcon build --symlink-install --event-handlers console_direct+
```
