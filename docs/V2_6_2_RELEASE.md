# LittleGreen ROS 2 Workspace v2.6.2

## Purpose

v2.6.2 is a clean-build packaging hotfix for `littlegreen_biped_pkg`. It is cumulative with the v2.6.1 ST3215 CMake export fix.

No servo behavior, calibration, joint limits, topic names, driver profiles, IMU behavior, policy model, or policy shadow semantics changed.

## Fixed

`littlegreen_biped_pkg` had no public header files, but its `CMakeLists.txt` still contained:

```cmake
install(DIRECTORY include/
  DESTINATION include)
```

An empty directory is not guaranteed to survive archive extraction. During a symlink install, `ament_cmake_symlink_install_directory()` therefore failed when the directory was absent:

```text
ament_cmake_symlink_install_directory() can't find
.../src/littlegreen_biped_pkg/include/
```

v2.6.2 removes both the empty include-directory install rule and the unused build/install include interface from the executable target. The package has no public headers, so there is nothing to export.

## Validator improvement

`validate_source_tree.py` now rejects literal `install(DIRECTORY ...)` rules whose source directory is missing or empty. This catches the same class of archive-dependent failure before release.

## Required rebuild

```bash
cd ~/littlegreen_ros2_ws
rm -rf build install log
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
colcon build --symlink-install --event-handlers console_direct+
```
