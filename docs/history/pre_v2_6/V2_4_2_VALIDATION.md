# v2.4.2 Validation Record

Validation performed in the packaging environment:

- Python syntax compilation for all 65 workspace Python source files.
- XML parsing for all 15 `package.xml` files.
- YAML parsing for the ST3215 driver configuration files.
- Static assertion that all 12 servo-map entries are `speed=0`, `acceleration=0`.
- Standalone C++17 compilation of `protocol.cpp`, `serial_port.cpp`, and `servo_bus.cpp`
  with warning flags enabled.
- `joint_map.cpp` was not standalone-compiled because the packaging environment does
  not provide the `yaml-cpp` development headers.
- Source inspection and a standalone packet harness verified that
  `SyncWritePositionEx` serializes the configured speed and acceleration fields and
  that zero values remain zero on the wire.
- ZIP integrity check after packaging.

The packaging environment does not include ROS 2 Humble/ament, so a full ROS 2 build
was not performed here. The complete package build and physical validation of the
zero-value speed/acceleration behavior must be performed on the Orange Pi 5 Max target.
