# v2.5.1 validation record

## Completed in the packaging environment

- Python source compilation for all workspace Python files.
- XML parsing for every `package.xml`.
- YAML parsing for workspace YAML configuration and contract files.
- Console-entry-point to Python-module/main-function audit for the new Python packages.
- ROS-independent C++ syntax checks for the ST3215 protocol, serial-port ownership lock, and all four maintenance executables (dependency stubs used only where ROS/yaml-cpp headers were unavailable).
- Protocol packet/reply assertion smoke test.
- Cross-process UART ownership-lock smoke test using a Linux pseudo-terminal.
- Static source checks for moved tool imports, installed data files, driver profile parameters, policy shadow publisher separation, and legacy-package exclusion.
- Archive content and SHA-256 verification after packaging.

## Requires Orange Pi / ROS 2 Humble validation

The packaging environment does not contain ROS 2 Humble, `colcon`, the Orange Pi UART, ST3215 bus, ICM-20948 stream, or ONNX Runtime deployment library. Therefore the following are intentionally not claimed here:

- full `colcon build`;
- launch-file execution;
- live ROS graph/QoS validation;
- ownership-lock validation on the physical Orange Pi UART;
- servo-bus scans or control-table backups;
- 50 Hz runtime/commissioning profile timing;
- IMU preflight against the current sensor;
- ONNX policy shadow inference.

## Recommended Orange Pi acceptance order

```bash
cd ~/berkeley_ros2_ws
colcon build --packages-select \
  bhl_st3215_driver \
  bhl_st3215_tools \
  bhl_st3215_maintenance \
  bhl_imu_tools \
  berkeley_biped_pkg \
  --symlink-install
source install/setup.bash
```

Then follow `docs/V2_5_1_ACCEPTANCE_CHECKLIST.md`.
