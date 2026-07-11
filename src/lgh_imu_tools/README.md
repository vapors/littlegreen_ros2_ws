# lgh_imu_tools

Source-independent validation at the canonical `sensor_msgs/msg/Imu` boundary. The same tools apply to the current micro-ROS ICM-20948 source and future direct Orange Pi I2C/SPI drivers.

```bash
ros2 run lgh_imu_tools imu_preflight
ros2 run lgh_imu_tools stationary_characterization --duration-sec 20
ros2 run lgh_imu_tools orientation_audit --pose neutral
ros2 run lgh_imu_tools orientation_audit --pose forward_pitch --expected-axis x --expected-sign positive
ros2 run lgh_imu_tools imu_recorder --duration-sec 10
```

The contract is in `config/imu_contract.yaml`. Orientation auditing records the transformed projected-gravity vector. Non-neutral sign expectations are explicit to avoid silently assuming an unverified mechanical pose convention.
