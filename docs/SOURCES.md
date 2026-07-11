# Installation Source References

The installation scripts target these upstream components.

## ROS 2 Humble

- Ubuntu deb installation: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
- Dependency management with rosdep: https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html
- ROS apt-source package: https://github.com/ros-infrastructure/ros-apt-source

Deployment target:

```text
Ubuntu 22.04 Jammy
aarch64
ROS 2 Humble
```

The installer pins `ros2-apt-source` 1.2.0 by default for reproducibility. Set `ROS_APT_SOURCE_VERSION` to intentionally override it.

## ONNX Runtime

- Installation documentation: https://onnxruntime.ai/docs/install/
- v1.22.0 release: https://github.com/microsoft/onnxruntime/releases/tag/v1.22.0

The workspace expects the Linux aarch64 C/C++ archive for ONNX Runtime 1.22.0.

## Orange Pi

The installer assumes the Orange Pi 5 Max Ubuntu 22.04 BSP, UART pinmux, and `/dev/ttyS3` configuration already exist. It does not alter boot configuration.
