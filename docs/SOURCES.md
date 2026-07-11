# Installation Source References

The v2.6.0 installation scripts were prepared against these upstream project sources on 2026-07-11:

## ROS 2 Humble

- Ubuntu deb installation: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
- Dependency management with rosdep: https://docs.ros.org/en/humble/Tutorials/Intermediate/Rosdep.html
- ROS apt-source package: https://github.com/ros-infrastructure/ros-apt-source

The deployment target remains Ubuntu 22.04 Jammy aarch64. The installer pins `ros2-apt-source` 1.2.0 by default for reproducibility (overridable with `ROS_APT_SOURCE_VERSION`) and retains the official ROS keyring/repository fallback.

## ONNX Runtime

- Installation documentation: https://onnxruntime.ai/docs/install/
- v1.22.0 release: https://github.com/microsoft/onnxruntime/releases/tag/v1.22.0

The workspace pins the Linux aarch64 C/C++ archive to ONNX Runtime 1.22.0 to match the policy package CMake contract.

## Orange Pi

The installer assumes the Orange Pi 5 Max Ubuntu 22.04 BSP, UART pinmux, and `/dev/ttyS3` configuration have already been established. It does not alter boot configuration.
