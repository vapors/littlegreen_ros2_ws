# Install on Ubuntu 22.04 x86_64

Use this path for an Ubuntu 22.04 x86_64 development host. The resulting workspace can run policy shadow, inspect deployment bundles, and build the full ROS 2 source tree. Hardware UART use remains optional.

## Install

```bash
cd ~/littlegreen_ros2_ws
./scripts/validate_source_tree.py
./scripts/install_ubuntu_x86_64.sh
```

The installer provides:

- ROS 2 Humble `ros-base`;
- `rosdep`, `colcon`, compiler, YAML, and Eigen dependencies;
- ONNX Runtime 1.22.0 Linux x64 under `~/libs/onnxruntime-linux-x64-1.22.0`;
- a clean workspace build;
- the LittleGreen environment block in `~/.bashrc`.

Gazebo is not installed.

## Options

```text
--skip-ros
--skip-onnx
--skip-build
--no-bashrc
```

Use an existing ONNX Runtime installation with:

```bash
ONNXRUNTIME_DIR=/path/to/onnxruntime-linux-x64-1.22.0 \
  ./scripts/install_ubuntu_x86_64.sh --skip-onnx
```

## Verify

Open a new terminal or reload the shell:

```bash
source ~/.bashrc
```

Then:

```bash
~/littlegreen_ros2_ws/scripts/verify_install.sh --software-only
```

The architecture-specific default used by CMake and the environment helper is:

```text
~/libs/onnxruntime-linux-x64-1.22.0
```

The Orange Pi runtime continues to use:

```text
~/libs/onnxruntime-linux-aarch64-1.22.0
```
