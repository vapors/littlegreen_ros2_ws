# v2.6.4 validation record

The v2.6.4 source tree was checked with:

- the ROS-independent source validator;
- Python AST/bytecode compilation for active Python packages;
- `bash -n` for active shell scripts;
- XML and YAML parsing;
- direct unit checks for Humble diagnostic levels `b'\x00'`, `b'\x01'`, and `b'\x02'` and integer representations;
- confirmation that all Python consumers use the shared compatibility helper;
- confirmation that `gazebo_ros` is no longer a required rosdep;
- inspection of the ROS apt-source conflict repair logic;
- ZIP integrity and executable-permission checks.

A real Orange Pi `colcon build`, live ROS diagnostic callback, UART test, and hardware commissioning remain the authoritative integration tests.
