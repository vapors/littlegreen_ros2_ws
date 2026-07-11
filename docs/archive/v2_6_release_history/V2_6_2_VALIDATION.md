# v2.6.2 validation record

Static validation completed for the cumulative clean-build fixes:

- `littlegreen_biped_pkg` no longer installs or exports an empty include directory;
- the node retains only its required ONNX Runtime include path;
- the executable retains C++17 compile requirements;
- the v2.6.1 exported ST3215 core target remains present;
- all Python files parse and compile;
- all active YAML and package XML files parse;
- shell scripts pass `bash -n`;
- the source-tree validator passes and now checks missing/empty literal CMake install directories;
- no servo map, driver profile, runtime parameter, ONNX model, policy YAML, or joint map changed.

A real ROS 2 Humble `colcon build` on the Orange Pi remains the authoritative integration check.
