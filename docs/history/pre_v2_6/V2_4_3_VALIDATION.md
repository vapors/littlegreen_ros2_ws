# v2.4.3 Validation Record

Validation performed before packaging:

- Python `py_compile` passed for `standing_load_characterization_runner.py`.
- AST parsing passed for all 66 Python source files in the workspace.
- All `package.xml` files parsed as valid XML.
- `protocol.cpp`, `serial_port.cpp`, and `servo_bus.cpp` compiled standalone as C++17 translation units with `-Wall -Wextra -Wpedantic`.
- Broadcast torque-disable and torque-enable packet harness passed with checksum validation.
- Runner helper tests passed for pose-sequence expansion, crouch/return speed selection, and guarded joint-limit validation.
- Synthetic summary tests passed for pose-joint, pose-level, bilateral, and transition aggregation.
- v2.4.2 -> v2.4.3 patch was applied to a clean v2.4.2 source tree and compared against the packaged v2.4.3 tree.
- Final ZIP integrity test passed.

The packaging environment does not contain ROS 2 Humble, so a complete `colcon build` and physical torque/standing test were not performed here. Those remain Orange Pi integration checks.
