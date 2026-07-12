# Validation Record

Validation performed for the 2.7.0 source release.

## Passed

- `scripts/validate_source_tree.py`;
- Python AST parsing for first-party scripts, tools, controller code, and launch files;
- XML parsing for all package manifests;
- YAML parsing for all active YAML files;
- Bash syntax checks for all workspace scripts;
- local-link validation for active root and operator documentation;
- verification that the supplied Track 1 action-contract-v3 policy YAML matches the canonical `joint_map.yaml` defaults, lower bounds, upper bounds, action indices, and joint names within the node's `1e-5 rad` tolerance;
- confirmation that the current packaged legacy `policy_latest.yaml` and `policy.onnx` remain checksum-paired and were not replaced by an unpaired artifact;
- confirmation that servo calibration, servo limits, driver timing, ST3215 profiles, and PD configuration are unchanged;
- confirmation that no generated `__pycache__` or `.pyc` files remain in the release tree.

## Source changes reviewed

- `littlegreen_biped_node.cpp` action-contract-v3 parser and cross-checks;
- `policy_live.launch.py`;
- refreshed shadow, teleop-mux, and deployment launch arguments;
- architecture-aware ONNX Runtime path selection;
- current operator documentation and release metadata.

## Not performed in the packaging environment

- ROS 2 Humble `colcon build`;
- live ONNX Runtime session creation;
- Orange Pi UART operation;
- IMU transport validation;
- policy shadow or live hardware motion.

The Orange Pi package-only build and shadow launch remain the authoritative integration checks for the updated C++ node.
