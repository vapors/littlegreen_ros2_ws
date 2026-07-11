# Berkeley ROS 2 Workspace v2.5 Validation

Offline packaging validation completed for the measured hardware-contract propagation.

## Passed

- `servo_map.yaml`, `joint_map.yaml`, `joint_limits.yaml`, and `track1_action_contract_v3.yaml` parse as YAML.
- Canonical joint order matches across all four limit representations.
- Lower and upper limits match exactly across the active native driver map, biped policy map, PD-controller map source, compatibility mirror, and standing-load contract audit mirror.
- Training-default positions lie inside all 12 measured safe ranges.
- `joint_map.yaml` calibration metadata matches the native `servo_map.yaml` for servo sign, center step, and safe step interval.
- Updated Track 1 `hardware_contract.py` passes `py_compile` and preserves the existing v1.2.3 action-contract API while changing only the lower/upper limit arrays and descriptive wording.
- Workspace ZIP integrity test passed.

## Not performed in packaging environment

- ROS 2 Humble `colcon build`.
- Orange Pi UART/hardware test.
- Isaac Lab simulation launch or training smoke test.

These remain host-side integration checks.
