# Validation

Validation performed for LittleGreen ROS 2 v2.7.3.

## Completed offline checks

- `scripts/validate_source_tree.py` passed and discovered 17 packages.
- Active Markdown links passed for the root README, current `docs/*.md`, and updated first-party package READMEs.
- First-party Python files parsed/compiled successfully.
- Workspace shell scripts passed `bash -n`.
- The command reference contains every first-party ST3215 tool, IMU tool, maintenance executable, policy utility, and launch argument reviewed from the v2.7.2 source tree.
- The exact current micro-ROS agent command is present in the cheat sheet, command reference, commissioning, fresh-install, IMU, workflow, live-deployment, and root README pages.
- Runtime code and configuration hashes are unchanged from v2.7.2, excluding documentation READMEs.
- Policy and hardware contract artifacts are unchanged:

```text
policy_latest.yaml  e9fcde32a3acf6eb6c64d1fd4faba771cd6b47a46128d8322b5cd0dc8c2e90ef
policy.onnx         c0079190bc0bf531a5eb6928541560d2402d1f711e14b641e436cad5d43e854d
servo_map.yaml      bcba21f0dee16572e3c4a35afb9accc20742e3dcb78e28599d71eca515105b3c
joint_map.yaml      593fa25dbe82beb960a4dde5dd6dcdd2f78eca5aab3b8cb343e47389daeaa331
```

- Full release and hotfix archives were tested with `unzip -t` after generation.
- The hotfix was applied to a clean v2.7.2 extraction and the required v2.7.3 documentation files/version were verified.

## Runtime validation still recommended

This is a documentation-only release, so the previously installed v2.7.2 binaries do not need to be rebuilt. On the Orange Pi, confirm the documented process layout with:

```bash
ros2 pkg executables lgh_st3215_tools
ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py --show-args
ros2 topic info /servo_target_radians --verbose
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200 -v0
```

The last command should be run only when the current XIAO IMU controller is connected on `/dev/ttyACM0`; use the actual enumerated device when it differs.
