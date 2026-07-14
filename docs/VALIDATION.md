# Validation

Validation status for LittleGreen ROS 2 v2.8.0. Detailed release-specific results and remaining hardware acceptance steps are in [`V2_8_0_VALIDATION.md`](V2_8_0_VALIDATION.md).

## Completed offline checks

- `scripts/validate_source_tree.py` validates workspace version, package inventory, Python/YAML syntax, required v2.8.0 files, observation/action contract tokens, and the retained packaged-policy identity.
- Twelve Python contract tests cover valid 45-D/47-D bundles, unsupported counts, missing phase metadata, ONNX input/output/type mismatches, v4 enforcement, and guarded v1.4.7 metadata annotation.
- First-party Python files compile successfully; nine existing ROS-independent calibration/diagnostic/outer-loop regression tests also pass.
- Workspace shell scripts pass `bash -n`.
- The v2.8.0 source does not replace servo calibration, physical limits, or the packaged v1.4.5s3 policy.

Retained deployment and hardware artifacts:

```text
policy.onnx         c0079190bc0bf531a5eb6928541560d2402d1f711e14b641e436cad5d43e854d
servo_map.yaml      bcba21f0dee16572e3c4a35afb9accc20742e3dcb78e28599d71eca515105b3c
joint_map.yaml      593fa25dbe82beb960a4dde5dd6dcdd2f78eca5aab3b8cb343e47389daeaa331
```

`policy_latest.yaml` is intentionally still the 45-D `Velocity-Lilgreen-Stand-ST3215-Loaded-v5s3` bundle.

## Build-host validation

On ROS 2 Humble with ONNX Runtime 1.22.0:

```bash
cd ~/littlegreen_ros2_ws
./scripts/validate_source_tree.py
./scripts/build_workspace.sh --clean
source install/setup.bash
colcon test --packages-select littlegreen_biped_pkg
colcon test-result --verbose
./scripts/verify_install.sh --software-only
ros2 run littlegreen_biped_pkg policy_bundle_audit
```

The installed audit uses `policy_onnx_contract_probe` and must report actual ONNX shapes `[1,45] -> [1,12]` for the packaged policy.

## Runtime validation still required

The robot must remain supported for initial regression testing. Compare live calibration maps before replacing them, launch the driver with writes disabled, run servo and IMU preflight, and verify the packaged 45-D policy in shadow mode.

A phase-guided hardware result is not claimed. Validation of `[1,47] -> [1,12]` remains pending until a genuine paired Track 1 v1.4.7 export is supplied.
