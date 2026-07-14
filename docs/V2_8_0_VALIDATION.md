# LittleGreen ROS 2 v2.8.0 Validation

## Validation completed in the release-construction environment

- Source archive extracted and treated as the modification baseline.
- Packaged v1.4.5s3 policy YAML, ONNX, `joint_map.yaml`, and `servo_map.yaml` retained.
- First-party Python source passed `compileall`; shell scripts passed `bash -n`.
- Twelve v2.8.0 Python contract tests passed, covering:
  - legacy 45-D compatibility;
  - valid explicit 47-D metadata;
  - rejection of a 47-D YAML paired with a 45-D ONNX shape;
  - rejection of missing gait-phase metadata;
  - rejection of unsupported observation counts;
  - rejection of phase metadata on a 45-D bundle;
  - action-contract-v4 enforcement for 47-D;
  - rejection of incorrect ONNX output dimensions and non-float32 tensors;
  - guarded annotation of a genuine 47-D export;
  - refusal to annotate a 45-D export or unexpected task.
- Nine existing ROS-independent calibration/diagnostic/outer-loop regression tests passed with their source packages on `PYTHONPATH`.
- A standalone C++17 harness compiled and executed the phase/observation header successfully.
- Header-level phase/observation tests cover:
  - phase-zero `[0,1]` encoding;
  - half-cycle convention;
  - 36-tick wrapping;
  - rejection of non-integral timing;
  - explicit reset to phase zero;
  - exact 45-D ordering;
  - phase append at indices 45 and 46.
- `scripts/validate_source_tree.py` passed and checks workspace version, required files, packaged-policy identity, unchanged action/hardware contracts, and v2.8.0 runtime tokens.
- Active Markdown local links across 81 current Markdown files, 37 YAML files, and 17 XML files parsed successfully.

## Not completed in the release-construction environment

The construction environment did not contain ROS 2 Humble or ONNX Runtime C/C++ 1.22.0. Therefore the complete package build, `ament_cmake_gtest` execution, installed ONNX probe, and ROS graph/runtime checks are not claimed here.

## Validation requiring a ROS 2 Humble build host

Run on Ubuntu 22.04 x86_64 or the Orange Pi after extraction:

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

The installed audit must report the packaged ONNX input `[1,45]` and output `[1,12]`.

## Hardware regression still required

With the robot securely supported:

1. Compare live and release `servo_map.yaml` and `joint_map.yaml`.
2. Preserve the newest verified calibration.
3. Launch `runtime_safe` with writes disabled.
4. Run ST3215 and IMU preflight.
5. Run the packaged 45-D policy in shadow mode.
6. Confirm no `/desired_position` publisher exists in shadow mode.
7. Capture `/policy_debug/observation` and confirm 45 values.
8. Run `policy_runtime_metrics` and review target clipping and tracking metrics.

## Future 47-D acceptance

Hardware validation of phase-guided inference is intentionally pending because no deployable v1.4.7 pair was supplied. When available, acceptance requires:

- exact YAML/ONNX checksum pairing;
- ONNX `[1,47] -> [1,12]` tensor contract;
- explicit observation metadata;
- supported-robot shadow capture showing a 36-tick phase cycle;
- review of repeated phase values during any readiness outage;
- no unexpected command authority;
- guarded live testing with `controller_mode:=safety_only`.

No statement in this release should be interpreted as a successful hardware deployment of Track 1 v1.4.7.
