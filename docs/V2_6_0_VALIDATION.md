# LittleGreen ROS 2 Workspace v2.6.0 — Validation Record

Validation date: 2026-07-11

## Result

**Static source validation: PASS**

**Orange Pi ROS/hardware validation: PENDING**

This packaging environment does not contain ROS 2 Humble, `colcon`, the Orange Pi UART, ST3215 bus, IMU transport, or the target ONNX Runtime installation. No claim is made that the archive has already completed an on-device build or hardware run.

## Source identity and structure

Passed:

- workspace root is `littlegreen_ros2_ws`;
- `VERSION` is `2.6.0`;
- required packages exist under `src/`;
- package manifests use the renamed identities;
- renamed Python package/resource directories match `setup.py`;
- renamed C++ include namespaces and CMake targets are internally consistent;
- renamed launch files reference current packages and executables;
- no active `bhl_*`, `berkeley_biped`, or `lilgreen_description` package/path remains;
- retired `servo_test_pkg` and stale duplicate files are absent;
- historical documents are isolated under `docs/history/pre_v2_6/`.

Intentional provenance references retained:

```text
bhl_st3215_microros_pio_v6_5_8/include/servo_map.h
Velocity-Lilgreen-Humanoid-v0
/home/scott/Berkeley-Humanoid-Lite/... Track 1 export paths
```

These identify external historical/current artifacts that have not yet been renamed. They are not active ROS package identities.

## Syntax and format validation

Passed:

- `bash -n` for every installation/build/environment script;
- Python `compileall` for workspace Python code and launch files;
- AST parsing and console-entry-point resolution;
- XML parsing for 17 `package.xml` files;
- YAML parsing for all active workspace YAML files;
- Markdown relative-link validation for root/current documentation;
- removal of generated `__pycache__` and `.pyc` files before archive creation.

## C++ and protocol validation

Passed without ROS dependencies:

- `protocol.cpp` C++17 warning-enabled syntax compilation;
- `serial_port.cpp` C++17 warning-enabled syntax compilation;
- protocol smoke executable for PING, READ, checksum, 12-servo SyncWrite packet size, and signed-value round trip;
- two-process pseudo-UART ownership test: first process opens and locks a PTY; second process refuses with exit code 3.

The YAML-dependent driver core, ROS node, generated message linkage, maintenance executables, and policy node require the target ROS/build environment and remain part of the Orange Pi `colcon` acceptance.

## v2.5.1 behavior-preservation audit

The following were compared against the archived v2.5.1 source after accounting only for required package/node identity keys and the `robot` display identity:

```text
servo_map.yaml
servo_driver.yaml
commissioning profile
runtime_safe profile
pd_config.yaml
joint_map.yaml
joint_limits.yaml
policy_latest.yaml
policy_runtime.yaml
imu_contract.yaml
standing_pose_library.yaml
track1_action_contract_v3.yaml
```

Result: **semantic match**.

The packaged `policy.onnx` is byte-identical to v2.5.1:

```text
SHA-256  a09803aeda3847b46ac0ab8878ee406adaed49e1d8332be5f29496bfb6362088
```

The core driver, tools, IMU tools, maintenance, and policy source comparison showed only intended identity, output-directory, lock-file, and operator-message changes after normalization.

## Pure control-core smoke test

Passed:

- existing `OuterLoopPositionController` produced the expected acceleration-limited first-cycle velocity and position command for the supplied deterministic test vector.

This confirms source continuity only. v2.6.0 does not authorize aggressive outer-PD use.

## Installer review

Passed static checks:

- normal-user enforcement;
- Ubuntu 22.04 and aarch64 enforcement;
- official ROS apt-source package path with pinned default `ros2-apt-source` 1.2.0 and official keyring fallback;
- ROS 2 Humble `ros-base`, development tooling, `rosdep`, and `colcon` install path;
- idempotent `rosdep init`/update logic;
- explicit `rosdep install --from-paths src --ignore-src --rosdistro humble -r -y`;
- pinned ONNX Runtime Linux aarch64 1.22.0 archive path;
- offline ONNX archive override;
- clean `colcon build --symlink-install` helper;
- idempotent LittleGreen shell environment block;
- `dialout` group configuration;
- no broad `apt upgrade` or Orange Pi boot-overlay modification.

Network downloads were not executed in the packaging container. Upstream URLs and filenames were independently verified from current official/upstream release information during preparation.

## Required Orange Pi acceptance

Complete these on the target host:

1. source-tree validation;
2. complete installer or equivalent manual dependency setup;
3. clean `colcon build --symlink-install`;
4. `verify_install.sh --software-only`;
5. `/dev/ttyS3` existence and `dialout` access;
6. maintenance `verify_ids` with the driver stopped;
7. feedback-only commissioning launch;
8. approximately 50 Hz joint state, feedback age, and commissioning telemetry;
9. `st3215_preflight --mode feedback --expect-writes false` exit code 0;
10. current IMU source and `imu_preflight`;
11. `runtime_safe` topic-surface verification;
12. policy-only shadow verification with no policy publisher on `/desired_position`;
13. guarded write-enabled startup hold and supported pose test.

Use [`FRESH_INSTALL_CHECKLIST.md`](FRESH_INSTALL_CHECKLIST.md) as the acceptance record.
