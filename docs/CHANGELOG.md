# Changelog

## 2.7.3

- Added a comprehensive command and option reference for first-party executables, launch files, services, parameters, and exit codes.
- Expanded the command cheat sheet into a task-oriented operating reference.
- Added the current micro-ROS agent command (`/dev/ttyACM0`, 115200 baud, `-v0`) to commissioning, IMU, shadow, and live workflows.
- Added ROS graph and command-authority documentation, including stale-publisher checks and pose-override release behavior.
- Refreshed IMU validation, commissioning, fresh-install, calibration, servo-replacement, live-deployment, workflow, and troubleshooting documentation.
- No runtime code, policy artifact, calibration map, action contract, or hardware configuration changes.

## 2.7.2

Calibration and serviceability release:

- separated **model zero** from the Track 1 **policy-default stance** throughout active tools and documentation;
- made `capture_calibration --reference model-zero` the recommended/default center-step workflow;
- added single-joint calibration with repeatable `--joint` arguments for replacement servos;
- preserved durable model-space `min_rad`/`max_rad` limits while deriving raw `min_step`/`max_step` from the new center;
- removed the misleading old-endpoint `RANGE_CONFLICT` behavior for small center changes;
- added `verify_model_zero`, `verify_policy_default`, `print_model_zero`, `print_policy_default`, and `assume_policy_default` commands;
- retained `verify_calibration`, `print_default_pose`, and `pose_console` as compatibility aliases with clarified semantics;
- synchronized servo-center/raw-limit mirrors in `joint_map.yaml` during approved calibration application;
- added a like-for-like servo replacement checklist;
- updated the standalone LittleGreen Hardware Limit Tool to v1.1.0 with current workspace paths and center-independent model-space contracts;
- added offline tests covering model-zero capture, raw-limit derivation, partial proposals, and map synchronization.

No Track 1 policy YAML, ONNX model, policy-default vector, physical radian limits, servo IDs, signs, or runtime bus timing changed.

## 2.7.1

- added direct action-contract-v4 support to `littlegreen_biped_node`;
- loaded non-uniform `action_residual_scale_rad` vectors without converting them to a scalar;
- validated v4 nominal residual bounds, deployment profile, and required transform flag;
- packaged the paired Track 1 v1.4.5s3 YAML/ONNX export;
- propagated the athletic default pose to `joint_map.yaml` and the driver training-default mirror;
- added `track1_action_contract_v4.yaml`;
- added `policy_bundle_audit`;
- added read-only `policy_runtime_metrics`;
- documented the Track 1/Track 2 metric boundary and updated live deployment instructions;
- retained action-contract-v3 compatibility and all existing driver/IMU safety boundaries.

## 2.7.0

Policy deployment and action-contract release:

- updated `littlegreen_biped_node` to recognize `action_contract_version: 3` directly;
- loads `action_residual_scale_rad` instead of requiring a legacy `action_scale` compatibility alias;
- validates exported action defaults, physical bounds, joint names, selected simulation indices, and selected simulation defaults against `joint_map.yaml`;
- requires normalized v3 action bounds `[-1, 1]`, positive residual scales, the bounded residual transform, and `previous_action_observation: bounded_normalized_action`;
- retains read compatibility with legacy policy YAML files that do not declare an action-contract version;
- added the explicit `policy_live.launch.py` launch for the policy node plus `pd_controller_node` in `safety_only` by default;
- refreshed `policy_shadow.launch.py` and `biped_teleop_mux.launch.py` so they expose the current policy, joint-map, runtime, and controller arguments;
- added `LIVE_POLICY_DEPLOYMENT.md` with the paired bundle, rebuild, shadow, write-enabled preflight, live launch, and stop sequence;
- updated the canonical joint-map metadata so policy timing and residual scale come from the paired policy YAML rather than a stale embedded policy snapshot;
- integrated Ubuntu 22.04 x86_64 installation scripts and corrected the x86_64 ONNX Runtime default to the CPU `x64` package;
- made build, verification, and environment scripts choose the ONNX Runtime default by host architecture.

The packaged legacy YAML/ONNX snapshot remains paired and unchanged. A new Track 1 policy must be copied as both `policy_latest.yaml` and `policy.onnx` before use.

## 2.6.5

Documentation-only refresh:

- replaced version-specific operator pages with current, task-oriented documentation;
- added a documentation index;
- documented both ST3215 driver profiles and their topic surfaces;
- added the missing `profile` and `policy_output_mode` launch arguments to the interface reference;
- corrected the hardware-contract path for `track1_action_contract_v3.yaml`;
- clarified that shell environment sourcing is automatic in new terminals and manual sourcing is optional;
- simplified installation, commissioning, calibration, workflow, and command pages;
- moved prior release and migration records under `docs/archive/`;
- updated package inventory versions and removed generated Python cache files.

No servo calibration, limits, timing, ROS interfaces, policy model, or runtime behavior changed.
