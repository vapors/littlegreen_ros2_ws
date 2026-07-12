# Changelog

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
