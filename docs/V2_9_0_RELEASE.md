# LittleGreen ROS 2 v2.9.0 Release

## Purpose

v2.9.0 integrates the unmodified LittleGreen Humanoid Lite v2.3.1 canonical Stand deployment bundle and reproduces its shared 47-D actor observation contract. The release is intentionally narrow: it changes policy bundle parsing, phase state, observation construction, auditing, tests, metrics, and deployment documentation.

## Packaged policy

```text
Task:              Velocity-Lilgreen-Stand-ST3215-Loaded-v23
Role:              stand
Observation:       [1,47]
Action:            [1,12]
Phase mode:        randomized_static_per_episode
Action contract:   v4
Policy rate:       50 Hz
ONNX SHA-256:      66936666934ff02e75681b8ae5c2c6727021df598419016ce0479e8466282608
```

The complete bundle is installed as:

```text
policy.onnx
policy.yaml
policy_latest.yaml
policy.sha256
deployment_contract.yaml
bundle_manifest.yaml
```

`policy.yaml` and `policy_latest.yaml` are byte-identical copies of the exported policy metadata.

## Runtime changes

- Accept export schema 2 and observation contract `littlegreen_velocity_47d_phase_v1` without annotation or rewriting.
- Verify the compact layout string and all explicit layout ranges.
- Build the exact shared 47-D observation.
- Sample Stand phase uniformly once per intentional policy episode and hold it static.
- Preserve the phase pair across readiness loss and waiting for the first complete sensor snapshot.
- Provide deterministic fixed-phase/seed injection for tests and replay; refuse it in live mode.
- Report phase, episode, successful tick, mode, movement state, and onset side on `/policy_debug/gait_phase`.
- Make `/policy/reset_gait_phase` an intentional new-episode operation in shadow/disabled modes and refuse it in live mode.
- Preserve action contract v4 and bounded previous-action observation exactly.
- Include a future Walk phase abstraction, while blocking live Walk when deployment stage/period is not explicitly pinned.

## Offline tooling

`policy_bundle_audit` now verifies:

- all five exported bundle files;
- policy YAML and manifest hashes;
- ONNX SHA-256;
- actual ONNX input `[1,47]` and output `[1,12]` tensor contracts;
- observation layout and phase mode;
- canonical joint order;
- action contract v4 defaults, scales, physical bounds, and previous-action semantics.

`policy_golden_vector_compare` provides fixture-driven hooks for:

- Track 1 vs Track 2 observation vectors;
- Track 1 vs ONNX raw actions when Python ONNX Runtime is available;
- bounded action parity;
- q-target parity.

## Preserved systems

The ST3215 runtime driver, calibration files, joint map, physical limits, IMU transform, maintenance tools, commissioning tools, safety services, launch authority boundaries, and PD safety behavior are unchanged unless explicitly listed in the source patch.

## Walk status

Walk construction logic is present for contract testing and shadow inspection. Live Walk is not released. A Walk bundle must explicitly record the checkpoint's intended deployment stage or exact pinned period because the training curriculum uses stage-specific phase periods.

## Validation boundary

The release archive was statically validated in a non-ROS build environment. Orange Pi, ROS 2 Humble, ONNX Runtime C++ execution, shadow runtime, and hardware validation remain required on the deployment host. See `V2_9_0_VALIDATION.md`.
