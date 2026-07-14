# LittleGreen ROS 2 v2.8.0

v2.8.0 prepares the Orange Pi 5 Max runtime for the Track 1 v1.4.7 phase-guided observation contract while preserving the existing hardware and deployment safety layers.

## Added

- exact support for both `observation[45] -> action[12]` and `observation[47] -> action[12]`;
- independent observation-contract version and metadata validation;
- deterministic 0.72-second gait clock with 36 policy ticks at 50 Hz;
- phase freeze on readiness loss or inference failure;
- phase-zero startup and node-restart semantics;
- `/policy_debug/gait_phase`;
- guarded `/policy/reset_gait_phase` service, refused in live mode;
- strict ONNX float32 tensor-rank and input/output dimension validation;
- installed `policy_onnx_contract_probe` used by `policy_bundle_audit`;
- runtime metrics support for 45-D and 47-D observations;
- `annotate_phase_guided_policy` for genuine 47-D v1.4.7 exports;
- C++ observation/phase tests and Python bundle-audit tests;
- migration, contract, integration-review, and validation documentation.

## Preserved

- the packaged v1.4.5s3 45-D YAML/ONNX pair;
- action contract v4 and previous bounded-action semantics;
- athletic policy default, per-joint residual scales, and physical clipping;
- ST3215 driver profiles, timing, command topics, services, and UART ownership;
- servo calibration and hardware-limit source files;
- model-zero versus policy-default semantics;
- IMU topic and extrinsic boundary;
- `safety_only` as the initial live downstream mode;
- all calibration, maintenance, identification, preflight, and documentation work from earlier Track 2 releases.

## Not included

- a deployable Track 1 v1.4.7 policy;
- a renamed or modified 45-D ONNX;
- action-contract v5;
- contact sensors or measured support-phase inference;
- driver bus optimizations or outer-PD retuning;
- hardware execution results for the future 47-D policy.

## Package version

`littlegreen_biped_pkg` advances from `0.5.0` to `0.6.0`. Other package versions remain unchanged because their runtime behavior is unchanged.
