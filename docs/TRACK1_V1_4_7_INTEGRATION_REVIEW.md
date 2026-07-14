# Track 1 v1.4.7 Integration Review

## Purpose

This review defines the Track 2 Volume 3 boundary for integrating the phase-guided Track 1 policy into the Orange Pi 5 Max ROS 2 stack without changing the established servo, calibration, safety, or action contracts.

## Authoritative source identities at handoff

```text
Track 1 archive:
Berkeley-Humanoid-Lite_v1_4_7_Phase_Guided_Gait.zip
SHA-256:
a29cd3ba7a1df09836c509151d2be2893618187913164857b58effef010ffff4

Recorded Track 2 v2.7.3 archive SHA-256:
6e3f0d5273dbfd62ecbd03e785beae78a4d4c7b7e125d5d52e49dbc176332635

Actual uploaded v2.7.3 modification baseline SHA-256:
6fb3348436f1d64c5d48029ff45afd0a89d5aa731e4a625cde87788d89bc1e2b
```

The recorded handoff hash and the uploaded archive hash were not byte-identical. The uploaded tree identifies itself as v2.7.3 and passed its source validation, so the actual uploaded source tree was used as the authoritative modification baseline. Historical paths or values did not override that tree.

## Confirmed Track 1 change

Task:

```text
Velocity-Lilgreen-Hardware-ST3215-Loaded-v7
```

Unchanged action side:

```text
action_contract_version: 4
num_actions: 12
bounded_default_centered_vector_residual
per-joint action_residual_scale_rad
bounded normalized previous actions
athletic q_default and physical clipping
```

Changed observation side:

```text
45 values -> 47 values
append [sin(2*pi*phase), cos(2*pi*phase)]
period 0.72 s
policy_dt 0.02 s
36 policy ticks per period
```

The phase clock is a policy input, not an offline reward-only metric.

## Packaged-artifact caveat

The Track 1 source archive demonstrates the v1.4.7 task and exporter capability, but its packaged `configs/policy_latest.yaml` still identifies the older v1.4.5s3 45-D standing policy. That policy and its ONNX remain valid as a 45-D pair, but they are not a v1.4.7 deployment artifact.

Track 2 must not:

- edit `num_observations` from 45 to 47 without a new ONNX;
- relabel the old task as v1.4.7;
- invent a policy checksum;
- create a synthetic ONNX and present it as deployable;
- introduce action-contract v5.

## v2.8.0 integration boundary

v2.8.0 implements and tests runtime compatibility while retaining the old packaged pair. It adds:

- independent observation-contract validation;
- 45-D legacy and 47-D phase-guided construction;
- deterministic gait-phase lifecycle;
- 47-D ONNX input acceptance and strict tensor inspection;
- gait-phase debug output and guarded reset;
- metrics support for both layouts;
- a metadata annotation helper that refuses non-v1.4.7 exports;
- source and unit tests for ordering, wrapping, metadata, and shape mismatch.

It does not alter:

- ST3215 UART ownership or bus timing;
- servo IDs, signs, center steps, raw limits, or physical radian limits;
- model-zero or policy-default calibration semantics;
- action-contract v4 target transformation;
- downstream `safety_only` controller behavior;
- IMU transport or canonical `/imu/data` interface.

## Future acceptance gate

A real v1.4.7 deployment requires all of:

```text
policy YAML num_observations: 47
ONNX input tensor [1,47]
ONNX output tensor [1,12]
matching policy_sha256
explicit phase metadata from OBSERVATION_CONTRACT.md
action_contract_version: 4
exact current joint defaults and physical bounds
```

The sequence remains:

```text
offline audit
feedback-only driver + IMU preflight
supported-robot shadow
phase/debug capture and review
write-enabled hold and authority inspection
guarded live launch with controller_mode=safety_only
```
