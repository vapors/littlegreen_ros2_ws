# Validation

Validation performed for the v2.7.2 source release.

## Completed offline checks

- source-tree validator passed;
- active Python files parsed and compiled, including the standalone limit tool;
- active YAML and package XML files parsed;
- shell scripts passed `bash -n`;
- Markdown relative links were checked;
- required package and tool files were checked;
- packaged policy YAML/ONNX checksum and action-contract-v4 checks remained unchanged;
- policy defaults and physical radian bounds still match `joint_map.yaml`;
- driver `training_default_rad` still matches the paired Track 1 policy default;
- every stored servo-map `min_step`/`max_step` was verified as the current derived result of `center_step`, `servo_sign`, `joint_zero_rad`, and `min_rad`/`max_rad`;
- model-zero center inference was unit tested;
- policy-default expected-step conversion was unit tested;
- a two-step center correction was verified to preserve the radian limits without a false range conflict;
- partial one-joint proposals were verified against both servo-map and joint-map patchers;
- the Hardware Limit Tool v1.1.0 rendered a synthetic current capture and reproduced the existing 12-joint safe radian/raw ranges;
- ZIP integrity and executable permissions were checked.

## Hardware/runtime validation still required

The Orange Pi remains authoritative for:

- `colcon build --symlink-install`;
- model-zero capture with real raw feedback;
- one-joint replacement-servo dry-run and application;
- `verify_model_zero` with an external physical alignment reference;
- guarded `assume_policy_default` motion;
- `verify_policy_default` against live ST3215 feedback;
- direct-UART Hardware Limit Tool capture when physical endpoints genuinely need remeasurement.

No claim is made that ROS feedback alone proves physical horn alignment. Model-zero calibration still requires an external physical fixture, angle reference, or alignment marks.
