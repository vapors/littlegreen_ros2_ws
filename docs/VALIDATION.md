# Validation

Validation performed for the v2.7.1 source release.

## Completed offline checks

- source-tree validator passed;
- all active Python files parsed and compiled;
- all active YAML and package XML files parsed;
- shell scripts passed `bash -n`;
- Markdown relative links were checked;
- required package files and executable install rules were checked;
- old active package/workspace identifiers were rejected except the documented servo-calibration provenance string;
- the packaged policy YAML reports action contract v4, 45 observations, 12 actions, and 50 Hz timing;
- the policy residual vector is positive and non-uniform;
- v4 nominal residual lower/upper bounds were recomputed from defaults, scales, and physical limits;
- exported defaults and physical bounds match `joint_map.yaml` within `1e-5 rad`;
- driver `training_default_rad` values match the exported athletic default pose;
- `track1_action_contract_v4.yaml` matches the paired policy export;
- Track 1 v1.4.5s3 standing/moving COM height, COM-forward band, and projected-gravity-x targets match the source constants;
- `configs/policy.onnx` and `checkpoints/policy.onnx` are identical;
- packaged ONNX SHA-256 matches the YAML value:

```text
c0079190bc0bf531a5eb6928541560d2402d1f711e14b641e436cad5d43e854d
```

- `policy_bundle_audit` passed against the packaged source pair;
- ZIP integrity and executable permissions were checked.

## Hardware/runtime validation still required

The Orange Pi remains the authoritative integration environment for:

- `colcon build --symlink-install`;
- `ros2 run littlegreen_biped_pkg policy_bundle_audit` from the installed package;
- driver and IMU preflight;
- action-contract-v4 policy shadow startup;
- runtime metrics capture from live ROS topics;
- guarded write-enabled standing with `controller_mode:=safety_only`.

No claim is made that COM height, COM-forward offset, foot-contact, gait-support, slip, or physical torque metrics are available from the current hardware interface.
