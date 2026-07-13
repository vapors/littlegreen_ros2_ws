# Track 1 v1.4.5s3 Integration Review

This review records the Track 1 changes that affect the Track 2 ROS deployment boundary.

## Deployment-critical changes

### Action contract v4

The v1.4.5s3 exporter changed from the uniform v3 residual contract to a non-uniform per-joint vector contract:

```yaml
action_contract_version: 4
action_contract_name: bounded_default_centered_vector_residual_athletic
action_transform: bounded_default_centered_vector_residual
deployment_contract_profile: v1_4_5_stabilized_vector_residual
```

Per-joint residual scale:

```text
[0.24, 0.16, 0.42, 0.58, 0.48, 0.26,
 0.24, 0.16, 0.42, 0.58, 0.48, 0.26]
```

The existing ROS target calculation was already vectorized, but the v2.7.0 parser accepted only contract v3. v2.7.1 updates the parser and startup validation instead of adding a legacy `action_scale` alias.

### Athletic default pose

The 12-action default changed to:

```text
[0.0, 0.0, -0.24, 0.62, -0.22, 0.0,
 0.0, 0.0, -0.24, 0.62, -0.22, 0.0]
```

This is propagated to `joint_map.yaml` and the driver training-default mirror. Calibrated servo centers and physical endpoints remain unchanged.

### Nominal residual bounds

The export now carries explicit nominal lower/upper targets after applying each residual scale around the athletic default and intersecting with the physical joint envelope. v2.7.1 recomputes and validates those values at startup.

### Paired artifact

```text
Task: Velocity-Lilgreen-Stand-ST3215-Loaded-v5s3
Policy period: 0.020 s
ONNX SHA-256: c0079190bc0bf531a5eb6928541560d2402d1f711e14b641e436cad5d43e854d
```

The YAML and ONNX model are installed as a pair.

## Training and analysis changes

The latest Track 1 task adds or emphasizes:

- raw-action range and excess outside `[-1, 1]`;
- bounded-action saturation;
- physical target-limit use;
- joint velocity and simulated torque utilization;
- posture RMS/max deviation;
- standing COM height and height error;
- standing COM-forward-over-feet target band;
- projected-gravity-x lean target;
- support-state fractions;
- foot air time, swing clearance, lift, and forward swing velocity;
- actuator delay and loaded-envelope activity.

## Propagated to Track 2

v2.7.1 adds a read-only runtime recorder for the metrics observable from existing ROS topics:

- raw action mean/std/min/max;
- raw action excess;
- bounded saturation and raw clip mask;
- physical target clipping;
- target residual magnitude;
- real joint tracking error;
- joint velocity-limit use;
- projected gravity;
- body angular-velocity norm;
- posture RMS/max;
- observable standing subconditions: upright, quiet yaw, near default;
- exact command entering the policy observation.

The result metadata also records the Track 1 actuator-model name, stage, delay scale, and velocity-scale ranges from the policy export.

## Not propagated as direct measurements

The current hardware interface cannot faithfully reproduce:

- base COM height;
- COM-forward offset relative to the feet;
- root XY drift;
- foot support/contact fractions;
- swing clearance and foot lift counts;
- foot slip;
- physical joint torque.

These remain clearly marked unavailable rather than being approximated from unvalidated proxies.

## No-change findings

- Observation size remains 45.
- Action size remains 12.
- Policy period remains 0.020 s.
- Command velocity remains observation elements `0:3`.
- Previous action remains the bounded normalized action.
- Physical joint bounds remain the measured Track 2 hardware envelope.
- The first live path remains `controller_mode:=safety_only`.
