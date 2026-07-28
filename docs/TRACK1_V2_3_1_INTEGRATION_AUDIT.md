# Track 1 v2.3.1 Bundle Integration Audit

This audit compares the unmodified LittleGreen Humanoid Lite v2.3.1 Stand export with the LittleGreen ROS 2 v2.8.0 policy runtime. The exported bundle is authoritative for policy semantics; the v2.8.0 workspace is authoritative for the existing hardware, safety, calibration, and deployment architecture.

## Accepted bundle identity

```text
Task:                 Velocity-Lilgreen-Stand-ST3215-Loaded-v23
Task role:            stand
Policy interface:     obs[47] -> actions[12]
Policy rate:          50 Hz (policy_dt 0.02 s)
Observation contract: v1 littlegreen_velocity_47d_phase_v1
Phase mode:           randomized_static_per_episode
Action contract:      v4 bounded default-centered vector residual
ONNX SHA-256:          66936666934ff02e75681b8ae5c2c6727021df598419016ce0479e8466282608
```

The ONNX tensor interface was inspected as:

```text
input:  obs     float32 [1,47]
output: actions float32 [1,12]
```

## v2.8.0 mismatches

The v2.8.0 47-D compatibility path expected a different export schema:

| Area | v2.8.0 expectation | v2.3.1 export |
|---|---|---|
| Contract name | `littlegreen_hardware_phase_guided_47_v1` | `littlegreen_velocity_47d_phase_v1` |
| Contract version | 2 | 1 under export schema 2 |
| Layout | YAML sequence | compact string plus explicit ranges |
| Phase field names | `gait_phase_*` | `phase_*` |
| Stand phase | phase zero, then 0.72 s successful-tick clock | uniform random once, static for episode |
| Readiness loss | freeze advancing clock | preserve static pair; no resample |
| Reset | reset clock to phase zero | begin intentional new episode and resample |
| Stand period | operational 0.72 s | `phase_period_s` present but not used to evolve Stand phase |
| Bundle companions | YAML/ONNX pair | five-file bundle with deployment contract and manifest |

Manually rewriting the exported YAML would break provenance and could pair policy bytes with false runtime semantics. v2.9.0 therefore updates the parser and runtime to consume the v2.3.1 schema unchanged.

## Exact 47-D builder

```text
0:3    commanded [vx, vy, yaw_rate]
3:6    base angular velocity in the policy/base frame
6:9    projected gravity in the policy/base frame
9:21   q - q_default for the 12 actionable joints
21:33  actionable joint velocity
33:45  previous bounded normalized action
45     sin(2*pi*phase)
46     cos(2*pi*phase)
```

The first 45 values are unchanged when only the Stand episode phase changes.

## Stand episode semantics

At policy-node startup, the runtime samples one phase uniformly from `[0,1)`, computes its sine/cosine pair, and holds the pair unchanged. Sensor delays, command changes, inference count, contact state, and elapsed time do not change it. A new sample occurs only when a new policy episode is intentionally begun.

The runtime supports a fixed phase or seed for tests and replay. Deterministic phase injection is refused in live mode.

## Action and hardware preservation

No changes are made to:

- action contract v4 transformation;
- canonical 12-action joint order;
- `q_default`, residual scales, physical limits, or previous-action semantics;
- `joint_map.yaml` or ST3215 calibration;
- servo driver UART ownership, runtime profiles, watchdogs, pose hold, or torque services;
- IMU transform and freshness gates;
- PD safety envelope, maintenance, identification, and calibration tooling.

## Future Walk gate

The architecture includes the exported command-synchronized, non-blocking Walk phase mode. Live Walk remains blocked unless the bundle explicitly pins its deployment stage or period. Track 1 uses stage-specific periods, so a late checkpoint must not silently inherit a fresh exporter environment's Stage-0 metadata.
