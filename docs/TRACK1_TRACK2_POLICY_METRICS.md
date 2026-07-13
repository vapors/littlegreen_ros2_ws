# Track 1 / Track 2 Policy Metrics

Track 1 v1.4.5s3 added deeper policy, posture, COM, and gait diagnostics. v2.7.1 propagates the metrics that can be observed faithfully from the current ROS hardware interface and explicitly labels the metrics that still require additional sensing or kinematic estimation.

## Current Track 1 reference geometry

The packaged policy export identifies the deployment contract. The accompanying Track 1 task uses these standing targets for training and evaluation:

```text
standing COM height target:       0.460 m
moving COM height target:         0.430 m
standing COM forward target:      0.070 m
standing COM forward band:        ±0.010 m
projected-gravity-x lean target:   0.065
```

These values are training/evaluation geometry, not direct servo commands. Only the projected-gravity signal is currently available at the ROS policy boundary without adding a base/foot kinematic estimator.

## Runtime recorder

Run during policy shadow or a guarded live session with `publish_policy_debug:=true`:

```bash
ros2 run littlegreen_biped_pkg policy_runtime_metrics \
  --duration-sec 30
```

Default output:

```text
~/.ros/littlegreen_policy_metrics/<UTC timestamp>/
├── timeseries.csv
└── summary.yaml
```

Useful options:

```bash
ros2 run littlegreen_biped_pkg policy_runtime_metrics \
  --duration-sec 60 \
  --standing-command-threshold 0.05 \
  --joint-velocity-limit-rad-s 4.72 \
  --output-dir ~/policy_checks/v145s3_shadow
```

Exit status:

| Code | Meaning |
|---:|---|
| `0` | capture completed |
| `3` | invalid or unsafe command-line precondition |
| `4` | policy debug data unavailable or not synchronized |
| `5` | malformed policy or joint-map configuration |
| `7` | operator interrupted the capture |
| `70` | internal software error |

## Metrics now aligned across Track 1 and Track 2

| Metric | ROS source | Interpretation |
|---|---|---|
| raw action mean magnitude, standard deviation, min, max | `/policy_debug/raw_action` | policy output range and spread before bounding |
| raw action excess fraction | raw action | fraction outside normalized `[-1, 1]` |
| bounded saturation fraction | `/policy_debug/clipped_raw_action` | fraction at the normalized action boundary |
| physical target-limit fraction and clip magnitude | target debug topics and saturation mask | frequency and magnitude of hardware-bound clipping |
| target residual magnitude | target minus exported default | demand relative to the v4 athletic default pose |
| joint tracking error | target plus `/joint_states` | real command-to-position mismatch |
| joint velocity-limit fraction | `/joint_states.velocity` | fraction near the configured velocity threshold |
| projected gravity x/y/z | `/policy_debug/observation[6:9]` | policy-frame body orientation signal |
| observable standing subconditions | observation | upright, quiet-yaw, and near-default tests matching Track 1 thresholds where sensors permit |
| joint posture RMS/max | `/policy_debug/observation[9:21]` | deviation from the exported athletic default |
| command velocity | `/policy_debug/observation[0:3]` | exact command used by the policy observation |

The recorder produces both global means and a standing-command subset. Standing samples are selected by command-vector magnitude, not by an assumption that the robot is physically stationary. It can reproduce the Track 1 upright, quiet-yaw, and near-default subconditions, but it does not report the full stable-standing condition because root XY velocity and foot contact are unavailable.

## Metrics not yet directly observable

The current runtime does not claim these Track 1 diagnostics:

```text
base COM height
COM forward offset relative to the feet
single-support / double-support / no-support fractions
foot air time
swing clearance and lift counts
foot slip
physical joint torque
```

Reasons:

- The robot does not yet publish foot-contact state.
- The runtime does not yet estimate base position or foot poses in a common world frame.
- ST3215 current/load telemetry is useful for health and relative loading but is not a calibrated physical joint-torque measurement.

Do not substitute uncalibrated servo load or current directly for the Track 1 torque metric.

## Recommended near-term extension

The next high-value Track 2 estimator is a read-only base/foot geometry monitor that combines:

```text
joint positions
+ URDF kinematics
+ IMU orientation
+ a clearly stated ground/contact assumption
```

That can provide approximate body height and forward body position relative to the feet during supported standing tests. It should remain a diagnostic estimator and must not be used as a safety authority until its assumptions are validated.

Foot-support and gait metrics should wait for reliable contact sensing or a separately validated contact estimator.

## Comparing Track 1 and hardware

For each policy candidate:

1. run the offline `policy_bundle_audit`;
2. capture a zero-command shadow dataset;
3. capture the same command sequence used in simulation evaluation;
4. compare raw-action excess, normalized saturation, physical target clipping, residual demand, projected gravity, and real tracking error;
5. keep unavailable metrics explicitly blank rather than inventing proxies.

A policy that is stable in simulation but shows substantially higher physical target clipping or tracking error on hardware is seeing a different effective plant. Feed that evidence back into Track 1 rather than changing ROS-side defaults or scales independently.
