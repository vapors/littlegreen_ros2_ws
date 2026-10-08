# LittleGreen ROS 2 v2.9.2 — Track 1 v10.2 sim-to-real bridge

## Purpose

v2.9.2 makes the Track-2 policy runtime consume deployment bundles exported from
`Velocity-Lilgreen-Locomotion-ST3215-Loaded-v102` without changing the trained
47-D observation or 12-D action contracts.

The packaged default policy is still the v2.3.1 Stand bundle.  v10.2 locomotion is
installed only after an explicit Track-1 export and audit.

## v10.2 observation contract

Track 2 builds the same actor vector used by Track 1:

```text
0:2    command velocity [vx, vy, yaw_rate]
3:5    base angular velocity
6:8    projected gravity
9:20   joint position relative to q_default
21:32  joint velocity
33:44  previous bounded normalized action
45     phase_sin = 0.0
46     phase_cos = 1.0
```

`phase_mode: neutral_static` is not a gait clock.  It is a constant observation
feature.  The ROS 2 runtime therefore does not infer cadence or advance phase for
v10.2.

## Command contract

The Track-1 exporter records the resolved training envelope in `policy.yaml` and
`deployment_contract.yaml`.  For v10.2:

```text
vx        [-0.45, +0.65] m/s
vy        [-0.30, +0.30] m/s
yaw_rate  [-0.50, +0.50] rad/s
```

Track 2 clamps `/command_velocity` axis-by-axis to these exported limits before
constructing the observation.  This prevents an accidental joystick scale from
feeding out-of-distribution commands to the policy.

## Export on the Track-1 training machine

From the v10.2 source tree:

```bash
cd ~/Littlegreen-Humanoid-Lite
python -m pip install -e source/littlegreen_humanoid_lite

python scripts/rsl_rl/export_policy.py \
  --task Velocity-Lilgreen-Locomotion-ST3215-Loaded-v102 \
  --load_run <V10_2_RUN> \
  --checkpoint model_10000.pt \
  --num_envs 1 \
  --headless
```

The run's `exported/` directory must contain:

```text
policy.onnx
policy.pt
deployment_contract.yaml
policy.yaml
policy.sha256
bundle_manifest.yaml
```

Do not hand-edit `policy.yaml` or `deployment_contract.yaml` after export.

## Audit and install on Track 2

Copy the complete `exported/` directory to the Orange Pi, then run from the source
workspace:

```bash
cd ~/littlegreen_ros2_ws

python3 src/littlegreen_biped_pkg/scripts/install_exported_policy_bundle.py \
  --bundle-dir ~/path/to/exported \
  --dry-run
```

A dry-run must report `AUDIT PASS`.  Then install:

```bash
python3 src/littlegreen_biped_pkg/scripts/install_exported_policy_bundle.py \
  --bundle-dir ~/path/to/exported
```

Rebuild so installed package resources cannot refer to stale policy artifacts:

```bash
rm -rf build/littlegreen_biped_pkg install/littlegreen_biped_pkg
colcon build --symlink-install --packages-select littlegreen_biped_pkg
source install/setup.bash
```

Run the package audit again before inference:

```bash
ros2 run littlegreen_biped_pkg policy_bundle_audit
```

## First runtime stage: shadow only

Keep the robot mechanically supported and the servo driver in write-disabled mode.
Start the normal sensor stack, then launch policy shadow mode.  Confirm:

- policy bundle audit passes;
- policy input is 47-D and output is 12-D;
- phase debug remains exactly `[0,1]` for v10.2;
- command clamping reports the exported envelope;
- IMU and joint-state freshness gates remain healthy;
- q-targets stay inside the exported physical limits;
- no unexpected action saturation or command jumps occur.

Only after shadow logs match the Track-1 contract should live servo writes be tested.

## Contract intentionally unchanged

v2.9.2 does not change:

- canonical 12-joint order;
- action contract v4;
- q_default or residual action scales;
- physical joint limits;
- ST3215 servo calibration or signs;
- IMU-to-base transform;
- 50 Hz policy rate;
- driver write gating, feedback-age checks, or safety ownership.
