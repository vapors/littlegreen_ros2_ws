# Track 2 v2.4.3 — Standing Load Characterization

## Purpose

v2.4.3 keeps the v2.4.2 fixed ST3215 `speed=0`, `acceleration=0` motion profile and adds a separate whole-body standing/crouch characterization workflow.

The single-joint `servo_identification` remains unchanged. New standing work uses `standing_characterization`.

## Native-driver additions

Two explicit all-servo torque services are added:

- `/st3215_driver/disable_torque_all`
- `/st3215_driver/enable_torque_hold_current`

Torque writes are queued to and executed by the single UART-owning bus worker. ROS service callbacks never access the serial port directly.

`enable_torque_hold_current`:

1. requires fresh complete feedback;
2. blocks external target publishers with the internal pose override;
3. seeds the command buffer with the measured physical pose;
4. allows several bus cycles to write that measured goal while torque is still off;
5. enables torque;
6. leaves the pose override active until `/st3215_driver/release_pose_override` is called.

`ServoTelemetry.msg` adds `torque_enabled_state`:

- `-1`: unknown/not commanded through the new service path;
- `0`: disabled;
- `1`: enabled.

## Runner modes

### `capture_pose`

- requires explicit ARM phrase;
- disables all servo torque;
- lets the operator manually position the mechanically supported robot;
- stores operator-measured `base_com_height_mean_m`;
- captures a median 12-joint pose from cycle telemetry;
- writes/updates a YAML pose library.

Torque remains off by default after capture. Optional `--reenable-torque-hold-after-capture` requires a second explicit ARM phrase.

### `evaluate`

- policy and PD controller must be absent;
- loads named whole-body poses from the YAML library;
- enables torque at measured pose and releases override only after a live 50 Hz reference stream exists;
- uses smoothstep whole-body transitions;
- separates `--crouch-speed-rad-s` and `--stand-return-speed-rad-s`;
- optionally returns to standing between crouch poses;
- records transition, settle, and static-hold telemetry.

Output includes:

- `timeseries.csv`
- `pose_joint_summary.csv`
- `pose_level_summary.csv`
- `bilateral_pose_summary.csv`
- `transition_joint_summary.csv`
- `metadata.yaml`
- `summary.txt`

## Important interpretation

Standing data adds configuration- and load-dependent actuator behavior, but does not automatically identify physical stiffness in N·m/rad. Without known joint torque or force-platform/force-sensor data, report loaded deflection, error, load proxy, current, voltage, temperature, stability, and bilateral asymmetry rather than inventing joint torque.
