# v2.2 Calibration Toolchain Changes

## Native driver

Added 50 Hz canonical-order raw hardware topics:

- `/st3215_driver/raw_position_steps` (`std_msgs/msg/Int32MultiArray`)
- `/st3215_driver/raw_speed` (`std_msgs/msg/Int32MultiArray`)

These are sourced directly from the state snapshot used for `/joint_states`.

## Calibration scripts

- `print_default_pose_reference.py`
- `capture_default_pose_calibration.py`
- `apply_servo_calibration.py`
- `verify_default_pose_calibration.py`
- shared `calibration_common.py`

### Capture

Collects fresh raw steps, calculates median measured default-pose steps, proposes
`center_step` corrections, checks usable step-range margins, classifies mechanical
alignment risk, and generates YAML/CSV/text artifacts. Capture is read-only.

### Apply

Dry-run by default. Validates proposal identity and source-map hash, prints a
unified diff, refuses blocking calibration flags, makes a timestamped backup, and
updates only `center_step` fields when `--apply` is explicitly supplied.

### Verify

Collects calibrated `/joint_states` samples and compares each median joint angle
to `training_default_rad`. Produces YAML and CSV verification reports.

## Calibration model

Robot-specific calibration remains visible in `servo_map.yaml`:

```text
raw ST3215 step
    ↓
center_step + servo_sign + joint_zero_rad
    ↓
physical joint radians
```

No tool in this update writes ST3215 EEPROM center offsets.
