# LittleGreen Hardware Limit Tool v1.1.0

Standalone ST3215 physical joint-limit capture and contract generation for the current `littlegreen_ros2_ws` layout. It has no ROS 2 runtime dependency and must be used only while the normal ST3215 driver is stopped.

## Authority model

The tool now keeps four ideas separate:

| Item | Meaning | Changes after model-zero calibration? |
|---|---|---:|
| `center_step` | raw servo position at model zero | yes |
| `training_default_rad` | Track 1 policy-default stance | no |
| `min_rad` / `max_rad` | durable physical/model-space joint limits | no |
| `min_step` / `max_step` | deployment endpoints derived from center + radian limits | yes |

A servo replacement normally needs model-zero calibration, not a new physical-limit capture. Re-capture limits only when the linkage, horn geometry, mechanical stops, or safe range actually changed.

## Default LittleGreen paths

```text
~/littlegreen_ros2_ws/src/lgh_st3215_driver/config/servo_map.yaml
~/littlegreen_ros2_ws/src/lgh_st3215_tools/config/track1_action_contract_v4.yaml
```

Override the workspace root with:

```bash
export LITTLEGREEN_ROS2_WS=/path/to/littlegreen_ros2_ws
```

## Capture physical limits

Stop `lgh_st3215_driver` first. The tool directly owns `/dev/ttyS3` and disables torque.

```bash
cd ~/littlegreen_ros2_ws
python3 tools/lgh_hardware_limit_tool/lgh_hardware_limit_tool.py capture \
  --device /dev/ttyS3 \
  --margin-steps 10 \
  --output-dir ~/lgh_limit_capture
```

The default servo map and deployment contract are discovered automatically.

## Re-capture one joint

```bash
python3 tools/lgh_hardware_limit_tool/lgh_hardware_limit_tool.py capture \
  --device /dev/ttyS3 \
  --margin-steps 10 \
  --output-dir ~/lgh_limit_capture \
  --resume \
  --recapture-joint leg_left_knee_pitch_joint
```

## Re-render after center calibration

After replacing a servo and recalibrating model zero, do not repeat the physical endpoint capture unless the mechanics changed. Re-render the saved model-space contract against the updated `servo_map.yaml`:

```bash
python3 tools/lgh_hardware_limit_tool/lgh_hardware_limit_tool.py render \
  --capture ~/lgh_limit_capture/physical_limit_capture.yaml \
  --margin-steps 10 \
  --output-dir ~/lgh_limit_capture/rendered_after_zero_calibration
```

The new `servo_map.measured_limits.generated.yaml` preserves the captured radian limits and derives raw endpoints from the current `center_step` values.

## Generated artifacts

```text
authoritative_hardware_contract.yaml
track1_hardware_contract.generated.py
servo_map.measured_limits.generated.yaml
comparison_to_deployment_contract.csv
comparison_report.txt
generation_manifest.json
```

The authoritative YAML stores physical and safety-margined limits in model-space radians. Capture-time raw endpoints are retained as provenance; generated runtime raw endpoints use the current calibrated centers.
