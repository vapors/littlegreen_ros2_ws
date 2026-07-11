# Migration from v2.5.1 to v2.6.0

v2.6.0 is a breaking ROS package-identity release. Treat it as a new workspace rather than building over an existing v2.5.1 overlay.

## Recommended migration

1. Archive v2.5.1.
2. Place v2.6.0 at `~/littlegreen_ros2_ws`.
3. Do not copy `build/`, `install/`, or `log/` from the old workspace.
4. Install dependencies and perform a clean build.
5. update external scripts, services, aliases, and firmware references deliberately.

## Workspace and package map

```text
~/berkeley_ros2_ws
  -> ~/littlegreen_ros2_ws
```

| Old | New |
|---|---|
| `bhl_st3215_driver` | `lgh_st3215_driver` |
| `bhl_st3215_tools` | `lgh_st3215_tools` |
| `bhl_st3215_maintenance` | `lgh_st3215_maintenance` |
| `bhl_imu_tools` | `lgh_imu_tools` |
| `berkeley_biped_pkg` | `littlegreen_biped_pkg` |
| `berkeley_biped_node` | `littlegreen_biped_node` |
| `berkeley_biped_launch.py` | `littlegreen_biped_launch.py` |
| `lilgreen_description` | `littlegreen_description` |

## Generated paths

```text
~/.ros/bhl_reports
  -> ~/.ros/lgh_reports

~/.ros/bhl_imu_datasets
  -> ~/.ros/lgh_imu_datasets

~/.ros/bhl_st3215_backups
  -> ~/.ros/lgh_st3215_backups

/tmp/bhl_st3215_dev_ttyS3.lock
  -> /tmp/lgh_st3215_dev_ttyS3.lock
```

Old report directories are not deleted or automatically migrated. Preserve them as historical datasets.

## Build migration

Do not source the old overlay. In a new shell:

```bash
cd ~/littlegreen_ros2_ws
rm -rf build install log
./scripts/install_orange_pi.sh
```

For a host that already has ROS and ONNX Runtime installed:

```bash
./scripts/install_orange_pi.sh --skip-ros --skip-onnx
```

## Source-map migration

The distributed v2.6.0 `servo_map.yaml` is carried forward from v2.5.1. Compare any local v2.5.1 modifications before copying values:

```bash
diff -u \
  ~/workspace_archives/v2_5_1/servo_map.yaml \
  ~/littlegreen_ros2_ws/src/lgh_st3215_driver/config/servo_map.yaml
```

Do not replace the whole new package with an old package directory. Apply only reviewed calibration value changes.

## External references to update manually

Search outside the workspace:

```bash
grep -RInE 'berkeley_ros2_ws|bhl_|berkeley_biped|lilgreen_description' \
  ~/.bashrc ~/.config/systemd/user /etc/systemd/system ~/bin 2>/dev/null
```

Review:

```text
systemd services
shell aliases/functions
micro-ROS launch scripts
deployment-copy scripts
udev rules
analysis notebooks
rosbag helper scripts
remote SSH commands
```

Do not blindly rename historical archive filenames or dataset metadata. Their original names are part of provenance.

## Topics and services

Generic runtime topic and service names are intentionally unchanged. Existing external consumers of these do not need a branding rename:

```text
/joint_states
/imu/data
/desired_position
/servo_target_radians
/joint_feedback_age_ms
/st3215_driver/diagnostics
/st3215_driver/telemetry
/st3215_driver/* services
```

Consumers that identify nodes or packages must be updated.

## Verification

```bash
ros2 pkg list | grep -E '^(bhl_|berkeley_biped|lilgreen_)'
```

Expected: no output.

```bash
ros2 pkg list | grep -E '^(lgh_|littlegreen_)'
```

Expected:

```text
lgh_imu_tools
lgh_st3215_driver
lgh_st3215_maintenance
lgh_st3215_tools
littlegreen_biped_pkg
littlegreen_description
```
