# lgh_icm20948_microros_pio_v_1

PlatformIO / Arduino / micro-ROS firmware for the LittleGreen Humanoid Lite IMU controller.

This release promotes the IMU track from the raw bring-up baseline to the first real orientation build. The servo ESP32 still owns `/servo_target_radians`, `/joint_states`, and `/st3215_feedback_debug`; this firmware owns `/imu/data`, `/imu/calibrate`, and optional `/imu/debug`.

## v1.0.1 debug transport fix

`/imu/data` remains best-effort at 100 Hz. `/imu/debug` is now a reliable publisher at 2 Hz so the rich diagnostic string can be fragmented by XRCE-DDS instead of exceeding the best-effort transport MTU. The debug payload also reports `dbg_pub` and `dbg_fail` counters.

## v_1 target

Publishes:

```text
/imu/data    sensor_msgs/msg/Imu    best-effort publisher, 100 Hz default
```

Optional/control topics:

```text
/imu/debug       std_msgs/msg/String    enabled in *_debug environments
/imu/calibrate   std_msgs/msg/Bool      true starts stationary gyro calibration
```

Default v_1 data policy:

```text
frame_id:              imu_link
publish rate:          100 Hz
transport:             USB serial micro-ROS
QoS:                   best effort for /imu/data
angular_velocity:      rad/s
linear_acceleration:   m/s^2, accelerometer/proper acceleration including gravity
orientation:           6-axis Madgwick gyro + accel estimate
orientation validity:  valid in Madgwick builds, invalid in raw builds
```

The v_1 orientation estimate does **not** use the ICM-20948 magnetometer yet. Roll and pitch are the useful first-order outputs. Yaw will drift over time.

## Hardware target

Default board:

```text
Seeed Studio XIAO ESP32-S3
ICM-20948 breakout over I2C
```

Default wiring:

```text
XIAO 3V3  -> ICM-20948 VIN / VCC, if breakout supports 3.3 V input
XIAO GND  -> ICM-20948 GND
XIAO D4   -> ICM-20948 SDA   GPIO5
XIAO D5   -> ICM-20948 SCL   GPIO6
```

The firmware scans both common ICM-20948 I2C addresses:

```text
0x68
0x69
```

## Build environments

Recommended v_1 orientation/debug build:

```bash
pio run -e xiao_esp32s3_v1_orientation_debug -t upload
```

Other useful builds:

```bash
pio run -e xiao_esp32s3_v1_orientation
pio run -e xiao_esp32s3_raw_debug
pio run -e xiao_esp32s3_raw
pio run -e xiao_esp32s3_madgwick_debug
pio run -e xiao_esp32s3_fast_orientation_debug
```

The default PlatformIO environment is `xiao_esp32s3_v1_orientation_debug`.

## Run the micro-ROS agent

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200
```

or:

```bash
./scripts/run_agent.sh /dev/ttyACM0 115200
```

## ROS 2 checks

First checks:

```bash
ros2 topic hz /imu/data
ros2 topic echo /imu/data --once
ros2 topic echo /imu/debug --once --full-length
```

Concurrent throughput checks with the ST3215 controller:

```bash
ros2 topic hz /joint_states
ros2 topic hz /imu/data
ros2 topic hz /servo_target_radians
```

## Calibration

Stationary gyro calibration is enabled by default. Keep the robot completely still and publish:

```bash
ros2 topic pub --once /imu/calibrate std_msgs/msg/Bool "{data: true}"
```

The firmware averages `IMU_CALIBRATION_SAMPLES` samples and stores the gyro bias in ESP32 NVS Preferences. v_1 debug makes the calibration status clearer:

```text
calib_state=idle|active|complete|disabled
calib_progress=500/500
calib_save_event=0|1
calib_saves=<persistent save count>
bias_loaded=0|1
boot=<persistent boot count>
```

`calib_save_event` is a short one-shot flag; `calib_saves` and `bias_loaded` are the longer-lived fields to confirm persistence.

Accel calibration is intentionally disabled by default because it depends on the mounted gravity direction. To enable accel bias calibration, build with:

```ini
-D IMU_CALIBRATE_ACCEL=1
-D IMU_STATIC_ACCEL_X_MPS2=0.0
-D IMU_STATIC_ACCEL_Y_MPS2=0.0
-D IMU_STATIC_ACCEL_Z_MPS2=9.80665
```

## v_1 debug additions

`/imu/debug` now includes:

```text
ver=...
boot=...
nvs_loaded=...
bias_loaded=...
calib_state=...
calib_progress=current/target
sample_age_ms=...
sample_dt_ms=...
sample_dt_min=...
sample_dt_max=...
sample_dt_avg=...
pub_age_ms=...
pub_dt_ms=...
pub_dt_min=...
pub_dt_max=...
pub_dt_avg=...
accel_mag=...
orient_mode=madgwick_6axis|identity_placeholder
orient_valid=...
q=[w,x,y,z]
rpy_deg=[roll,pitch,yaw]
```

This makes it easier to check whether the ROS 2 observation stack is receiving fresh IMU data and whether the orientation build is behaving sensibly.

## Orientation modes

Madgwick environments enable a 6-axis gyro + accel orientation estimate:

```bash
pio run -e xiao_esp32s3_v1_orientation_debug -t upload
```

Raw environments still publish an identity quaternion and mark orientation invalid:

```text
orientation_covariance[0] = -1.0
```

The raw builds remain useful for isolating policy-node plumbing if the fusion output needs to be disabled during testing.

## Key files

```text
platformio.ini            Build flags and environment variants
src/main.cpp              Fixed-rate loop, scheduler, orientation update guard
src/icm20948_bus.cpp      Minimal ICM-20948 I2C driver, NVS calibration/boot count, sample stats
src/microros_node.cpp     micro-ROS publishers/subscriber, timestamping, debug stats
src/madgwick_filter.cpp   6-axis orientation fusion and quaternion-to-Euler debug helper
include/imu_config.h      Compile-time settings
include/imu_types.h       Sample/status/calibration structs
config/imu_config.yaml    ROS-facing config mirror/documentation
```

## Notes

- USB `Serial` is reserved for the micro-ROS transport. Do not enable serial debug while connected to the micro-ROS agent unless you intentionally move micro-ROS to another transport.
- `/imu/data` uses best-effort publishing to avoid blocking the transport and producing stale observations.
- The policy/controller side should reject IMU samples by timestamp age if the agent or sensor drops out. A first cutoff around 50 ms is reasonable for 100 Hz IMU data.