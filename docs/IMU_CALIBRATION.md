# IMU Source, Agent, and Validation

LittleGreen consumes IMU data at the canonical ROS boundary:

```text
/imu/data  sensor_msgs/msg/Imu
```

The current source is the XIAO ESP32-S3 ICM-20948 micro-ROS firmware. The validation tools remain source-independent, so the same checks apply after a future move to direct Orange Pi I2C or SPI.

## 1. Start the current micro-ROS agent

Use a dedicated terminal:

```bash
ros2 run micro_ros_agent micro_ros_agent serial \
  --dev /dev/ttyACM0 \
  -b 115200 \
  -v0
```

Keep this process running during IMU preflight, policy shadow, and live policy operation.

The command does not launch the firmware. It creates the host-side micro-ROS serial transport used by the already-flashed XIAO.

## 2. Confirm the USB serial device

USB CDC numbers can change after reboots or reconnection:

```bash
ls -l /dev/ttyACM*
ls -l /dev/serial/by-id/
```

When the controller appears on a different device, change only the `--dev` value:

```bash
ros2 run micro_ros_agent micro_ros_agent serial \
  --dev /dev/ttyACM1 \
  -b 115200 \
  -v0
```

Use a stable `/dev/serial/by-id/...` path when one is available and verified.

## 3. Verify the ROS boundary

```bash
ros2 topic list -t | grep imu
ros2 topic hz /imu/data
ros2 topic echo /imu/data --once
ros2 topic info /imu/data --verbose
```

Expected source characteristics for the current firmware include:

```text
topic: /imu/data
message: sensor_msgs/msg/Imu
frame_id: imu_link
rate: approximately 100 Hz
```

The policy node uses sensor-data QoS for the IMU. A best-effort sensor publisher is expected.

## 4. Run IMU preflight

```bash
ros2 run lgh_imu_tools imu_preflight
```

Expanded example:

```bash
ros2 run lgh_imu_tools imu_preflight \
  --duration-sec 5 \
  --timeout-sec 10 \
  --output-root ~/.ros/lgh_reports
```

The contract file is installed with `lgh_imu_tools` and can be overridden:

```bash
ros2 run lgh_imu_tools imu_preflight \
  --contract /absolute/path/to/imu_contract.yaml
```

## 5. Stationary characterization

Place the supported robot and IMU in a stable pose:

```bash
ros2 run lgh_imu_tools stationary_characterization \
  --topic /imu/data \
  --duration-sec 20
```

This records stationary noise and stability data under:

```text
~/.ros/lgh_imu_datasets
```

Use the same support condition and duration when comparing firmware, filters, mounts, or transport changes.

## 6. Orientation audit

Neutral pose:

```bash
ros2 run lgh_imu_tools orientation_audit \
  --pose neutral
```

Known-direction example:

```bash
ros2 run lgh_imu_tools orientation_audit \
  --pose forward_pitch \
  --expected-axis x \
  --expected-sign positive \
  --minimum-magnitude 0.5
```

Options:

```text
--pose TEXT                         required operator label
--contract PATH                     optional contract override
--duration-sec FLOAT                default 3.0
--expected-axis x|y|z               optional sign check axis
--expected-sign positive|negative   optional sign check
--minimum-magnitude FLOAT           default 0.5
--output-root PATH                  default ~/.ros/lgh_imu_audits
```

Do not guess an axis/sign expectation. Use a controlled physical orientation and record exactly how the robot was moved.

## 7. Raw recorder

```bash
ros2 run lgh_imu_tools imu_recorder \
  --topic /imu/data \
  --duration-sec 10
```

Use the recorder for side-by-side comparisons or when a policy readiness failure needs the original message stream preserved.

## 8. Policy-frame transform

The policy runtime applies `imu_to_base_matrix` from:

```text
src/littlegreen_biped_pkg/src/configs/policy_runtime.yaml
```

Current matrix semantics:

```text
x_base =  y_imu
y_base = -x_imu
z_base =  z_imu
```

Changing the sensor mount, firmware orientation convention, or source driver requires repeating the orientation audit before changing this matrix.

## 9. Policy shadow and live requirements

For real hardware, keep:

```text
override_imu:=false
```

`override_imu:=true` substitutes nominal values and is useful only for limited software inspection. It is not a valid real-hardware balance test.

The policy freshness gate defaults to:

```text
imu_timeout_sec: 0.050
```

At a nominal 100 Hz IMU rate, occasional transport jitter may be tolerated, but a stopped agent or disconnected USB device should quickly make the policy unready.

## 10. Troubleshooting

### Agent starts but `/imu/data` is absent

```bash
ls -l /dev/ttyACM*
ros2 node list
ros2 topic list -t
```

Check that:

- the correct XIAO device is selected;
- no second micro-ROS agent owns the same device;
- the firmware is running and uses the expected serial transport/baud;
- the user has permission to open the device.

### Device is busy

```bash
lsof /dev/ttyACM0
fuser -v /dev/ttyACM0
```

Stop the stale agent or serial monitor that owns the port.

### Topic rate is unstable

```bash
ros2 topic hz /imu/data
ros2 run lgh_imu_tools stationary_characterization --duration-sec 30
```

Record whether the issue changes with USB reconnect, power sequencing, or another cable/port before changing policy freshness limits.

### Policy reports stale IMU

Check the source and policy process separately:

```bash
ros2 topic hz /imu/data
ros2 topic echo /imu/data --once
ros2 node info /littlegreen_biped_node
```

Do not disable the freshness gate to hide a stopped or unreliable source.
