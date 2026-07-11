# Integration with the current littlegreen_ros2_ws

The native driver is intentionally compatible with the current policy + PD graph, so no topic rename is required for the first pass.

## Current graph

```text
/imu/data ---------------------------> littlegreen_biped_node
/joint_states -----------------------> littlegreen_biped_node
/joint_feedback_age_ms -------------> littlegreen_biped_node

littlegreen_biped_node
  /desired_position
        |
        v
pd_controller_node
  /servo_target_radians
        |
        v
lgh_st3215_driver
        |
        +--> /joint_states
        +--> /joint_feedback_age_ms
        +--> /st3215_driver/diagnostics
```

## Recommended commissioning order

### Terminal 1: native servo driver, feedback only

```bash
source /opt/ros/humble/setup.bash
source ~/littlegreen_ros2_ws/install/setup.bash

ros2 launch lgh_st3215_driver lgh_st3215_driver.launch.py \
  enable_writes:=false
```

### Terminal 2: policy with IMU override for interface testing

```bash
source /opt/ros/humble/setup.bash
source ~/littlegreen_ros2_ws/install/setup.bash

ros2 run littlegreen_biped_pkg littlegreen_biped_node \
  --ros-args \
  -p override_imu:=true
```

The policy should move from waiting for joint state / hardware ages to ready once both native feedback topics are fresh and complete.

### Terminal 3: inspect diagnostics

```bash
ros2 topic hz /joint_states
ros2 topic hz /joint_feedback_age_ms
ros2 topic echo /joint_feedback_age_ms --once
ros2 topic echo /st3215_driver/diagnostics --once
```

## Adding the driver to the existing launch file later

After feedback-only and write-enabled commissioning, the following node can be added to `littlegreen_biped_launch.py`:

```python
Node(
    package='lgh_st3215_driver',
    executable='lgh_st3215_driver_node',
    name='lgh_st3215_driver',
    output='screen',
    parameters=[
        [FindPackageShare('lgh_st3215_driver'), '/config/servo_driver.yaml'],
        {
            'port': '/dev/ttyS3',
            'writes_enabled': True,
            'joint_map_path': [
                FindPackageShare('lgh_st3215_driver'),
                '/config/servo_map.yaml',
            ],
        },
    ],
),
```

For the first integrated live test, launching the servo driver separately is preferable because `enable_writes` remains visually explicit at the command line.
