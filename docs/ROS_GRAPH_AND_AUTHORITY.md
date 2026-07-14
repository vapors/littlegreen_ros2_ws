# ROS Graph and Command Authority

LittleGreen intentionally keeps hardware, policy, controller, teleop, and sensor processes separate. This makes each authority boundary visible, but it also means an old publisher can remain active after another launch is restarted.

## Canonical command chain

```text
joystick / keyboard / other command source
            │
            ▼
     /command_velocity
            │
            ▼
 littlegreen_biped_node
            │
       /desired_position
            │
            ▼
   pd_controller_node
            │
   /servo_target_radians
            │
            ▼
  lgh_st3215_driver
            │
       /dev/ttyS3
```

In policy shadow mode, the policy publishes only `/policy_shadow/desired_position`, so it has no live servo authority.

## Authority states inside the driver

| Driver condition | External publisher accepted? | Physical writes? |
|---|:---:|:---:|
| `enable_writes:=false` | command may be received/cached | no |
| writes enabled, no pose override | yes | yes, subject to feedback and limit gates |
| `hold_current_pose` active | no | current/last safe pose is held |
| policy-default ramp active | no | guarded internal ramp |
| policy-default reached with default settings | no | policy-default pose held |
| `release_pose_override` called | yes, immediately | yes |
| torque disabled | external command blocked by override | torque off |

`startup_hold_current_position=true` seeds the command buffer from measured state. It is not an authority lock. A later active `/servo_target_radians` publisher can replace that target when no pose override is active.

## Before calibration or maintenance

Calibration should have no command publisher:

```bash
ros2 topic info /servo_target_radians --verbose
```

Expected:

```text
Publisher count: 0
```

Find likely nodes:

```bash
ros2 node list | grep -E 'biped|policy|pd_controller|identification|standing|teleop'
```

Inspect a specific publisher:

```bash
ros2 node info /pd_controller_node
ros2 node info /littlegreen_biped_node
```

## Before releasing a pose override

The release service returns authority immediately. Inspect the graph first:

```bash
ros2 topic info /servo_target_radians --verbose
ros2 topic echo /servo_target_radians --once
```

Then release only when the publisher and target are intentional:

```bash
ros2 service call \
  /st3215_driver/release_pose_override \
  std_srvs/srv/Trigger '{}'
```

## Verify launch shutdown

After pressing `Ctrl+C`, verify that the nodes actually stopped:

```bash
ros2 node list
ros2 topic info /servo_target_radians --verbose
ros2 topic info /desired_position --verbose
ros2 topic info /command_velocity --verbose
```

Useful process-level checks:

```bash
ps -ef | grep -E 'ros2|littlegreen|pd_controller|micro_ros_agent' | grep -v grep
```

Use normal `Ctrl+C` shutdown first. Kill a process only after identifying it unambiguously.

## Recommended terminal layout

### Feedback or calibration

```text
Terminal A: ST3215 driver
Terminal B: preflight/calibration tool
Terminal C: topic and authority inspection
```

No policy or PD controller should be running.

### Policy shadow

```text
Terminal A: micro-ROS agent for /imu/data
Terminal B: runtime_safe driver, writes disabled
Terminal C: policy_shadow.launch.py
Terminal D: preflight, metrics, and graph checks
```

### Live policy

```text
Terminal A: micro-ROS agent
Terminal B: runtime_safe driver, writes enabled
Terminal C: policy_live.launch.py
Terminal D: joystick/command source, if used
Terminal E: diagnostics and emergency operator console
```

The driver, policy launch, and micro-ROS agent are separate by design. Stopping one does not automatically stop the others.

## Gait-phase reset authority

For a future 47-D phase-guided policy, `/policy/reset_gait_phase` changes only the policy node's logical observation clock. It does not command the driver, release a pose override, change torque, or reset servo feedback.

The service is intentionally limited:

| Policy output mode | Phase reset |
|---|---|
| `disabled` | allowed |
| `shadow` | allowed |
| `live` | refused |

A live phase reset could abruptly change the ONNX input while the policy has command authority. To start a new live deployment episode at phase zero, stop the guarded live policy and restart it after confirming the ROS graph and physical support. The phase clock freezes during readiness loss and therefore does not need an automatic reset when IMU or joint feedback recovers.
