# LittleGreen ROS2 v2.9.3 Hotfix 1

## Purpose
Fix `policy_handoff_control` on ROS 2 Humble installations where
`rclpy.parameter_client.AsyncParameterClient` is unavailable.

The handoff controller now uses the standard ROS parameter service directly:
`/lgh_st3215_driver/set_parameters` (`rcl_interfaces/srv/SetParameters`).

## Scope
Import/service-client compatibility only. No policy, handoff pose,
previous-action seed, safety threshold, servo-driver behavior, observation
contract, action contract, or deployment bundle is changed.

## Apply
Overlay this archive onto an existing v2.9.3 workspace, then rebuild only
`littlegreen_biped_pkg`.
