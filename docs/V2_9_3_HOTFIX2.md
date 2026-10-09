# LittleGreen ROS 2 v2.9.3 Hotfix 2

Hotfix 2 corrects a ROS 2 Humble `rclpy.node.Node` attribute-name collision in
`policy_handoff_control`.

`Node` exposes `clients` as a read-only property. Hotfix 1 assigned the handoff
service-client dictionary to `self.clients`, which raised:

```text
AttributeError: can't set attribute 'clients'
```

The helper now stores its service clients in the private
`self._service_clients` dictionary instead. No service names, handoff pose,
previous-action seed, policy artifact, safety threshold, driver behavior, or
handoff sequence changed.
