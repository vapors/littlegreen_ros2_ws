#!/usr/bin/env python3
"""Capture a non-motion ST3215 hardware/ROS snapshot as YAML."""
from __future__ import annotations
import argparse, math, socket, time
from datetime import datetime, timezone
from pathlib import Path
import yaml
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import UInt32MultiArray
from lgh_st3215_tools.exit_codes import ExitCode
from lgh_st3215_tools.result import make_report_dir

class SnapshotNode(Node):
    def __init__(self) -> None:
        super().__init__('st3215_hardware_snapshot')
        self.diag = None; self.joint = None; self.ages = None
        self.create_subscription(DiagnosticArray, '/st3215_driver/diagnostics', self._diag, 10)
        self.create_subscription(JointState, '/joint_states', self._joint, qos_profile_sensor_data)
        self.create_subscription(UInt32MultiArray, '/joint_feedback_age_ms', self._age, qos_profile_sensor_data)
    def _diag(self, msg):
        for status in msg.status:
            if status.name == 'ST3215 native single bus':
                self.diag = {'level': int(status.level), 'message': status.message, 'hardware_id': status.hardware_id, 'values': {v.key: v.value for v in status.values}}
    def _joint(self, msg):
        self.joint = {'stamp': {'sec': int(msg.header.stamp.sec), 'nanosec': int(msg.header.stamp.nanosec)}, 'frame_id': msg.header.frame_id, 'name': list(msg.name), 'position': list(msg.position), 'velocity': list(msg.velocity), 'effort': list(msg.effort)}
    def _age(self, msg): self.ages = [int(v) for v in msg.data]

def main() -> int:
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout-sec', type=float, default=5.0)
    parser.add_argument('--output-root', type=Path, default=None)
    args=parser.parse_args()
    out=make_report_dir(args.output_root, 'st3215_hardware_snapshot')
    rclpy.init(); node=SnapshotNode(); deadline=time.monotonic()+args.timeout_sec
    try:
        while time.monotonic()<deadline and rclpy.ok() and (node.diag is None or node.joint is None or node.ages is None):
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.destroy_node(); rclpy.shutdown(); return int(ExitCode.INTERRUPTED_BY_SIGINT)
    topic_publishers={topic: sorted({i.node_name for i in node.get_publishers_info_by_topic(topic)}) for topic in ['/joint_states','/joint_feedback_age_ms','/st3215_driver/telemetry','/st3215_driver/raw_position_steps','/st3215_driver/raw_speed','/servo_target_radians']}
    payload={
        'schema_version': 1,
        'timestamp_utc': datetime.now(timezone.utc).isoformat(),
        'hostname': socket.gethostname(),
        'diagnostics': node.diag,
        'joint_state': node.joint,
        'feedback_age_ms': node.ages,
        'topic_publishers': topic_publishers,
        'nodes': sorted(name for name,_ns in node.get_node_names_and_namespaces()),
    }
    (out/'hardware_snapshot.yaml').write_text(yaml.safe_dump(payload, sort_keys=False, width=140))
    ok=node.diag is not None and node.joint is not None and node.ages is not None
    code=ExitCode.PASS if ok else ExitCode.TIMEOUT_OR_UNAVAILABLE
    print(f'ST3215 HARDWARE SNAPSHOT: {"PASS" if ok else "INCOMPLETE"}')
    print(out/'hardware_snapshot.yaml'); print(f'exit_code: {int(code)}')
    node.destroy_node(); rclpy.shutdown(); return int(code)
if __name__=='__main__': raise SystemExit(main())
