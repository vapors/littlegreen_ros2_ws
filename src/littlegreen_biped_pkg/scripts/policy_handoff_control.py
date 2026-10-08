#!/usr/bin/env python3
"""Guarded Track-1 zero-command -> Track-2 live-policy handoff controller.

This tool does not redefine q_default. It loads the SHA-bound learned zero-command
handoff pose, configures the ST3215 driver's guarded pose target, and coordinates
policy previous-action seeding with driver pose-override release.

Stages:
  pose     Ramp to the learned zero-command handoff pose and keep driver override.
  arm      Arm the policy at the already-held handoff pose; keep driver override.
  live     Ramp, arm, release driver override, then enable policy authority.
  release  Arm at the current handoff pose, release override, enable authority.
  disable  Disable policy authority and latch the measured current pose in driver.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from std_srvs.srv import Trigger
import yaml

NUM_ACTIONS = 12


def load_profile(path: Path) -> dict:
    data = yaml.safe_load(path.read_text(encoding='utf-8'))
    if not isinstance(data, dict):
        raise ValueError('handoff profile must be a YAML mapping')
    if int(data.get('schema_version', -1)) != 1:
        raise ValueError('handoff profile schema_version must be 1')
    if data.get('mode') != 'learned_zero_command':
        raise ValueError('handoff profile mode must be learned_zero_command')
    pose = data.get('joint_position_rad')
    prev = data.get('previous_action_bounded')
    if not isinstance(pose, list) or len(pose) != NUM_ACTIONS:
        raise ValueError('joint_position_rad must contain 12 values')
    if not isinstance(prev, list) or len(prev) != NUM_ACTIONS:
        raise ValueError('previous_action_bounded must contain 12 values')
    return data


class HandoffController(Node):
    def __init__(self) -> None:
        super().__init__('policy_handoff_control')
        self.driver_params = AsyncParameterClient(self, 'lgh_st3215_driver')
        self.clients = {
            'move': self.create_client(Trigger, '/st3215_driver/move_to_policy_handoff_pose'),
            'release': self.create_client(Trigger, '/st3215_driver/release_pose_override'),
            'hold': self.create_client(Trigger, '/st3215_driver/hold_current_pose'),
            'arm': self.create_client(Trigger, '/policy/arm_handoff'),
            'enable': self.create_client(Trigger, '/policy/enable_authority'),
            'disable': self.create_client(Trigger, '/policy/disable_authority'),
        }

    def wait_future(self, future, timeout: float):
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done():
            raise TimeoutError(f'operation timed out after {timeout:.1f}s')
        exc = future.exception()
        if exc is not None:
            raise RuntimeError(str(exc))
        return future.result()

    def configure_driver_pose(self, pose: list[float], timeout: float = 5.0) -> None:
        if not self.driver_params.wait_for_service(timeout_sec=timeout):
            raise RuntimeError('driver parameter service is unavailable')
        params = [
            Parameter('policy_handoff_pose_rad', value=[float(x) for x in pose]),
            Parameter('policy_handoff_pose_enabled', value=True),
        ]
        result = self.wait_future(self.driver_params.set_parameters(params), timeout)
        if result is None or len(result) != len(params):
            raise RuntimeError('driver parameter update returned no complete result')
        failures = [item.reason for item in result if not item.successful]
        if failures:
            raise RuntimeError('driver rejected handoff parameters: ' + '; '.join(failures))

    def call_trigger(self, key: str, timeout: float = 5.0):
        client = self.clients[key]
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f'service unavailable: {client.srv_name}')
        response = self.wait_future(client.call_async(Trigger.Request()), timeout)
        if response is None:
            raise RuntimeError(f'no response from {client.srv_name}')
        return response

    def require_trigger(self, key: str, timeout: float = 5.0):
        response = self.call_trigger(key, timeout)
        print(f'{self.clients[key].srv_name}: success={response.success} message={response.message}')
        if not response.success:
            raise RuntimeError(response.message)
        return response

    def arm_with_retry(self, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        last = ''
        while time.monotonic() < deadline:
            response = self.call_trigger('arm', timeout=min(3.0, max(0.5, deadline - time.monotonic())))
            last = response.message
            if response.success:
                print(f'{self.clients["arm"].srv_name}: success=True message={response.message}')
                return
            time.sleep(0.25)
        raise RuntimeError(f'policy handoff never became armable: {last}')

    def release_with_retry(self, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        last = ''
        while time.monotonic() < deadline:
            response = self.call_trigger('release', timeout=min(3.0, max(0.5, deadline - time.monotonic())))
            last = response.message
            if response.success:
                print(f'{self.clients["release"].srv_name}: success=True message={response.message}')
                return
            time.sleep(0.20)
        raise RuntimeError(f'driver pose override could not be released: {last}')


def main() -> int:
    default_profile = Path(get_package_share_directory('littlegreen_biped_pkg')) / 'configs' / 'policy_handoff.yaml'
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('stage', choices=['pose', 'arm', 'live', 'release', 'disable'])
    parser.add_argument('--profile', type=Path, default=default_profile)
    parser.add_argument('--ramp-wait-sec', type=float, default=4.5)
    parser.add_argument('--settle-timeout-sec', type=float, default=8.0)
    args = parser.parse_args()

    try:
        profile = load_profile(args.profile.expanduser().resolve())
    except (OSError, ValueError, yaml.YAMLError) as exc:
        print(f'HANDOFF CONTROL: CONFIG ERROR: {exc}', file=sys.stderr)
        return 5

    rclpy.init()
    node = HandoffController()
    try:
        if args.stage == 'disable':
            node.require_trigger('disable')
            node.require_trigger('hold')
            print('HANDOFF CONTROL: POLICY DISABLED; CURRENT PHYSICAL POSE HELD')
            return 0

        if args.stage in {'pose', 'live'}:
            node.configure_driver_pose(profile['joint_position_rad'])
            print(
                'Configured driver handoff pose from SHA-bound profile: '
                f"task={profile.get('task')} sha={profile.get('policy_sha256')}"
            )
            node.require_trigger('move')
            time.sleep(max(0.0, args.ramp_wait_sec))
            if args.stage == 'pose':
                print('HANDOFF CONTROL: POSE STAGE COMPLETE; DRIVER OVERRIDE REMAINS ACTIVE')
                print('Measure IMU/posture now. Do not release pose override until ready for live handoff.')
                return 0

        if args.stage == 'arm':
            node.arm_with_retry(args.settle_timeout_sec)
            print('HANDOFF CONTROL: POLICY ARMED; DRIVER OVERRIDE REMAINS ACTIVE')
            return 0

        if args.stage in {'live', 'release'}:
            if args.stage == 'release':
                node.configure_driver_pose(profile['joint_position_rad'])
            node.arm_with_retry(args.settle_timeout_sec)
            node.release_with_retry(args.settle_timeout_sec)
            try:
                node.require_trigger('enable')
            except Exception:
                # Once the driver override is released, any failure to enable policy
                # immediately re-latches the latest measured physical pose.
                try:
                    node.require_trigger('hold')
                finally:
                    raise
            print('HANDOFF CONTROL: LIVE POLICY AUTHORITY ENABLED')
            return 0

        raise RuntimeError(f'unhandled stage: {args.stage}')
    except (RuntimeError, TimeoutError) as exc:
        print(f'HANDOFF CONTROL: FAIL: {exc}', file=sys.stderr)
        return 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
