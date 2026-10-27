#!/usr/bin/env python3
"""Exercise language-command service contracts against a fresh baseline simulation."""
import argparse
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from rosidl_runtime_py.convert import message_to_ordereddict
from scipy.spatial.transform import Rotation

from classify_interfaces.msg import TaskResult, TaskStatus
from classify_interfaces.srv import RobotCommand
from gazebo_msgs.srv import GetEntityState
from std_srvs.srv import Trigger


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', choices=['guard', 'cancel', 'cross_bin', 'hold'], required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = rclpy.create_node('verify_command_session')
    statuses, results = [], []
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions = [
        node.create_subscription(TaskStatus, '/classify/status', statuses.append, qos),
        node.create_subscription(TaskResult, '/classify/result', results.append, qos),
    ]
    command = node.create_client(RobotCommand, '/classify/command')
    start = node.create_client(Trigger, '/classify/start')
    cancel = node.create_client(Trigger, '/classify/cancel')
    entity = node.create_client(GetEntityState, '/gazebo/get_entity_state')
    record = {'case': args.case, 'passed': False}

    def call(client, request):
        assert client.wait_for_service(timeout_sec=10), client.srv_name
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10)
        assert future.done() and future.result() is not None, client.srv_name
        return future.result()

    def submit(intent, colors=(), quantity=0, target_bin=''):
        request = RobotCommand.Request()
        request.intent, request.colors = intent, list(colors)
        request.quantity, request.target_bin = quantity, target_bin
        request.selector = 'nearest'
        return call(command, request)

    def wait_result(previous):
        deadline = time.monotonic() + 180
        while len(results) <= previous and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        assert len(results) > previous, 'No new task result'
        return results[-1]

    def pose(name):
        request = GetEntityState.Request()
        request.name, request.reference_frame = name, 'world'
        response = call(entity, request)
        assert response.success, name
        return response.state.pose

    def xyz(p):
        return np.array([p.position.x, p.position.y, p.position.z])

    try:
        # Drain latched history before counting results for this session.
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        assert statuses and statuses[-1].state == 'IDLE', 'Requires a fresh task server'
        before = {color: pose(color + '_block') for color in ['red', 'blue', 'green']}
        previous = len(results)
        if args.case in ['guard', 'cancel']:
            # The baseline contains one red cube. Requesting two must fail
            # without motion, then both entry points must require reset.
            response = submit('pick_place', ['red'], 2)
            assert response.accepted, response.message
            if args.case == 'cancel':
                assert call(cancel, Trigger.Request()).success
            result = wait_result(previous)
            expected = 'CANCELLED' if args.case == 'cancel' else 'FAILED'
            assert not result.success and statuses[-1].state == expected
            home = submit('home')
            record['home_after_failure'] = message_to_ordereddict(home)
            assert not home.accepted, expected + ' incorrectly accepted a HOME command'
            retry = call(start, Trigger.Request())
            record['batch_after_failure'] = message_to_ordereddict(retry)
            assert not retry.success, expected + ' incorrectly accepted a batch'
        else:
            holding = args.case == 'hold'
            if holding:
                for quantity in [0, 2]:
                    invalid = submit('pick_hold', ['blue'], quantity)
                    assert not invalid.accepted, 'Holding more than one object must be rejected'
            response = submit('pick_hold' if holding else 'pick_place',
                              ['blue' if holding else 'red'], 1,
                              '' if holding else 'blue')
            assert response.accepted, response.message
            result = wait_result(previous)
            assert result.success, result.detail
            if holding:
                assert statuses[-1].state == 'HOLDING'
                assert statuses[-1].held_object == 'blue_block'
                assert xyz(pose('blue_block'))[2] > xyz(before['blue'])[2] + .12
                home = submit('home')
                record['home_while_holding'] = message_to_ordereddict(home)
                assert not home.accepted, 'Holding must retain controller ownership'
            else:
                assert result.succeeded == 1 and list(result.bins) == ['bin_blue']
                block, bin_pose = pose('red_block'), pose('bin_blue')
                inverse = Rotation.from_quat([
                    getattr(bin_pose.orientation, k) for k in ['x', 'y', 'z', 'w']
                ]).inv()
                center = inverse.apply(xyz(block) - xyz(bin_pose))
                rotation = inverse * Rotation.from_quat([
                    getattr(block.orientation, k) for k in ['x', 'y', 'z', 'w']
                ])
                corners = rotation.apply(np.array([
                    [x, y, z] for x in [-.025, .025]
                    for y in [-.025, .025] for z in [-.025, .025]
                ])) + center
                assert np.max(np.abs(corners[:, :2])) < .054
                assert abs(center[2] - .037) < .006
                assert np.linalg.norm(xyz(pose('blue_block')) - xyz(before['blue'])) < .002
                previous = len(results)
                home = submit('home')
                assert home.accepted, 'Successful placement must allow another command'
                assert wait_result(previous).success
                retry = call(start, Trigger.Request())
                record['batch_after_placement_and_home'] = message_to_ordereddict(retry)
                assert not retry.success, 'Legacy batch must require a fresh full inventory'
        record['green_displacement_mm'] = float(
            np.linalg.norm(xyz(pose('green_block')) - xyz(before['green'])) * 1000)
        assert record['green_displacement_mm'] < 2
        record['passed'] = True
        print('VERIFY_COMMAND_OK:', args.case, flush=True)
    except Exception as error:
        record['failure'] = str(error)
        print('VERIFY_COMMAND_FAILED:', error, flush=True)
    finally:
        record['statuses'] = [message_to_ordereddict(m) for m in statuses]
        record['results'] = [message_to_ordereddict(m) for m in results]
        (args.output / 'record.json').write_text(json.dumps(record, indent=2) + '\n')
        node.destroy_node()
        rclpy.shutdown()
    if not record['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
