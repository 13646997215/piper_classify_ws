#!/usr/bin/env python3
"""Simulation-only integration checks for confirmed scene updates and freshness.

Use --displacement with the detector running. Use --injection only after stopping
that detector; it injects diagnostic observations, then leaves the scene stale.
Gazebo is used for test setup/reference only, never as a perception input.
"""
import argparse
import copy
import json
from pathlib import Path
import subprocess
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from classify_interfaces.msg import DetectedBlock, DetectedBlocks
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from std_srvs.srv import Empty, Trigger


class Check(Node):
    def __init__(self):
        super().__init__('scene_guard_check', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.scene = self.create_client(GetPlanningScene, '/get_planning_scene')
        self.ready = self.create_client(Trigger, '/classify/scene_ready')
        self.pause = self.create_client(Empty, '/pause_physics')
        self.resume = self.create_client(Empty, '/unpause_physics')
        self.pub = self.create_publisher(DetectedBlocks, '/classify/detections', 10)
        self.results = []

    def spin(self, duration):
        until = time.monotonic() + duration
        while time.monotonic() < until:
            rclpy.spin_once(self, timeout_sec=.05)

    def call(self, client, request):
        assert client.wait_for_service(timeout_sec=10)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10)
        assert future.done() and future.result() is not None, 'Service timed out'
        return future.result()

    def positions(self):
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        scene = self.call(self.scene, request).scene
        return {o.id: [o.pose.position.x+o.primitive_poses[0].position.x,
                       o.pose.position.y+o.primitive_poses[0].position.y,
                       o.pose.position.z+o.primitive_poses[0].position.z]
                for o in scene.world.collision_objects if o.id.endswith('_block')}

    def state(self):
        return self.call(self.ready, Trigger.Request()).success

    def sample(self, positions):
        msg = DetectedBlocks()
        msg.header.frame_id = 'base_link'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.camera_stable = True; msg.status = 'Diagnostic scene guard test'
        for name, p in positions.items():
            b = DetectedBlock(); b.color = name[:-6]
            b.center.x, b.center.y, b.center.z = float(p[0]), float(p[1]), float(p[2]-.347)
            b.top_z = b.center.z+.025; b.confidence = 1.
            b.top_width = b.top_depth = .05; b.plane_error = .0001; b.visible_points = 1000
            msg.blocks.append(b)
        return msg

    def freshen(self, positions):
        for _ in range(4):
            self.pub.publish(self.sample(positions)); self.spin(.25)
        assert self.state(), 'Valid observations did not reacquire scene'

    def same(self, before):
        after = self.positions()
        assert before.keys() == after.keys()
        assert all(np.linalg.norm(np.array(after[n])-p) < .0001 for n, p in before.items()), (before, after)

    def injection(self):
        # The only publisher must be this test node; don't race the real detector.
        self.spin(2)
        assert self.count_publishers('/classify/detections') == 1, 'Stop the detector before injection checks'
        before = self.positions(); assert len(before) == 3
        self.freshen(before)
        jump = copy.deepcopy(before); jump['red_block'][0] -= .02
        self.pub.publish(self.sample(jump)); self.spin(.15)
        self.same(before); assert not self.state()
        self.results.append('one_frame_jump_rejected')
        self.freshen(before)
        stale = self.sample(jump); stale.header.stamp.sec -= 3
        self.pub.publish(stale); self.spin(.15)
        self.same(before); assert self.state()
        self.results.append('expired_stamp_rejected')
        self.freshen(before)
        self.call(self.pause, Empty.Request())
        try:
            stamp = self.get_clock().now().nanoseconds
            self.spin(1.4)
            assert self.get_clock().now().nanoseconds == stamp, 'Simulation clock did not pause'
            self.same(before); assert not self.state(), 'Paused clock hid a detector stream timeout'
            self.results.append('no_stream_with_paused_clock_expires')
        finally:
            self.call(self.resume, Empty.Request())

    def displacement(self):
        self.spin(2); assert self.state()
        before = self.positions()
        original = subprocess.check_output(['gz','model','-m','red_block','-p'], text=True, timeout=10)
        pose = list(map(float, original.split()))
        def set_red(p):
            subprocess.run(['gz','model','-m','red_block','-x',str(p[0]),'-y',str(p[1]),
                            '-z',str(p[2]),'-R',str(p[3]),'-P',str(p[4]),'-Y',str(p[5])],
                           check=True, timeout=10)
        try:
            moved = [pose[0]-.02,pose[1]+.035,pose[2], *pose[3:]]; set_red(moved); self.spin(3)
            after = self.positions(); error = float(np.linalg.norm(np.array(after['red_block'])-moved[:3])*1000)
            assert error < 5 and self.state(), (error, after)
            for n in ['green_block','blue_block']:
                assert np.linalg.norm(np.array(after[n])-before[n]) < .0001
            self.results.append({'real_red_displacement_error_mm': error, 'positions': after})
        finally:
            set_red(pose); self.spin(3)
        after = self.positions()
        assert np.linalg.norm(np.array(after['red_block'])-pose[:3]) < .005
        assert self.state()
        self.results.append({'red_restored': after['red_block']})


def main():
    parser = argparse.ArgumentParser()
    modes = parser.add_mutually_exclusive_group(required=True)
    modes.add_argument('--displacement', action='store_true')
    modes.add_argument('--injection', action='store_true')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    rclpy.init(); check = Check()
    try:
        if args.displacement: check.displacement()
        else: check.injection()
        args.output.write_text(json.dumps(check.results, indent=2))
        print('SCENE_GUARDS_OK', check.results, flush=True)
    finally:
        check.destroy_node(); rclpy.shutdown()


if __name__ == '__main__': main()
