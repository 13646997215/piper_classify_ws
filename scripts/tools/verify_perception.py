#!/usr/bin/env python3
"""Simulation-only RGB-D/TF evidence recorder and controlled motion regression.

Python is used as test tooling; the detector and scene bridge remain C++ nodes.
Run with a running Gazebo, MoveIt and detector in ROS_DOMAIN_ID=42.
Every interpolated arm state is checked against MoveIt's collision scene before
the test sends a trajectory. The original arm pose is restored in finally.
"""
import argparse
import collections
import json
from pathlib import Path
import subprocess
import time

import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from classify_interfaces.msg import DetectedBlocks, SceneStatus
from std_srvs.srv import Trigger
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.srv import GetPlanningScene, GetStateValidity
from moveit_msgs.msg import PlanningSceneComponents
from sensor_msgs.msg import CameraInfo, Image, JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from tf2_ros import Buffer, TransformListener
from scipy.spatial.transform import Rotation


def stamp_ns(header):
    return header.stamp.sec * 1000000000 + header.stamp.nanosec


class Probe(rclpy.node.Node):
    def __init__(self, output):
        super().__init__('perception_motion_probe', parameter_overrides=[
            rclpy.parameter.Parameter('use_sim_time', value=True)])
        self.output = output
        self.output.mkdir(parents=True, exist_ok=False)
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self)
        self.rgb = collections.OrderedDict()
        self.depth = collections.OrderedDict()
        self.info = None
        self.joints = None
        self.phase = 'startup'
        self.samples = []
        self.detections = []
        self.statuses = []
        self.rgb_depth_deltas = []
        self.last_capture = 0
        self.seen_stamp = 0
        self.subs = [
            self.create_subscription(Image, '/camera/color/image_raw', self.on_rgb, qos_profile_sensor_data),
            self.create_subscription(Image, '/camera/depth/image_raw', self.on_depth, qos_profile_sensor_data),
            self.create_subscription(CameraInfo, '/camera/color/camera_info', self.on_info, qos_profile_sensor_data),
            self.create_subscription(JointState, '/joint_states', self.on_joints, 10),
            self.create_subscription(DetectedBlocks, '/classify/detections', self.on_detection, 10),
            self.create_subscription(SceneStatus, '/classify/scene_status', self.on_status, 10),
        ]
        self.timer = self.create_timer(.1, self.capture)
        self.validity = self.create_client(GetStateValidity, '/check_state_validity')
        self.scene = self.create_client(GetPlanningScene, '/get_planning_scene')
        self.ready = self.create_client(Trigger, '/classify/scene_ready')
        self.arm = ActionClient(self, FollowJointTrajectory,
                                '/piper_arm_controller/follow_joint_trajectory')
        self.names = [f'joint{i}' for i in range(1, 7)]

    def on_rgb(self, msg):
        t = stamp_ns(msg.header)
        self.rgb[t] = msg
        if self.depth:
            delta = (t - next(reversed(self.depth))) / 1e6
            self.rgb_depth_deltas.append({'stamp': t, 'delta_ms': delta, 'phase': self.phase})
        while len(self.rgb) > 30:
            self.rgb.popitem(last=False)

    def on_depth(self, msg):
        self.depth[stamp_ns(msg.header)] = msg
        while len(self.depth) > 30:
            self.depth.popitem(last=False)

    def on_info(self, msg):
        self.info = msg

    def on_joints(self, msg):
        self.joints = msg

    def on_detection(self, msg):
        self.detections.append({
            'stamp': stamp_ns(msg.header), 'phase': self.phase,
            'camera_stable': msg.camera_stable, 'status': msg.status,
            'positions': {b.color: [b.center.x, b.center.y, b.center.z] for b in msg.blocks}})

    def on_status(self, msg):
        self.statuses.append({'stamp': stamp_ns(msg.header), 'phase': self.phase,
                              'ready': msg.ready, 'camera_stable': msg.camera_stable,
                              'fresh': list(msg.fresh_colors), 'stale': list(msg.stale_colors)})

    def capture(self):
        common = self.rgb.keys() & self.depth.keys()
        if not common or self.info is None:
            return
        stamp = max(common)
        if stamp <= self.seen_stamp or stamp - self.last_capture < 300000000:
            return
        try:
            transform = self.buffer.lookup_transform('base_link', self.rgb[stamp].header.frame_id, Time(nanoseconds=stamp))
        except Exception:
            return
        rgb, depth = self.rgb[stamp], self.depth[stamp]
        if rgb.encoding != 'rgb8' or depth.encoding not in ('32FC1','16UC1'):
            raise RuntimeError(f'Unexpected sensor encodings: {rgb.encoding}, {depth.encoding}')
        image = np.frombuffer(rgb.data, np.uint8).reshape(rgb.height, rgb.step)[:, :rgb.width*3].reshape(rgb.height, rgb.width, 3)
        if depth.encoding == '32FC1':
            distances = np.frombuffer(depth.data, np.float32).reshape(depth.height, depth.step // 4)[:, :depth.width]
        else:
            distances = np.frombuffer(depth.data, np.uint16).reshape(depth.height, depth.step // 2)[:, :depth.width].astype(np.float32) / 1000.
            distances[distances == 0] = np.nan
        t, q = transform.transform.translation, transform.transform.rotation
        matrix = np.eye(4)
        matrix[:3, :3] = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        matrix[:3, 3] = [t.x, t.y, t.z]
        name = f'frame_{len(self.samples):04d}.npz'
        np.savez_compressed(self.output / name, rgb=image, depth=distances,
                            k=np.array(self.info.k).reshape(3, 3), camera_to_base=matrix,
                            stamp=np.int64(stamp))
        self.samples.append({'file': name, 'stamp': stamp, 'phase': self.phase,
                             'joints': self.current_positions()})
        self.seen_stamp = stamp
        self.last_capture = stamp

    def current_positions(self):
        if self.joints is None:
            return []
        lookup = dict(zip(self.joints.name, self.joints.position))
        return [lookup[n] for n in self.names]

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=.1)

    def wait_future(self, future, seconds=15):
        rclpy.spin_until_future_complete(self, future, timeout_sec=seconds)
        if not future.done() or future.result() is None:
            raise RuntimeError('ROS operation timed out')
        return future.result()

    def move(self, target, phase):
        start = self.current_positions()
        for ratio in np.linspace(0, 1, 25):
            req = GetStateValidity.Request()
            req.group_name = 'arm'
            req.robot_state.joint_state.name = list(self.joints.name)
            req.robot_state.joint_state.position = list(self.joints.position)
            target_map = dict(zip(self.names, (np.array(start)*(1-ratio)+np.array(target)*ratio).tolist()))
            req.robot_state.joint_state.position = [target_map.get(n, p) for n, p in zip(self.joints.name, self.joints.position)]
            result = self.wait_future(self.validity.call_async(req))
            if not result.valid:
                contacts = [(c.contact_body_1, c.contact_body_2) for c in result.contacts]
                raise RuntimeError(f'Unsafe interpolated state for {phase}: {contacts}')
        self.phase = phase + '_moving'
        print(self.phase, flush=True)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.names
        for values, seconds in [(start, .2), (target, 4.2)]:
            p = JointTrajectoryPoint()
            p.positions = [float(v) for v in values]
            p.velocities = [0.] * 6
            p.time_from_start = Duration(seconds=seconds).to_msg()
            goal.trajectory.points.append(p)
        handle = self.wait_future(self.arm.send_goal_async(goal))
        if not handle.accepted:
            raise RuntimeError('Arm controller rejected test goal')
        result = self.wait_future(handle.get_result_async(), 30)
        if result.status != 4 or result.result.error_code != 0:
            raise RuntimeError(f'Trajectory failed: {result.status} {result.result.error_string}')
        self.phase = phase + '_settled'
        self.spin_for(3)
        scene_req = GetPlanningScene.Request()
        scene_req.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        scene = self.wait_future(self.scene.call_async(scene_req)).scene
        positions = {}
        for obj in scene.world.collision_objects:
            if obj.id.endswith('_block'):
                a = obj.pose.position
                p = obj.primitive_poses[0].position
                positions[obj.id] = [a.x+p.x, a.y+p.y, a.z+p.z]
        print(self.phase, 'joints', [round(v,4) for v in self.current_positions()],
              'scene', positions, flush=True)
        ready = self.wait_future(self.ready.call_async(Trigger.Request())) if self.ready.service_is_ready() else None
        self.scene_snapshots.append({'phase': self.phase, 'positions': positions,
                                    'ready': ready.success if ready else None})
        if ready: print('scene_ready',ready.success,ready.message,flush=True)

    def save(self, truth, failure):
        data = {'samples': self.samples, 'detections': self.detections,
                'rgb_depth_deltas': self.rgb_depth_deltas, 'scene_snapshots': self.scene_snapshots,
                'statuses': self.statuses, 'truth_world': truth, 'failure': failure}
        (self.output / 'record.json').write_text(json.dumps(data, indent=2))

    def check(self, truth):
        errors=[]
        for row in self.detections:
            for name,p in row['positions'].items():
                target=np.array(truth[name][:3]); target[2]-=.347
                errors.append(float(np.linalg.norm(np.array(p)-target)*1000))
        scene_errors=[float(np.linalg.norm(np.array(p)-np.array(truth[name[:-6]][:3]))*1000)
                      for row in self.scene_snapshots for name,p in row['positions'].items()]
        assert errors and max(errors)<5, f'Detection error mm: {max(errors) if errors else "no observations"}'
        assert scene_errors and max(scene_errors)<5, f'Scene error mm: {max(scene_errors) if scene_errors else "no objects"}'
        assert all(row['ready'] for row in self.scene_snapshots if 'home_' in row['phase']), 'Home did not reacquire all blocks'
        assert any(not s['ready'] for s in self.statuses if s['phase']=='out_of_view_settled'), 'Invisible blocks reported ready'
        assert any(not s['camera_stable'] for s in self.statuses if s['phase'].endswith('_moving')), 'Motion was not flagged'
        print('VERIFY_OK max detection mm',round(max(errors),3),'max scene mm',round(max(scene_errors),3),flush=True)


def model_truth():
    result = {}
    for color in ['red', 'green', 'blue']:
        output = subprocess.check_output(['gz', 'model', '-m', color+'_block', '-p'], text=True, timeout=10)
        result[color] = list(map(float, output.split()))
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--check', action='store_true', help='Assert fixed-detector and scene freshness behavior')
    args = parser.parse_args()
    rclpy.init()
    probe = Probe(args.output)
    probe.scene_snapshots = []
    failure = None
    home = None
    truth = {}
    try:
        truth = model_truth()
        assert probe.validity.wait_for_service(timeout_sec=10)
        assert probe.scene.wait_for_service(timeout_sec=10)
        assert probe.arm.wait_for_server(timeout_sec=10)
        probe.spin_for(3)
        home = probe.current_positions()
        assert len(home) == 6
        probe.phase = 'home_settled'; probe.spin_for(3)
        poses = []
        for label, joint, delta in [('yaw_left', 0, .16), ('yaw_right', 0, -.16),
                                    ('wrist_tilt', 4, -.18), ('out_of_view', 0, .75)]:
            target = list(home); target[joint] += delta
            poses.append((label, target))
        for label, target in poses:
            probe.move(target, label)
            probe.move(home, 'home_after_'+label)
        if args.check: probe.check(truth)
    except BaseException as exc:
        failure = repr(exc)
        print('VERIFY_FAILURE', failure, flush=True)
        raise
    finally:
        if home and np.linalg.norm(np.array(probe.current_positions())-home) > .01:
            try:
                probe.move(home, 'restore')
            except Exception as exc:
                failure = str(failure) + '; restore: ' + repr(exc)
                print('RESTORE_FAILURE', repr(exc), flush=True)
        probe.save(truth, failure)
        print('Saved', len(probe.samples), 'RGB-D/TF pairs to', probe.output, flush=True)
        probe.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
