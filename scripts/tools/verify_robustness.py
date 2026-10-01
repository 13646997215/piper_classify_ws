#!/usr/bin/env python3
"""Run complete headless trials, visibility guards, cancellation and reset checks."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET

import cv2
from cv_bridge import CvBridge
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from rosidl_runtime_py.convert import message_to_ordereddict
from classify_interfaces.msg import TaskStatus,TaskResult,SceneStatus
from gazebo_msgs.srv import GetEntityState,SpawnEntity,DeleteEntity
from sensor_msgs.msg import Image,JointState
from action_msgs.msg import GoalStatusArray
from std_srvs.srv import Trigger
from moveit_msgs.srv import GetPlanningScene

ROOT=Path(__file__).resolve().parents[2]


class Check(Node):
    def __init__(self):
        super().__init__('verify_robustness')
        self.task=None;self.scene=None;self.result=None;self.joints=None;self.rgb=None;self.goals=[]
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.subs=[
            self.create_subscription(TaskStatus,'/classify/status',lambda m:setattr(self,'task',m),qos),
            self.create_subscription(TaskResult,'/classify/result',lambda m:setattr(self,'result',m),qos),
            self.create_subscription(SceneStatus,'/classify/scene_status',lambda m:setattr(self,'scene',m),qos),
            self.create_subscription(JointState,'/joint_states',lambda m:setattr(self,'joints',m),10),
            self.create_subscription(Image,'/camera/color/image_raw',lambda m:setattr(self,'rgb',m),qos_profile_sensor_data),
            self.create_subscription(GoalStatusArray,'/piper_arm_controller/follow_joint_trajectory/_action/status',self.goals.append,qos),
        ]
        self.service_clients={name:self.create_client(kind,name) for name,kind in [
            ('/gazebo/get_entity_state',GetEntityState),('/spawn_entity',SpawnEntity),
            ('/delete_entity',DeleteEntity),('/classify/start',Trigger),('/classify/cancel',Trigger),
            ('/get_planning_scene',GetPlanningScene)]}

    def call(self,name,req):
        client=self.service_clients[name];assert client.wait_for_service(timeout_sec=10),name
        future=client.call_async(req);rclpy.spin_until_future_complete(self,future,timeout_sec=10)
        assert future.done() and future.result() is not None,name
        return future.result()

    def wait(self,predicate,timeout=45):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            rclpy.spin_once(self,timeout_sec=.05)
            if predicate():return
        raise AssertionError('Timed out waiting for trial condition')

    def spin(self,seconds):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:rclpy.spin_once(self,timeout_sec=.05)

    def positions(self):
        result={}
        for color in ['red','green','blue']:
            req=GetEntityState.Request();req.name=color+'_block';req.reference_frame='world'
            response=self.call('/gazebo/get_entity_state',req);assert response.success
            result[color]=message_to_ordereddict(response.state)
        return result

    def ready(self,world):
        self.wait(lambda:self.task and self.scene and self.joints and self.scene.ready)
        assert self.task.state=='IDLE' and self.task.completed==0 and not self.task.placed_objects
        assert self.result is None,'Stale result after reset'
        positions=self.positions();root=ET.parse(world).getroot().find('world')
        for color in ['red','green','blue']:
            p=positions[color]['pose']['position'];expected=list(map(float,root.find(f"model[@name='{color}_block']/pose").text.split()))[:3]
            assert max(abs(a-b) for a,b in zip([p['x'],p['y'],p['z']],expected))<.002
        req=GetPlanningScene.Request();req.components.components=1023
        scene=self.call('/get_planning_scene',req).scene
        assert not scene.robot_state.attached_collision_objects
        ids={o.id for o in scene.world.collision_objects}
        obstacle_errors={}
        for model in root.findall('model'):
            if model.get('name').startswith('obstacle_'):
                name=model.get('name');assert name in ids
                item=next(o for o in scene.world.collision_objects if o.id==name)
                req=GetEntityState.Request();req.name=name;req.reference_frame='world'
                truth=self.call('/gazebo/get_entity_state',req);assert truth.success
                assert item.header.frame_id=='world' and len(item.primitives)==1
                # MoveIt may put the world transform in object.pose and keep
                # primitive_poses relative. Compose both, as for cube checks.
                origin=item.pose.position;local=item.primitive_poses[0].position;t=truth.state.pose.position
                rotation=Rotation.from_quat([getattr(item.pose.orientation,k) for k in ['x','y','z','w']])
                center=np.array([origin.x,origin.y,origin.z])+rotation.apply([local.x,local.y,local.z])
                error=float(np.linalg.norm(center-[t.x,t.y,t.z]));assert error<1e-6
                obstacle_errors[name]=error
        return {'initial_positions':positions,'task':message_to_ordereddict(self.task),
                'scene':message_to_ordereddict(self.scene),'collision_ids':sorted(ids),
                'obstacle_position_errors_m':obstacle_errors}

    def visibility_cancel(self,directory,timeout_failure=False):
        # An actual Gazebo grey screen physically hides green's top, no fake detections.
        req=SpawnEntity.Request();req.name='test_camera_occluder';req.reference_frame='world'
        req.initial_pose.orientation.w=1.
        req.xml='''<sdf version="1.6"><model name="test_camera_occluder"><static>true</static>
        <pose>.37 .14 .43 0 0 0</pose><link name="screen"><collision name="c"><geometry><box>
        <size>.14 .12 .01</size></box></geometry></collision><visual name="v"><geometry><box>
        <size>.14 .12 .01</size></box></geometry><material><ambient>.4 .4 .4 1</ambient>
        <diffuse>.4 .4 .4 1</diffuse></material></visual></link></model></sdf>'''
        assert self.call('/spawn_entity',req).success
        self.wait(lambda:self.scene and not self.scene.ready and 'green' not in self.scene.fresh_colors,10)
        self.spin(1)
        if self.rgb:cv2.imwrite(str(directory/'occluded_rgb.png'),CvBridge().imgmsg_to_cv2(self.rgb,'bgr8'))
        before_goals=sum(len(m.status_list) for m in self.goals)
        before=dict(zip(self.joints.name,self.joints.position))
        assert self.call('/classify/start',Trigger.Request()).success
        self.wait(lambda:self.task and self.task.state=='DETECTING')
        self.spin(2)
        assert sum(len(m.status_list) for m in self.goals)==before_goals,'Arm goal sent without green visibility'
        after=dict(zip(self.joints.name,self.joints.position))
        assert max(abs(after[k]-v) for k,v in before.items())<.005
        if timeout_failure:
            self.wait(lambda:self.result is not None,30)
            assert self.task.state=='FAILED' and not self.result.success and self.result.failed==1
        else:
            assert self.call('/classify/cancel',Trigger.Request()).success
            self.wait(lambda:self.result is not None,10)
            assert self.task.state=='CANCELLED' and not self.result.success and self.result.failed==0
        assert sum(len(m.status_list) for m in self.goals)==before_goals,'Arm goal sent before visibility recovered'
        assert not self.call('/classify/start',Trigger.Request()).success,'Cancelled session restarted without reset'
        delete=DeleteEntity.Request();delete.name=req.name;assert self.call('/delete_entity',delete).success
        # Terminal cancellation deliberately keeps red owned/frozen until reset.
        # The unowned green reference must recover when the screen is removed.
        self.wait(lambda:self.scene.camera_stable and 'green' in self.scene.fresh_colors,10)
        return {'reference_reacquired':message_to_ordereddict(self.scene),'result':message_to_ordereddict(self.result),
                'no_motion_while_occluded':True,'restart_requires_reset':True}

    def held_cancel(self):
        assert self.call('/classify/start',Trigger.Request()).success
        self.wait(lambda:self.task and self.task.stage=='lift object' and self.task.held_object=='red_block',65)
        start=time.monotonic();response=self.call('/classify/cancel',Trigger.Request());latency=time.monotonic()-start
        assert response.success and latency<2.,latency
        self.wait(lambda:self.result is not None,10)
        assert self.task.state=='CANCELLED' and self.task.held_object=='red_block'
        assert self.result.failed==0 and self.result.succeeded==0 and not self.result.success
        assert not self.call('/classify/start',Trigger.Request()).success
        self.spin(1);before=dict(zip(self.joints.name,self.joints.position));self.spin(2)
        after=dict(zip(self.joints.name,self.joints.position));error=max(abs(after[k]-v) for k,v in before.items())
        assert error<.01,error
        assert self.task.held_object=='red_block' and self.task.gripper_open_ratio<.9
        return {'cancel_response_sec':latency,'stopped_joint_max_change':error,
                'status':message_to_ordereddict(self.task),'result':message_to_ordereddict(self.result),
                'restart_requires_reset':True}


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--cases',nargs='+',choices=['baseline_a','baseline_b','clutter_stress','occlusion_cancel','vision_timeout_fail','held_cancel','reset_after_cancel'])
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    args.output=args.output.resolve();baseline=ROOT/'src/description/classify_description/worlds/classify.world'
    clutter=ROOT/'docs/robustness_20261002/clutter_seed7.world'
    cases=[('baseline_a',baseline,[]),('baseline_b',baseline,[]),
           ('clutter_stress',clutter,['--sensor-stress','--seed','7']),
           ('occlusion_cancel',baseline,[]),('vision_timeout_fail',baseline,[]),
           ('held_cancel',baseline,[]),('reset_after_cancel',baseline,[])]
    if args.cases:cases=[case for case in cases if case[0] in args.cases]
    if subprocess.run(['python3',str(ROOT/'scripts/tools/classify_lifecycle.py'),'--check']).returncode:
        raise SystemExit('Stop the current workspace simulation before this suite')
    summary=[];rclpy.init()
    try:
        for name,world,options in cases:
            directory=args.output/name;directory.mkdir();print('TRIAL_START:',name,flush=True)
            check=None
            with (directory/'launch.log').open('w') as log:
                process=subprocess.Popen(['bash',str(ROOT/'scripts/run_classify.sh'),'--headless','--world',str(world),*options],cwd=ROOT,stdout=log,stderr=subprocess.STDOUT)
                try:
                    check=Check();initial=check.ready(world)
                    (directory/'initial.json').write_text(json.dumps(initial,indent=2))
                    if name=='occlusion_cancel':result=check.visibility_cancel(directory)
                    elif name=='vision_timeout_fail':result=check.visibility_cancel(directory,timeout_failure=True)
                    elif name=='held_cancel':result=check.held_cancel()
                    elif name=='reset_after_cancel':result={'reset_restored_initial':True}
                    else:
                        if name=='baseline_b':
                            subprocess.run(['python3',str(ROOT/'scripts/tools/capture_camera_reference.py'),
                                '--source','simulation','--duration','4','--output',str(directory/'camera_reference')],cwd=ROOT,check=True)
                        check.destroy_node();check=None
                        subprocess.run(['python3',str(ROOT/'scripts/tools/verify_task.py'),'--output',str(directory/'batch')],cwd=ROOT,check=True)
                        a=json.loads((directory/'batch/record.json').read_text())
                        result={'green_displacement_mm':a['green_displacement_mm'],'elapsed_sec':a['results'][-1]['elapsed_sec']}
                    summary.append({'case':name,'passed':True,**result});print('TRIAL_OK:',name,flush=True)
                except Exception as error:
                    summary.append({'case':name,'passed':False,'error':str(error)});raise
                finally:
                    if check:check.destroy_node()
                    subprocess.run(['bash',str(ROOT/'scripts/stop_classify.sh')],cwd=ROOT,check=True)
                    process.wait(timeout=20)
                    (args.output/'summary.json').write_text(json.dumps(summary,indent=2))
    finally:rclpy.shutdown()
    print('ROBUSTNESS_SUITE_OK:',len(summary),'trials',flush=True)


if __name__=='__main__':main()
