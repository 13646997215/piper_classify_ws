#!/usr/bin/env python3
"""Trigger a batch and independently check Gazebo placement, green, feedback and scene.

Requires a fresh simulation and the four servers. --displace moves only red
before starting to prove that pickup targets are supplied by camera vision.
"""
import argparse
import json
import time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict
from gazebo_msgs.srv import GetEntityState, SetEntityState
from moveit_msgs.srv import GetPlanningScene
from classify_interfaces.msg import TaskStatus, TaskResult
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
from moveit_task_constructor_msgs.msg import TaskDescription, Solution

def main():
    args=argparse.ArgumentParser()
    args.add_argument('--output',type=Path,required=True)
    args.add_argument('--displace',action='store_true')
    args=args.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    rclpy.init();n=Node('verify_classification')
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    statuses=[];results=[];joints=[];descriptions=[];solutions=[]
    subs=[n.create_subscription(TaskStatus,'/classify/status',lambda m:statuses.append(message_to_ordereddict(m)),qos),
          n.create_subscription(TaskResult,'/classify/result',lambda m:results.append(message_to_ordereddict(m)),qos),
          n.create_subscription(JointState,'/joint_states',joints.append,10),
          n.create_subscription(TaskDescription,'/classify_mtc/description',lambda m:descriptions.append(message_to_ordereddict(m)),qos),
          n.create_subscription(Solution,'/classify_mtc/solution',lambda m:solutions.append(m),qos)]
    get=n.create_client(GetEntityState,'/gazebo/get_entity_state')
    set_state=n.create_client(SetEntityState,'/gazebo/set_entity_state')
    start=n.create_client(Trigger,'/classify/start')
    scene=n.create_client(GetPlanningScene,'/get_planning_scene')
    def call(client,req):
        assert client.wait_for_service(timeout_sec=10),client.srv_name
        f=client.call_async(req);rclpy.spin_until_future_complete(n,f,timeout_sec=10)
        assert f.done() and f.result() is not None,client.srv_name
        return f.result()
    def pose(name):
        req=GetEntityState.Request();req.name=name;req.reference_frame='world'
        res=call(get,req);assert res.success,name
        return res.state
    def xyz(state):return np.array([state.pose.position.x,state.pose.position.y,state.pose.position.z])
    failure=None;record={}
    try:
        before={c:pose(c+'_block') for c in ['red','blue','green']}
        if args.displace:
            red=before['red'];red.name='red_block';red.reference_frame='world'
            red.pose.position.x-=.018;red.pose.position.y+=.030
            req=SetEntityState.Request();req.state=red;assert call(set_state,req).success
            end=time.monotonic()+3
            while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
            before['red']=pose('red_block')
        record['before']={k:message_to_ordereddict(v) for k,v in before.items()}
        res=call(start,Trigger.Request());record['start']=message_to_ordereddict(res);assert res.success,res.message
        # The asynchronous service must reject a duplicate while the task owns controllers.
        duplicate=call(start,Trigger.Request());record['duplicate']=message_to_ordereddict(duplicate)
        assert not duplicate.success,'Duplicate start accepted'
        end=time.monotonic()+240
        while not results and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
        assert results,'No result within 240 s'
        assert results[-1]['success'],results[-1]['detail']
        assert results[-1]['succeeded']==2 and results[-1]['failed']==0,results[-1]
        after={c:pose(c+'_block') for c in ['red','blue','green']}
        record['after']={k:message_to_ordereddict(v) for k,v in after.items()}
        for color in ['red','blue']:
            bin_state=pose('bin_'+color)
            inverse=Rotation.from_quat([getattr(bin_state.pose.orientation,k) for k in ['x','y','z','w']]).inv()
            center=inverse.apply(xyz(after[color])-xyz(bin_state))
            rotation=inverse*Rotation.from_quat([getattr(after[color].pose.orientation,k) for k in ['x','y','z','w']])
            corners=rotation.apply(np.array([[x,y,z] for x in [-.025,.025] for y in [-.025,.025] for z in [-.025,.025]]))+center
            assert np.max(np.abs(corners[:,:2]))<.054,color+' outside bin walls'
            assert abs(center[2]-.037)<.006,color+' not resting on bin floor'
        green_error=float(np.linalg.norm(xyz(after['green'])-xyz(before['green']))*1000)
        record['green_displacement_mm']=green_error;assert green_error<2,green_error
        counts={s['completed'] for s in statuses};assert {0,1,2}.issubset(counts),counts
        ratios=[s['gripper_open_ratio'] for s in statuses];assert max(ratios)>.95 and min(ratios)<.7
        held={s['held_object'] for s in statuses}
        assert {'red_block','blue_block'}.issubset(held) and 'green_block' not in held,held
        assert statuses[-1]['held_object']=='','Physical constraint still holds an object'
        assert joints,'No joint feedback'
        q=dict(zip(joints[-1].name,joints[-1].position))
        goal=[0.,.87266,-1.22173,0.,1.22,0.]
        assert max(abs(q[f'joint{i+1}']-v) for i,v in enumerate(goal))<.065,'Home not restored'
        req=GetPlanningScene.Request();req.components.components=1023
        ps=call(scene,req).scene;record['scene']=message_to_ordereddict(ps)
        assert not ps.robot_state.attached_collision_objects,'Object still attached after placement'
        scene_errors={}
        for obj in ps.world.collision_objects:
            if obj.id not in ['red_block','blue_block','green_block']:continue
            origin=obj.pose.position;local=obj.primitive_poses[0].position
            rotation=Rotation.from_quat([getattr(obj.pose.orientation,k) for k in ['x','y','z','w']])
            point=np.array([origin.x,origin.y,origin.z])+rotation.apply([local.x,local.y,local.z])
            assert obj.header.frame_id in ['world','base_link'],obj.header.frame_id
            if obj.header.frame_id=='base_link':point[2]+=.347
            error=float(np.linalg.norm(point-xyz(after[obj.id[:-6]]))*1000)
            scene_errors[obj.id]=error
        record['scene_errors_mm']=scene_errors
        assert len(scene_errors)==3 and max(scene_errors.values())<5,scene_errors
        assert len({m['task_id'] for m in descriptions if m['stages']})>=2,'Missing MTC task descriptions'
        assert len({m.task_id for m in solutions})>=2,'Missing published MTC solutions'
        record['mtc_descriptions']=[m for m in descriptions if m['stages']]
        record['mtc_solutions']=[{'task_id':m.task_id,'trajectories':len(m.sub_trajectory)} for m in solutions]
        record['passed']=True
        print('VERIFY_TASK_OK: red/blue inside bins; green displacement %.3f mm; feedback 0/2 -> 1/2 -> 2/2; home restored'%green_error,flush=True)
    except Exception as e:
        failure=str(e);record['passed']=False;record['failure']=failure
        print('VERIFY_TASK_FAILED:',failure,flush=True)
    finally:
        record['statuses']=statuses;record['results']=results
        (args.output/'record.json').write_text(json.dumps(record,indent=2))
        n.destroy_node();rclpy.shutdown()
    if failure:raise SystemExit(1)

if __name__=='__main__':main()
