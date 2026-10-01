#!/usr/bin/env python3
"""Independent vision-ID/yaw and complete physical multi-cube batch acceptance."""
import argparse
import importlib.util
import json
from pathlib import Path
import time
import xml.etree.ElementTree as ET
import numpy as np
import cv2
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rosidl_runtime_py.convert import message_to_ordereddict as msgdict
from gazebo_msgs.srv import GetEntityState, SpawnEntity, DeleteEntity
from moveit_msgs.srv import GetPlanningScene
from classify_interfaces.msg import TaskStatus, TaskResult, SceneStatus, DetectedBlocks
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState, Image
from tf2_ros import Buffer, TransformListener

ROOT=Path(__file__).resolve().parents[2]

def matrix(p):
    out=np.eye(4)
    out[:3,:3]=Rotation.from_quat([getattr(p.orientation,k) for k in ['x','y','z','w']]).as_matrix()
    out[:3,3]=[p.position.x,p.position.y,p.position.z]
    return out

def corners(p):
    return np.array([[x,y,z,1] for x in [-.025,.025] for y in [-.025,.025] for z in [-.025,.025]])@p.T

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--world',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--occlude-before-start',action='store_true')
    parser.add_argument('--expect-capacity-failure',action='store_true')
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    tree=ET.parse(args.world);models=tree.getroot().find('world')
    targets=[m.get('name') for m in models.findall('model') if m.get('name').startswith(('red_block','blue_block'))]
    count=len(targets);n=None;record={'world':str(args.world.resolve()),'target_count':count};failure=None
    rclpy.init();n=Node('verify_multi_classification',parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)])
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    statuses=[];results=[];scenes=[];detections=[];joints=[];images={}
    subs=[n.create_subscription(TaskStatus,'/classify/status',lambda m:statuses.append(msgdict(m)),qos),
          n.create_subscription(TaskResult,'/classify/result',lambda m:results.append(msgdict(m)),qos),
          n.create_subscription(SceneStatus,'/classify/scene_status',scenes.append,qos),
          n.create_subscription(DetectedBlocks,'/classify/detections',detections.append,10),
          n.create_subscription(JointState,'/joint_states',joints.append,10),
          n.create_subscription(Image,'/camera/color/image_raw',lambda m:images.update(rgb=m),10)]
    tf=Buffer(node=n);listener=TransformListener(tf,n)
    entity=n.create_client(GetEntityState,'/gazebo/get_entity_state')
    scene=n.create_client(GetPlanningScene,'/get_planning_scene');start=n.create_client(Trigger,'/classify/start')
    def call(client,req):
        assert client.wait_for_service(timeout_sec=10),client.srv_name
        f=client.call_async(req);rclpy.spin_until_future_complete(n,f,timeout_sec=10)
        assert f.done() and f.result(),client.srv_name
        return f.result()
    def physical(name):
        req=GetEntityState.Request();req.name=name;req.reference_frame='world'
        res=call(entity,req);assert res.success,name
        return matrix(res.state.pose)
    def planning():
        req=GetPlanningScene.Request();req.components.components=1023
        return call(scene,req).scene
    def world_objects(ps,base):
        out={}
        for obj in ps.world.collision_objects:
            origin=matrix(obj.pose);local=matrix(obj.primitive_poses[0])
            assert obj.header.frame_id in ['world','base_link'],obj.header.frame_id
            out[obj.id]=(base if obj.header.frame_id=='base_link' else np.eye(4))@origin@local
        return out
    try:
        end=time.monotonic()+40
        while time.monotonic()<end:
            rclpy.spin_once(n,timeout_sec=.1)
            if scenes and scenes[-1].ready and len(scenes[-1].fresh_objects)==count+1:break
        assert scenes and scenes[-1].ready,'Full inventory never became fresh'
        record['initial_scene_status']=msgdict(scenes[-1])
        before={name:physical(name) for name in targets+['green_block']}
        tr=tf.lookup_transform('world','base_link',rclpy.time.Time()).transform
        base=np.eye(4);base[:3,:3]=Rotation.from_quat([getattr(tr.rotation,k) for k in ['x','y','z','w']]).as_matrix()
        base[:3,3]=[tr.translation.x,tr.translation.y,tr.translation.z]
        initial=world_objects(planning(),base);mapping={};errors={};yaw_errors={}
        colours=dict(zip(scenes[-1].object_ids,scenes[-1].object_colors))
        for id in scenes[-1].fresh_objects:
            c=colours[id];candidates={name:p for name,p in before.items() if name.startswith(c+'_block')}
            name=min(candidates,key=lambda key:np.linalg.norm(initial[id][:3,3]-candidates[key][:3,3]))
            assert name not in mapping.values(),'Two visual identities describe one physical cube'
            mapping[id]=name;errors[id]=float(np.linalg.norm(initial[id][:3,3]-before[name][:3,3])*1000)
            ya=np.arctan2(initial[id][1,0],initial[id][0,0]);yb=np.arctan2(before[name][1,0],before[name][0,0])
            yaw_errors[id]=float(abs((ya-yb+np.pi/4)%(np.pi/2)-np.pi/4)*180/np.pi)
        record.update(initial_visual_to_physical=mapping,initial_position_errors_mm=errors,initial_yaw_errors_degrees=yaw_errors)
        assert max(errors.values())<5,errors
        assert max(yaw_errors.values())<3,yaw_errors
        if images:cv2.imwrite(str(args.output/'initial_camera.png'),CvBridge().imgmsg_to_cv2(images['rgb'],'bgr8'))
        if args.occlude_before_start:
            # A small physical grey screen centred on the viewing ray of one
            # red instance, above the table and away from the other cubes.
            visual_id=next(id for id,name in mapping.items() if name.startswith('red') and before[name][0,3]>.45)
            camera=tf.lookup_transform('world','camera_color_optical_frame',rclpy.time.Time()).transform.translation
            origin=np.array([camera.x,camera.y,camera.z]);top=before[mapping[visual_id]][:3,3]+[0,0,.025]
            normal=(top-origin)/np.linalg.norm(top-origin);axis=np.cross([0,0,1],normal);axis/=np.linalg.norm(axis)
            quat=Rotation.from_matrix(np.column_stack([axis,np.cross(normal,axis),normal])).as_quat()
            request=SpawnEntity.Request();request.name='multi_camera_occluder';request.reference_frame='world'
            center=origin+.8*(top-origin)
            request.initial_pose.position.x,request.initial_pose.position.y,request.initial_pose.position.z=map(float,center)
            request.initial_pose.orientation.x,request.initial_pose.orientation.y,request.initial_pose.orientation.z,request.initial_pose.orientation.w=map(float,quat)
            request.xml='<sdf version="1.6"><model name="multi_camera_occluder"><static>true</static><link name="screen"><visual name="v"><geometry><box><size>.065 .065 .002</size></box></geometry><material><ambient>.4 .4 .4 1</ambient><diffuse>.4 .4 .4 1</diffuse></material></visual></link></model></sdf>'
            spawn=n.create_client(SpawnEntity,'/spawn_entity');delete=n.create_client(DeleteEntity,'/delete_entity')
            assert call(spawn,request).success
            end=time.monotonic()+10
            while time.monotonic()<end:
                rclpy.spin_once(n,timeout_sec=.1)
                if visual_id in scenes[-1].stale_objects and not scenes[-1].ready:break
            assert visual_id in scenes[-1].stale_objects and not scenes[-1].ready,'Occluded ID stayed fresh'
            assert any(id!=visual_id and colours[id]=='red' for id in scenes[-1].fresh_objects),'Other same-colour object also lost'
            record['occluded_scene_status']=msgdict(scenes[-1])
            if images:cv2.imwrite(str(args.output/'occluded_camera.png'),CvBridge().imgmsg_to_cv2(images['rgb'],'bgr8'))
            cached=world_objects(planning(),base);assert np.linalg.norm(cached[visual_id][:3,3]-initial[visual_id][:3,3])<.001
            request=DeleteEntity.Request();request.name='multi_camera_occluder';assert call(delete,request).success
            end=time.monotonic()+15
            while time.monotonic()<end:
                rclpy.spin_once(n,timeout_sec=.1)
                if scenes[-1].ready:break
            assert scenes[-1].ready and set(scenes[-1].object_ids)==set(mapping),'Reacquisition changed visual IDs'
            record['reacquired_scene_status']=msgdict(scenes[-1])
        assert statuses and statuses[-1]['state']=='IDLE','Verifier requires a fresh task server'
        # Ignore transient results cached from a previous DDS publisher during restart.
        results.clear();statuses.clear()
        initial_joints=dict(zip(joints[-1].name,joints[-1].position))
        response=call(start,Trigger.Request());record['start']=msgdict(response);assert response.success,response.message
        duplicate=call(start,Trigger.Request());record['duplicate']=msgdict(duplicate);assert not duplicate.success
        end=time.monotonic()+360
        while not results and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
        assert results,'No result within 360 s'
        result=results[-1]
        if args.expect_capacity_failure:
            assert not result['success'] and result['succeeded']==0 and result['failed']==1,result
            assert 'capacity insufficient' in result['detail'],result['detail']
            assert not any(s['state'] in ['PICKING','PLACING'] for s in statuses)
            assert max(abs(dict(zip(joints[-1].name,joints[-1].position))[k]-v) for k,v in initial_joints.items())<.001
            for name in before:assert np.linalg.norm(physical(name)[:3,3]-before[name][:3,3])<.001
            assert not call(start,Trigger.Request()).success,'Capacity failure allowed retry without reset'
            record['passed']=True;print('VERIFY_CAPACITY_OK: rejection before motion; retry locked',flush=True);return
        assert result['success'],result['detail']
        assert result['succeeded']==count and result['failed']==0,result
        assert len(set(result['objects']))==count and set(result['physical_objects'])==set(targets),result
        after={name:physical(name) for name in targets+['green_block']}
        green_error=float(np.linalg.norm(after['green_block'][:3,3]-before['green_block'][:3,3])*1000)
        record['green_displacement_mm']=green_error;assert green_error<2,green_error
        bin_local={};cavity={}
        for c in ['red','blue']:
            link=models.find(f"model[@name='bin_{c}']/link")
            floor=np.fromstring(link.findtext("collision[@name='collision_floor']/geometry/box/size"),sep=' ')
            wall=np.fromstring(link.findtext("collision[@name='collision_south']/geometry/box/size"),sep=' ')
            cavity[c]=floor[:2]/2-wall[1]
            bins=physical('bin_'+c)
            for name in targets:
                if not name.startswith(c+'_block'):continue
                local=np.linalg.inv(bins)@after[name];pts=corners(local)
                assert np.all(np.abs(pts[:,:2])<cavity[c]+.001),name+' crosses walls'
                assert np.min(pts[:,2])>.010 and abs(local[2,3]-.037)<.006,name+' not on bin floor'
                bin_local[name]=local
            names=[name for name in targets if name.startswith(c+'_block')]
            for i,a in enumerate(names):
                for b in names[:i]:
                    pa,pb=corners(bin_local[a]),corners(bin_local[b])
                    # Conservative AABB non-overlap for the tested flat releases.
                    assert any(pa[:,k].max()+.002<pb[:,k].min() or pb[:,k].max()+.002<pa[:,k].min() for k in [0,1]),'Cubes overlap: '+a+','+b
        record['final_bin_local_poses']={k:v.tolist() for k,v in bin_local.items()}
        ps=planning();assert not ps.robot_state.attached_collision_objects
        final=world_objects(ps,base);scene_errors={}
        for id,name in zip(result['objects'],result['physical_objects']):
            assert mapping[id]==name,'Visual ID switched during task: '+id
            scene_errors[id]=float(np.linalg.norm(final[id][:3,3]-after[name][:3,3])*1000)
        assert max(scene_errors.values())<5,scene_errors;record['final_scene_errors_mm']=scene_errors
        assert set(range(count+1)).issubset({s['completed'] for s in statuses})
        held={s['held_object'] for s in statuses};assert set(targets).issubset(held) and 'green_block' not in held
        assert statuses[-1]['held_object']==''
        assert joints,'No feedback';q=dict(zip(joints[-1].name,joints[-1].position))
        assert max(abs(q[f'joint{i+1}']-v) for i,v in enumerate([0,.87266,-1.22173,0,1.22,0]))<.065,'HOME not restored'
        record['passed']=True;print(f'VERIFY_MULTI_OK: {count} cubes separately placed; green {green_error:.3f} mm',flush=True)
    except Exception as e:
        failure=str(e);record.update(passed=False,failure=failure);print('VERIFY_MULTI_FAILED:',failure,flush=True)
    finally:
        record['statuses']=statuses;record['results']=results
        (args.output/'record.json').write_text(json.dumps(record,indent=2))
        n.destroy_node();rclpy.shutdown()
    if failure:raise SystemExit(1)

if __name__=='__main__':main()
