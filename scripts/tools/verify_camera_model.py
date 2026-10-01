#!/usr/bin/env python3
"""Verify calibrated optical chain/profile; --live also records actual ROS images.
Python is validation tooling, not a perception or registration implementation.
"""
import argparse
import hashlib
import importlib.util
import json
import math
import itertools
import subprocess
from pathlib import Path
import time
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
import yaml

ROOT = Path(__file__).resolve().parents[2]
DESCRIPTION = ROOT/'src/description/classify_description'

def transform(xyz, rpy):
    t=np.eye(4); t[:3,:3]=Rotation.from_euler('xyz',rpy).as_matrix();t[:3,3]=xyz;return t

def static_check(urdf=None):
    profile=yaml.safe_load((DESCRIPTION/'config/gemini335.yaml').read_text())
    he=profile['hand_eye']
    color=profile['color']
    fov=[math.degrees(2*math.atan(color['width']/(2*color['fx']))),math.degrees(2*math.atan(color['height']/(2*color['fy'])))]
    assert np.allclose(fov,[color['hfov_deg'],color['vfov_deg']],atol=1e-8), 'FOV metadata and actual lens intrinsics differ'
    root=ET.parse(urdf or DESCRIPTION/'urdf/piper_sim.urdf').getroot()
    joints={j.find('child').get('link'):j for j in root.findall('joint')}
    def chain(frame, parent):
        result=np.eye(4)
        while frame!=parent:
            joint=joints[frame];origin=joint.find('origin')
            result=transform(list(map(float,origin.get('xyz','0 0 0').split())),list(map(float,origin.get('rpy','0 0 0').split())))@result
            frame=joint.find('parent').get('link')
        return result
    target=transform(he['xyz_m'],he['rpy_rad'])
    actual=chain('camera_color_optical_frame','link6')
    assert np.allclose(actual,target,atol=1e-10), 'Generated URDF differs from reference optical calibration'
    offset=np.linalg.inv(chain('camera_depth_optical_frame','link6'))@actual
    assert np.allclose(offset[:3,3],profile['depth']['color_origin_in_depth_optical_m'],atol=1e-10)
    assert np.allclose(offset[:3,:3],np.eye(3),atol=1e-10)
    source=Path(he['source'])
    if source.is_file():
        assert hashlib.sha256(source.read_bytes()).hexdigest()==he['source_sha256'], 'Reference file changed; re-import hand-eye values'
    assert he['calibrated_frame']=='camera_color_optical_frame'
    for sensor in root.findall('gazebo/sensor'):
        key='color' if sensor.get('type')=='camera' else 'depth'
        values=profile[key]
        intrinsics=sensor.find('camera/lens/intrinsics')
        assert intrinsics is not None, 'Missing actual render intrinsics'
        for k in ['fx','fy','cx','cy']:assert abs(float(intrinsics.findtext(k))-values[k])<1e-8
        assert float(sensor.findtext('update_rate'))==15
    spec=importlib.util.spec_from_file_location('workcell_config',ROOT/'src/moveit/classify_moveit_config/launch/workcell_config.py')
    mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
    parameters=mod.parameters(DESCRIPTION)
    counts={name:len(parameters[name+'_boxes'])//7 for name in ['table_top','table_legs','bin_red','bin_blue']}
    assert counts=={'table_top':1,'table_legs':4,'bin_red':5,'bin_blue':5}
    world=ET.parse(DESCRIPTION/'worlds/classify.world').getroot()
    for color in ['red','green','blue']:
        xyz=list(map(float,world.find(f"world/model[@name='{color}_block']/pose").text.split()))[:3]
        assert parameters['pickup_x_min']<xyz[0]<parameters['pickup_x_max']
        assert abs(xyz[1])<parameters['pickup_y_abs_max']
        assert abs(xyz[2]-.347-parameters['table_top_z']-parameters['cube_size']/2)<1e-6
    return profile, {'static_pass':True,'stream_mode':profile.get('stream_mode','rgb_4_3'),'rgb_fov_deg':fov,'link6_to_color_optical':actual.tolist(),'depth_to_color_optical':offset.tolist(),'fixed_geometry_boxes':counts}

def live_check(profile, output):
    import cv2
    import rclpy
    from rclpy.time import Time
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image,CameraInfo
    from tf2_ros import Buffer,TransformListener
    from classify_interfaces.msg import DetectedBlocks, SceneStatus
    from moveit_msgs.srv import GetPlanningScene
    from moveit_msgs.msg import PlanningSceneComponents
    rclpy.init()
    node=rclpy.create_node('gemini335_model_probe',parameter_overrides=[rclpy.parameter.Parameter('use_sim_time',value=True)])
    buffer=Buffer(node=node);listener=TransformListener(buffer,node)
    messages={};stamps={};pairs={};detections=[];statuses=[]
    def callback(key,msg):
        messages[key]=msg
        if isinstance(msg,Image):
            ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
            stamps.setdefault(key,[]).append(ns)
            if key in ('rgb','aligned'):pairs.setdefault(key,{})[ns]=msg
    topics={'rgb':('/camera/color/image_raw',Image),'native':('/camera/native/depth/image_raw',Image),'aligned':('/camera/depth/image_raw',Image),'color_info':('/camera/color/camera_info',CameraInfo),'aligned_info':('/camera/depth/camera_info',CameraInfo),'native_info':('/camera/native/depth/camera_info',CameraInfo)}
    subscriptions=[node.create_subscription(cls,topic,lambda msg,k=key:callback(k,msg),qos_profile_sensor_data) for key,(topic,cls) in topics.items()]
    subscriptions.append(node.create_subscription(DetectedBlocks,'/classify/detections',lambda m:detections.append(m),10))
    subscriptions.append(node.create_subscription(SceneStatus,'/classify/scene_status',lambda m:statuses.append(m),10))
    end=time.monotonic()+12
    while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
    try:
        assert set(messages)==set(topics),f'Missing topics: {set(topics)-set(messages)}'
        result={'topics':{}}
        for key,msg in messages.items():
            result['topics'][key]={'frame':msg.header.frame_id,'width':msg.width,'height':msg.height}
            configured=profile['depth' if key.startswith('native') else 'color']
            assert (msg.width,msg.height)==(configured['width'],configured['height']), (key,'Running simulation has stale dimensions')
            if isinstance(msg,Image):
                unique=np.unique(stamps[key]);freq=1e9/np.median(np.diff(unique))
                result['topics'][key].update(encoding=msg.encoding,frames=len(unique),sim_fps=float(freq))
                assert 14.5<freq<15.1,(key,freq)
            else:result['topics'][key]['K']=list(msg.k)
        assert messages['rgb'].encoding=='rgb8'
        assert messages['native'].encoding=='32FC1' and messages['aligned'].encoding=='16UC1'
        for name,key in [('color_info','color'),('aligned_info','color'),('native_info','depth')]:
            p=profile[key];m=messages[name]
            assert np.allclose([m.k[0],m.k[4],m.k[2],m.k[5]],[p[k] for k in ['fx','fy','cx','cy']])
        common=sorted(pairs['rgb'].keys() & pairs['aligned'].keys())
        assert len(common)>20,'RGB/D2C exact pairs missing'
        rgb=pairs['rgb'][common[-2]];depth=pairs['aligned'][common[-2]]
        assert rgb.header.frame_id==depth.header.frame_id=='camera_color_optical_frame'
        def tf_matrix(parent,child,stamp=Time()):
            t=buffer.lookup_transform(parent,child,stamp).transform
            m=np.eye(4);q=t.rotation;m[:3,:3]=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix()
            m[:3,3]=[t.translation.x,t.translation.y,t.translation.z];return m
        measured=tf_matrix('link6','camera_color_optical_frame')
        target=transform(profile['hand_eye']['xyz_m'],profile['hand_eye']['rpy_rad'])
        assert np.allclose(measured,target,atol=1e-10)
        base=tf_matrix('base_link','camera_color_optical_frame',Time(nanoseconds=common[-2]))
        result.update(live_pass=True,exact_pairs=len(common),live_link6_to_color_optical=measured.tolist(),camera_to_base=base.tolist())
        raw=np.frombuffer(rgb.data,np.uint8).reshape(rgb.height,rgb.step)[:,:rgb.width*3].reshape(rgb.height,rgb.width,3)
        z=np.frombuffer(depth.data,np.uint16).reshape(depth.height,depth.step//2)[:,:depth.width]
        output.mkdir(parents=True,exist_ok=True)
        cv2.imwrite(str(output/'home_rgb.png'),cv2.cvtColor(raw,cv2.COLOR_RGB2BGR))
        # Independent render check: project Gazebo cube corners with K/TF and
        # compare to actual RGB silhouettes. Model truth is test input only.
        hsv=cv2.cvtColor(raw,cv2.COLOR_RGB2HSV)
        k=np.array(messages['color_info'].k).reshape(3,3)
        checks={}
        for color,lo,hi in [('red',(0,80,50),(12,255,255)),('green',(35,80,50),(90,255,255)),('blue',(95,80,50),(135,255,255))]:
            truth=list(map(float,subprocess.check_output(['gz','model','-m',color+'_block','-p'],text=True,timeout=10).split()))
            rotation=Rotation.from_euler('xyz',truth[3:]).as_matrix()
            local=np.array(list(itertools.product([-.025,.025],repeat=3)))
            corners=local@rotation.T+np.array(truth[:3])-[0,0,.347]
            optical=(corners-base[:3,3])@base[:3,:3]
            uv=(optical@k.T)[:,:2]/optical[:,2,None]
            expected=np.r_[uv.min(axis=0),uv.max(axis=0)]
            mask=cv2.inRange(hsv,np.array(lo),np.array(hi))
            if color=='red':mask|=cv2.inRange(hsv,np.array((170,80,50)),np.array((179,255,255)))
            contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            rectangles=[cv2.boundingRect(c) for c in contours if cv2.contourArea(c)>100]
            target=uv.mean(axis=0)
            x,y,w,h=min(rectangles,key=lambda b:np.linalg.norm(np.array([b[0]+b[2]/2,b[1]+b[3]/2])-target))
            observed=np.array([x,y,x+w-1,y+h-1])
            error=float(np.max(np.abs(observed-expected)))
            assert error<2.5,(color,'Actual render disagrees with lens K',error)
            checks[color]={'expected_bbox':expected.tolist(),'observed_bbox':observed.tolist(),'max_edge_error_px':error}
        result['render_projection']=checks
        result['max_render_edge_error_px']=max(c['max_edge_error_px'] for c in checks.values())
        result['aligned_valid_fraction']=float(np.count_nonzero(z)/z.size)
        reliable=[m for m in detections if m.camera_stable and len(m.blocks)==3]
        assert reliable,'No reliable 3-block observations at HOME'
        result['latest_blocks_base']={b.color:[b.center.x,b.center.y,b.center.z] for b in reliable[-1].blocks}
        assert statuses and statuses[-1].ready,'Planning scene not ready at HOME'
        scene_client=node.create_client(GetPlanningScene,'/get_planning_scene')
        assert scene_client.wait_for_service(timeout_sec=3)
        req=GetPlanningScene.Request();req.components.components=PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        future=scene_client.call_async(req);rclpy.spin_until_future_complete(node,future,timeout_sec=5)
        assert future.done() and future.result()
        objects=future.result().scene.world.collision_objects
        result['fixed_geometry']={o.id:{'frame':o.header.frame_id,'positions':[(Rotation.from_quat([o.pose.orientation.x,o.pose.orientation.y,o.pose.orientation.z,o.pose.orientation.w]).apply([p.position.x,p.position.y,p.position.z])+[o.pose.position.x,o.pose.position.y,o.pose.position.z]).tolist() for p in o.primitive_poses],'dimensions':[list(p.dimensions) for p in o.primitives]} for o in objects if o.id in ['table_top','table_legs','bin_red','bin_blue']}
        for name in ['bin_red','bin_blue']:
            assert len(result['fixed_geometry'][name]['positions'])==5
            assert abs(result['fixed_geometry'][name]['positions'][0][0]-.52)<1e-8
        return result
    finally:
        node.destroy_node();rclpy.shutdown()

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--urdf',type=Path);parser.add_argument('--live',action='store_true');parser.add_argument('--output',type=Path,default=ROOT/'docs/gemini335_20261001')
    args=parser.parse_args();profile,result=static_check(args.urdf)
    if args.live:result.update(live_check(profile,args.output))
    args.output.mkdir(parents=True,exist_ok=True)
    (args.output/('camera_live.json' if args.live else 'camera_static.json')).write_text(json.dumps(result,indent=2))
    print(json.dumps(result,ensure_ascii=False,indent=2))
