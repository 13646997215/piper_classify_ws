#!/usr/bin/env python3
"""Capture unit CameraInfo, RGB-D samples and TF; never change calibration."""
import argparse
import json
from pathlib import Path
import time

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import CameraInfo,Image
from tf2_ros import Buffer,TransformListener
from rclpy.time import Time


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--source',required=True,choices=['hardware','simulation'])
    parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--duration',default=12.,type=float)
    parser.add_argument('--color-topic',default='/camera/color/image_raw')
    parser.add_argument('--depth-topic',default='/camera/depth/image_raw')
    parser.add_argument('--color-info-topic',default='/camera/color/camera_info')
    parser.add_argument('--depth-info-topic',default='/camera/depth/camera_info')
    parser.add_argument('--serial',default='not_recorded')
    args=parser.parse_args()
    if not 3<=args.duration<=120:parser.error('Duration must be 3..120 seconds')
    if args.output.exists():parser.error('Output directory already exists')
    args.output.mkdir(parents=True)
    rclpy.init();node=Node('capture_camera_reference');bridge=CvBridge()
    buffer=Buffer();listener=TransformListener(buffer,node)
    streams={};infos={};pairs=[];last_saved=-1.;failure=None;pending=None
    def image(key,message):streams[key]=message
    def info(key,message):infos[key]=message
    subscriptions=[
        node.create_subscription(Image,args.color_topic,lambda m:image('color',m),qos_profile_sensor_data),
        node.create_subscription(Image,args.depth_topic,lambda m:image('depth',m),qos_profile_sensor_data),
        node.create_subscription(CameraInfo,args.color_info_topic,lambda m:info('color',m),qos_profile_sensor_data),
        node.create_subscription(CameraInfo,args.depth_info_topic,lambda m:info('depth',m),qos_profile_sensor_data),
    ]
    deadline=time.monotonic()+args.duration
    try:
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
            if args.source=='hardware' and node.count_publishers('/clock'):
                raise RuntimeError('Simulation clock is present; isolate hardware in another ROS_DOMAIN_ID')
            if not all(k in streams and k in infos for k in ['color','depth']):continue
            if pending is None:
                color,depth=streams['color'],streams['depth']
                ct=color.header.stamp.sec+color.header.stamp.nanosec*1e-9
                dt=depth.header.stamp.sec+depth.header.stamp.nanosec*1e-9
                if abs(ct-dt)>.033 or ct-last_saved<.5:continue
                pending=(color,depth,time.monotonic())
                continue
            # Let image-time robot TF reach the buffer; never substitute latest TF.
            if time.monotonic()-pending[2]<.15:continue
            color,depth,_=pending;pending=None
            ct=color.header.stamp.sec+color.header.stamp.nanosec*1e-9
            dt=depth.header.stamp.sec+depth.header.stamp.nanosec*1e-9
            if abs(ct-dt)>.033 or ct-last_saved<.5:continue
            index=len(pairs)
            if index>=20:continue
            rgb=bridge.imgmsg_to_cv2(color,'bgr8');z=bridge.imgmsg_to_cv2(depth)
            if z.dtype not in [np.dtype('uint16'),np.dtype('float32')]:raise RuntimeError('Unsupported depth encoding')
            unit='millimetres' if z.dtype==np.uint16 else 'metres'
            np.savez_compressed(args.output/f'sample_{index:02d}.npz',color_bgr=rgb,depth=z)
            if index==0:cv2.imwrite(str(args.output/'color.png'),rgb)
            image_transforms={};image_missing=[]
            for target in ['link6','base_link','camera_depth_optical_frame']:
                try:
                    tf=buffer.lookup_transform(target,color.header.frame_id,Time.from_msg(color.header.stamp))
                    image_transforms[f'{target}<-{color.header.frame_id}']=message_to_ordereddict(tf)
                except Exception as error:image_missing.append({'target':target,'reason':str(error)})
            pairs.append({'index':index,'color_header':message_to_ordereddict(color.header),
                          'depth_header':message_to_ordereddict(depth.header),'time_difference_sec':ct-dt,
                          'depth_encoding':depth.encoding,'depth_units':unit,
                          'image_time_transforms':image_transforms,'missing_image_time_transforms':image_missing})
            last_saved=ct
        if not pairs:raise RuntimeError('No matching RGB-D/CameraInfo sample; check topics and QoS')
    except Exception as error:
        failure=str(error)
    transforms={};missing=[]
    optical=infos['color'].header.frame_id if 'color' in infos else 'camera_color_optical_frame'
    for target,source in [('link6',optical),('base_link',optical),
                          ('camera_depth_optical_frame',optical)]:
        try:
            transform=buffer.lookup_transform(target,source,rclpy.time.Time())
            transforms[f'{target}<-{source}']=message_to_ordereddict(transform)
        except Exception as error:missing.append({'target':target,'source':source,'reason':str(error)})
    topics=dict(node.get_topic_names_and_types())
    metadata={'source_label':args.source,'serial_label':args.serial,'source_label_is_not_device_authentication':True,
              'camera_info':{k:message_to_ordereddict(v) for k,v in infos.items()},'pairs':pairs,
              'latest_transforms':transforms,'missing_transforms':missing,'topic_types':topics,
              'failure':failure,'successful_stream_capture':bool(pairs) and failure is None,
              'limitations':['CameraInfo/TF reflect this running driver; original hand-eye raw samples still required',
                            'Aligned depth CameraInfo may equal RGB; native depth intrinsics need native topic',
                            'No measured noise or calibration is automatically imported into the simulator']}
    (args.output/'metadata.json').write_text(json.dumps(metadata,indent=2))
    node.destroy_node();rclpy.shutdown()
    if failure:raise SystemExit(failure)
    print(f'Captured {len(pairs)} RGB-D samples and CameraInfo; missing TF transforms: {len(missing)}')


if __name__=='__main__':main()
