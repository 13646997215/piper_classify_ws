#!/usr/bin/env python3
"""Replay sensor evidence into the actual C++ detector in an isolated ROS domain."""
import argparse
import copy
import json
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from classify_interfaces.msg import DetectedBlocks
from geometry_msgs.msg import TransformStamped
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, CameraInfo
from tf2_msgs.msg import TFMessage
from scipy.spatial.transform import Rotation


def main():
    parser=argparse.ArgumentParser();parser.add_argument('capture',type=Path)
    parser.add_argument('--negative',action='store_true',help='Verify mismatched stamps and paused-clock stream loss')
    args=parser.parse_args()
    record=json.loads((args.capture/'record.json').read_text())
    rclpy.init();node=Node('perception_replay')
    clock=node.create_publisher(Clock,'/clock',10)
    tf=node.create_publisher(TFMessage,'/tf',100)
    rgb_pub=node.create_publisher(Image,'/camera/color/image_raw',qos_profile_sensor_data)
    depth_pub=node.create_publisher(Image,'/camera/depth/image_raw',qos_profile_sensor_data)
    info_pub=node.create_publisher(CameraInfo,'/camera/color/camera_info',qos_profile_sensor_data)
    outputs=[]
    sub=node.create_subscription(DetectedBlocks,'/classify/detections',lambda msg:outputs.append(msg),10)
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
    spin(2)
    for sample in (record['samples'][:10] if args.negative else record['samples']):
        data=np.load(args.capture/sample['file']);stamp=int(data['stamp'])
        c=Clock();c.clock.sec=stamp//1000000000;c.clock.nanosec=stamp%1000000000
        transform=TransformStamped();transform.header.stamp=c.clock
        transform.header.frame_id='base_link';transform.child_frame_id='camera_color_optical_frame'
        matrix=data['camera_to_base'];q=Rotation.from_matrix(matrix[:3,:3]).as_quat()
        transform.transform.translation.x,transform.transform.translation.y,transform.transform.translation.z=map(float,matrix[:3,3])
        transform.transform.rotation.x,transform.transform.rotation.y,transform.transform.rotation.z,transform.transform.rotation.w=map(float,q)
        clock.publish(c);tf.publish(TFMessage(transforms=[transform]))
        image=Image();image.header.stamp=c.clock;image.header.frame_id='camera_color_optical_frame'
        image.height,image.width=data['depth'].shape;image.encoding='rgb8';image.step=image.width*3
        image.data=data['rgb'].tobytes()
        depth=Image();depth.header=image.header;depth.height=image.height;depth.width=image.width
        depth.encoding='32FC1';depth.step=depth.width*4;depth.data=data['depth'].tobytes()
        info=CameraInfo();info.header=image.header;info.width=image.width;info.height=image.height
        info.k=data['k'].reshape(-1).tolist()
        info_pub.publish(info);spin(.04)
        rgb_pub.publish(image);depth_pub.publish(depth);spin(.10)
    spin(.3)
    if args.negative:
        assert any(m.blocks for m in outputs), 'No valid control observations'
        boundary = len(outputs)
        for _ in range(5):
            stamp += 250000000
            c.clock.sec=stamp//1000000000;c.clock.nanosec=stamp%1000000000
            transform.header.stamp=copy.deepcopy(c.clock)
            clock.publish(c);tf.publish(TFMessage(transforms=[transform]))
            image.header.stamp=copy.deepcopy(c.clock)
            depth.header=copy.deepcopy(image.header)
            depth.header.stamp.nanosec += 1
            rgb_pub.publish(image);depth_pub.publish(depth);spin(.1)
        spin(1.2)
        assert not any(m.blocks for m in outputs[boundary:]), 'Unmatched RGB-D generated detections'
        assert any(not m.camera_stable for m in outputs[boundary:]), 'Paused-clock watchdog did not expire'
        report={'mismatched_nanosecond_stamps_rejected':True,'paused_clock_sensor_timeout':True}
        (args.capture/'replay_negative.json').write_text(json.dumps(report,indent=2))
        print('SYNC_GUARDS_OK',report,flush=True)
        node.destroy_node();rclpy.shutdown();return
    result=[];maximum=0;accepted=0
    for msg in outputs:
        for b in msg.blocks:
            p=np.array([b.center.x,b.center.y,b.center.z])
            truth=np.array(record['truth_world'][b.color][:3]);truth[2]-=.347
            error=float(np.linalg.norm(p-truth)*1000);maximum=max(maximum,error);accepted+=1
            result.append({'stamp':msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec,
                           'color':b.color,'center':p.tolist(),'error_mm':error})
    report={'accepted':accepted,'max_error_mm':maximum,
            'unavailable_outputs':sum(not m.camera_stable for m in outputs),'results':result}
    (args.capture/'replay_fixed.json').write_text(json.dumps(report,indent=2))
    print('REPLAY',accepted,'accepted centres; max error mm',round(maximum,3),
          'unavailable',report['unavailable_outputs'],flush=True)
    node.destroy_node();rclpy.shutdown()
    assert accepted>=30,'Too few usable observations'
    assert maximum<5,'Accepted a misplaced block'


if __name__=='__main__':main()
