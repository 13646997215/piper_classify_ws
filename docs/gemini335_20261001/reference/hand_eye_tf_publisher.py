#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
手眼标定TF发布器（已修改为piper_link6 → camera_link）

原版本: piper_base_link → camera_link
修改后: piper_link6 → camera_link （符合手眼标定实际含义）

手眼标定参数直接导入 toy_grasp/grasp_config.py 的 HAND_EYE_CALIB/HAND_EYE_QUAT，禁止复制数值。
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

from toy_grasp.grasp_config import HAND_EYE_CALIB, HAND_EYE_QUAT

class HandEyeTFPublisher(Node):
    def __init__(self):
        super().__init__('hand_eye_tf_publisher')
        
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        
        # 单一来源：grasp_config.py；标定更新后TF和抓取坐标变换同步生效。
        position = HAND_EYE_CALIB['position']
        rpy = HAND_EYE_CALIB['rpy']
        quat = (
            HAND_EYE_QUAT['x'], HAND_EYE_QUAT['y'],
            HAND_EYE_QUAT['z'], HAND_EYE_QUAT['w'])
        
        # 发布静态TF: piper_link6 → camera_link
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'piper_link6'  # ⭐ 修改：父坐标系改为piper_link6
        t.child_frame_id = 'camera_link'
        
        t.transform.translation.x = position[0]
        t.transform.translation.y = position[1]
        t.transform.translation.z = position[2]
        
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        
        self.tf_broadcaster.sendTransform(t)
        
        self.get_logger().info('✅ 手眼TF已发布: piper_link6 → camera_link')
        self.get_logger().info(f'   位置: x={position[0]:.5f}, y={position[1]:.5f}, z={position[2]:.5f}')
        self.get_logger().info(
            f'   姿态(RPY): roll={math.degrees(rpy[0]):.2f}°, '
            f'pitch={math.degrees(rpy[1]):.2f}°, yaw={math.degrees(rpy[2]):.2f}°')

def main(args=None):
    rclpy.init(args=args)
    node = HandEyeTFPublisher()
    
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
