#!/usr/bin/env python3
"""
measure_scene.py — 用 Gazebo Transport 直接测量场景，检测穿模与悬空。

为什么不用 ROS 服务
  gazebo_ros 的 get_link_state / get_model_state 等服务依赖 /gazebo 节点，
  而本机的 ros2 daemon 缓存经常拿不到这些服务（实测多次超时或不可见）。
  Gazebo Transport（gzserver 的 TCP 接口）不经过 ROS 发现，稳得多。
  gzserver 默认监听 11345 端口。

只读，不修改任何状态。

用法
  python3 scripts/tools/measure_scene.py
"""
import sys
import xml.etree.ElementTree as ET

import numpy as np
from gazebo.transport import TransportFactory
import gz

TABLE_TOP_Z = 0.70
TABLE_CENTER_Y = 0.25
TABLE_HALF_X = 0.45
TABLE_HALF_Y = 0.30

# 各 link 的碰撞盒半尺寸（米），来自 upgrade_urdf.py
COLLISION_HALF = {
    'piper_base_link':    (0.04625, 0.037, 0.040),
    'piper_link1':        (0.03293, 0.03575, 0.0335),
    'piper_link2':        (0.17099, 0.04527, 0.0455),
    'piper_link3':        (0.04003, 0.11936, 0.03738),
    'piper_link4':        (0.02836, 0.02754, 0.02922),
    'piper_link5':        (0.02698, 0.05374, 0.02746),
    'piper_link6':        (0.01715, 0.01717, 0.00168),
    'piper_gripper_base': (0.03225, 0.0695, 0.0285),
    'piper_link7':        (0.02588, 0.03613, 0.01113),
    'piper_link8':        (0.02588, 0.03613, 0.01113),
}

ARM_LINKS = list(COLLISION_HALF.keys())
BLOCKS = ['red_block', 'green_block', 'blue_block']


def get_model_names():
    node = TransportFactory().LookupNode(':11345')
    if not node:
        return []
    world = node.World()
    if not world:
        return []
    xml = world.World().serialize() if hasattr(world, 'serialize') else None
    if xml is None:
        return []
    root = ET.fromstring(xml)
    return [m.get('name') for m in root.iter('model')]


def get_pose(model, link):
    """返回 (model, link) 的世界坐标 (x, y, z)。"""
    node = TransportFactory().LookupNode(':11345')
    msg = gz.msgs.Pose()
    full = f'{model}::{link}' if link else model
    ok = node.GetPose(full, msg, 5.0) if hasattr(node, 'GetPose') else False
    if not ok:
        return None
    return np.array([msg.position.x, msg.position.y, msg.position.z])


def main():
    TransportFactory().Connect(':11345', 5)

    models = get_model_names()
    if not models:
        print('✘ 无法连接 gzserver (:11345) —— 仿真没在运行？')
        return 1
    print('场景中的模型:', ', '.join(sorted(models)))
    if 'piper' not in models:
        print('❌ 场景中不存在 piper 模型')
        return 1

    node = TransportFactory().LookupNode(':11345')

    # ------------------------------------------------------------ 机械臂
    print()
    print('=' * 78)
    print('机械臂 piper 各连杆世界坐标')
    print(f'（桌面顶面 z = {TABLE_TOP_Z}，桌面范围 x ±{TABLE_HALF_X}, '
          f'y {TABLE_CENTER_Y - TABLE_HALF_Y}..{TABLE_CENTER_Y + TABLE_HALF_Y}）')
    print('=' * 78)
    print(f"{'link':<22}{'x':>9}{'y':>9}{'z':>9}{'碰撞盒zmin':>12}  判定")
    print('-' * 78)

    poses = {}
    for name in ARM_LINKS:
        p = get_pose('piper', name)
        if p is None:
            print(f'  {name:<20} <读取失败>')
            continue
        poses[name] = p
        half = COLLISION_HALF[name]
        zmin = p[2] - half[2]
        over_table = (abs(p[0]) < TABLE_HALF_X + half[0]
                      and abs(p[1] - TABLE_CENTER_Y) < TABLE_HALF_Y + half[1])
        if over_table and zmin < TABLE_TOP_Z - 0.002:
            v = f'❌ 穿桌面 {-zmin + TABLE_TOP_Z:.4f}m'
        elif over_table:
            v = '✅ 桌面之上'
        else:
            v = '— 桌面范围外'
        print(f'  {name:<20}{p[0]:>9.4f}{p[1]:>9.4f}{p[2]:>9.4f}{zmin:>12.4f}  {v}')

    # ------------------------------------------------------------ 物块
    print()
    print('=' * 78)
    print('物块（50mm 立方体，底面应正好落在 z = 0.700）')
    print('=' * 78)
    print(f"{'block':<16}{'x':>9}{'y':>9}{'z':>9}{'底面':>9}  判定")
    print('-' * 78)
    HALF = 0.025
    for name in BLOCKS:
        p = get_pose(name, 'link')
        if p is None:
            print(f'  {name:<14} <读取失败>')
            continue
        bottom = p[2] - HALF
        gap = bottom - TABLE_TOP_Z
        if abs(gap) < 0.003:
            v = '✅ 平放于桌面'
        elif gap < 0:
            v = f'❌ 穿入桌面 {-gap:.4f}m'
        else:
            v = f'❌ 悬空 {gap:.4f}m'
        print(f'  {name:<14}{p[0]:>9.4f}{p[1]:>9.4f}{p[2]:>9.4f}{bottom:>9.4f}  {v}')

    # ------------------------------------------------------------ 结论
    print()
    print('=' * 78)
    base = poses.get('piper_base_link')
    if base is not None:
        gap = base[2] - TABLE_TOP_Z
        print(f'机械臂底座 z = {base[2]:.4f}   桌面顶面 z = {TABLE_TOP_Z:.4f}')
        if abs(gap) < 0.003:
            print('✅ 底座正好坐在桌面上')
        else:
            kind = '悬空' if gap > 0 else '嵌入桌面'
            print(f'❌ 底座{kind}，相差 {abs(gap):.4f} m')
            print(f'   修正方式: gazebo.launch.py 中 spawn_entity 的 -z '
                  f'应改为 {TABLE_TOP_Z:.3f}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
