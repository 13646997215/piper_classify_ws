#!/usr/bin/env python3
"""
================================================================================
upgrade_urdf.py — 从官方 piper_description.urdf 生成"碰撞体优化 + 惯量修正"版
================================================================================

用途
    官方 URDF 的 <collision> 全部直接引用 STL 网格。Gazebo 的 ODE 物理引擎
    对 mesh 碰撞体支持很差（需要三角化 + BVH），其中 link2.STL 有 73166 个三角面，
    会导致物理步进极慢甚至卡死。

    本脚本把每个 <collision> 的 mesh 换成一个内缩的 box（box 是 Gazebo 原生支持
    的凸体，效率最高），并修正 link6 明显失真的 inertial。

处理原则
    1. 只改 <collision>、link6 的 <inertial> 与 <visual> 的网格包名，
       **不碰任何 <joint>** → 运动学与外观与官方完全一致。
    2. box = 网格包围盒向内等距内缩
         inset = min(3mm, 8% × 最小边长)
       内缩是为了避免 box 比真实外形"胖"导致自碰撞误报；
       上限 8% 保证薄板类零件（link6 厚 4mm）不会被内缩到消失。
    3. link6 是半径 17.5mm、厚 4mm 的圆盘，按实心钢件重算质量与惯量。

用法
    python3 upgrade_urdf.py <官方URDF路径> <meshes目录> <输出URDF路径>
================================================================================
"""
import os
import struct
import sys
import xml.etree.ElementTree as ET

import numpy as np

MAX_INSET = 0.003          # 最大内缩量 3mm
MIN_RATIO = 0.08           # 内缩量不超过最小边长的 8%

# link6 修正值：圆盘 r=17.5mm, h=4mm, 按实心钢件(7800kg/m^3)估算
LINK6_FIX = {
    'mass': 0.030,
    'xyz': '0 0 -0.002',                       # 几何中心（官方给的 +0.046 落在网格外）
    'ixx': 2.4e-06, 'iyy': 2.4e-06, 'izz': 4.6e-06,
    'ixy': 0.0, 'ixz': 0.0, 'iyz': 0.0,
}


def stl_bbox(path):
    """读二进制 STL，返回 (min, max) 包围盒。"""
    data = open(path, 'rb').read()
    n_tri = struct.unpack('<I', data[80:84])[0]
    raw = np.frombuffer(data[84:84 + n_tri * 50], dtype=np.uint8).reshape(n_tri, 50)
    verts = raw[:, 12:48].copy().view(np.float32).reshape(-1, 3)
    return verts.min(0), verts.max(0)


def fmt(v):
    return f'{v:.6f}'


def make_box(center_xyz, size_xyz):
    """生成 <box>，并把包围盒中心写成 <collision><origin> 的偏移。

    关键: 包围盒中心通常不在 link 原点上（例如 base_link 网格中心在
    z = +0.043）。若省略这个 origin 偏移，box 会被放在 link 原点，
    相当于整体下沉，底座碰撞体会陷进支撑面。
    """
    geom_box = ET.Element('box')
    geom_box.set('size', f'{fmt(size_xyz[0])} {fmt(size_xyz[1])} {fmt(size_xyz[2])}')
    return geom_box


def set_collision_origin(collision_el, center_xyz):
    """在 <collision> 下写入 <origin xyz="cx cy cz" rpy="0 0 0"/>。"""
    origin = collision_el.find('origin')
    if origin is None:
        origin = ET.Element('origin')
        # origin 必须排在 geometry 之前
        geom = collision_el.find('geometry')
        if geom is not None:
            collision_el.insert(list(collision_el).index(geom), origin)
        else:
            collision_el.append(origin)
    origin.set('xyz', f'{fmt(center_xyz[0])} {fmt(center_xyz[1])} {fmt(center_xyz[2])}')
    origin.set('rpy', '0 0 0')


def upgrade(urdf_path, mesh_dir, out_path):
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    report = []

    for link in root.findall('link'):
        name = link.get('name')

        # ---- 1. collision: mesh → box --------------------------------------
        col = link.find('collision')
        if col is not None and col.find('geometry/mesh') is not None:
            mesh_name = col.find('geometry/mesh').get('filename').split('/')[-1]
            mn, mx = stl_bbox(os.path.join(mesh_dir, mesh_name))
            size = mx - mn
            center = (mn + mx) / 2.0
            inset = min(MAX_INSET, MIN_RATIO * size.min())
            new_size = size - 2.0 * inset

            geom = col.find('geometry')
            for child in list(geom):
                geom.remove(child)
            geom.append(make_box(center, new_size))
            # 关键: 把包围盒中心写进 <collision><origin>，
            # 否则 box 会落在 link 原点而不是包围盒中心，整体下沉导致穿模
            set_collision_origin(col, center)

            report.append(
                f'  {name:14s} collision: mesh({mesh_name}) '
                f'-> box size=({fmt(new_size[0])}, {fmt(new_size[1])}, {fmt(new_size[2])}) '
                f'origin=({fmt(center[0])}, {fmt(center[1])}, {fmt(center[2])})'
            )

        visual_mesh = link.find('visual/geometry/mesh')
        if visual_mesh is not None:
            uri = visual_mesh.get('filename')
            if uri and uri.startswith('package://piper_description/meshes/'):
                visual_mesh.set('filename', uri.replace('package://piper_description/',
                                                        'package://classify_description/', 1))

        # ---- 2. link6 inertial 修正 ----------------------------------------
        if name == 'link6':
            inertial = link.find('inertial')
            origin = inertial.find('origin')
            origin.set('xyz', LINK6_FIX['xyz'])
            inertial.find('mass').set('value', str(LINK6_FIX['mass']))
            inertia = inertial.find('inertia')
            for k in ('ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz'):
                inertia.set(k, str(LINK6_FIX[k]))
            report.append(
                f'  {name:14s} inertial : 官方 mass=0.007 ixx=1.521e-03(质心落在网格外) '
                f'-> mass={LINK6_FIX["mass"]} ixx={LINK6_FIX["ixx"]:.1e}'
            )

    # ---- 3. 确认 base_link 是根连杆 ----------------------------------------
    # 刻意不添加 <link name="world"> + <joint name="world_to_base" type="fixed">。
    #
    # 原因: 早先版本加了这两个元素，结果机械臂被固定关节焊死在世界原点，
    #       spawn_entity 传入的 -x -y -z -R -P -Y 全部失效，
    #       表现为"机械臂怎么改 spawn 参数都转不过来"。
    #       Gazebo 中模型自带的 fixed joint 约束优先于 spawn 的初始位姿。
    #
    # 正确做法: 让 base_link 做根连杆，位姿完全交给 spawn_entity 控制。
    #           MoveIt 侧需要固定根关节时，在 SRDF 里用 virtual_joint 声明，
    #           而不是往 URDF 里塞 fixed joint。
    roots = [l.get('name') for l in root.findall('link')
             if not any(j.find('child') is not None
                        and j.find('child').get('link') == l.get('name')
                        for j in root.findall('joint'))]
    if roots != ['base_link']:
        raise SystemExit(
            f'✘ 根连杆应为 base_link，实际为 {roots}\n'
            f'  URDF 里不应存在 <link name="world"> 配合 world_to_base 固定关节，\n'
            f'  否则 spawn_entity 的位姿参数会失效。'
        )
    report.append(f'  根连杆         {roots[0]}（spawn 位姿可正常生效）')

    ET.indent(tree, space='  ')
    tree.write(out_path, encoding='utf-8', xml_declaration=True)

    print(f'已生成: {out_path}')
    print('改动明细:')
    for line in report:
        print(line)


if __name__ == '__main__':
    if len(sys.argv) != 4:
        print(__doc__)
        sys.exit(1)
    upgrade(sys.argv[1], sys.argv[2], sys.argv[3])
