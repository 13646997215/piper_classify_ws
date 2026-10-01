#!/usr/bin/env python3
"""数值 IK 可达性验证：确认 5 个工作点在当前场景布局下真的够得着。

为什么需要这个
--------------
场景布局（桌子/物块/料盒的位置）是会改的，改完光看 Gazebo 画面判断不了
"够不够得着"。Piper 的关节限位又是单侧的（joint2 只能 [0, 3.14]，
joint3 只能 [-2.967, 0]），很多在几何上明明在臂展范围内的工作点，
实际上无解。所以每次改布局都要跑一遍这个脚本。

注意结果里 j1 的反直觉数值
--------------------------
red_block 在基座正前方，但解出来的 j1 是 103° 而不是接近 0°。这不是 bug：
单侧限位锁死了肘部配置，只有这一族解同时满足位置和 45 度斜抓姿态。
MoveIt 在关节空间搜索，多解不影响规划，只是执行时基座会先转一大圈。

用法:
    python3 scripts/tools/check_ik.py
"""

import sys, numpy as np
sys.path.insert(0, '/tmp/opencode')
from scipy.optimize import least_squares

# Piper 6-DOF chain, from the official URDF (base -> link6)
O = [((0,0,0.123),(0,0,0)), ((0,0,0),(1.5708,-0.1359,-3.1416)),
     ((0.28503,0,0),(0,0,-1.7939)), ((-0.021984,-0.25075,0),(1.5708,0,0)),
     ((0,0,0),(-1.5708,0,0)), ((8.8259e-05,-0.091,0),(1.5708,0,0))]
LO = np.array([-2.618, 0.0, -2.967, -1.745, -1.22, -2.0944])
HI = np.array([ 2.618, 3.14,  0.0,  1.745,  1.22,  2.0944])
TCP = 0.135

def rpy(r):
    cr,sr,cp,sp,cy,sy = np.cos(r[0]),np.sin(r[0]),np.cos(r[1]),np.sin(r[1]),np.cos(r[2]),np.sin(r[2])
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                     [-sp,   cp*sr,      cp*cr]])
Rr = []
for xyz, rot in O:
    A = np.eye(4); A[:3,:3] = rpy(rot); A[:3,3] = xyz; Rr.append(A)

def fk(q):
    T = np.eye(4)
    for i, A in enumerate(Rr):
        T = T @ A
        Tj = np.eye(4); Tj[:3,:3] = rpy([0,0,q[i]]); T = T @ Tj
    return T

HOME = np.array([0, 0.87266, -1.22173, 0, 1.22173, 0.0])

def err(q, Td):
    T = fk(q)
    pos = T[:3,3] + T[:3,:3] @ np.array([0,0,TCP])
    dp = pos - Td[:3,3]
    Re = T[:3,:3] @ Td[:3,:3].T
    dw = 0.5*np.array([Re[2,1]-Re[1,2], Re[0,2]-Re[2,0], Re[1,0]-Re[0,1]])
    return np.concatenate([dp, dw])

def ik(Td, seeds=40):
    best = None; rng = np.random.default_rng(0)
    for s in range(seeds):
        q0 = np.clip(LO + rng.random(6)*(HI-LO) if s else HOME.copy(), LO+1e-4, HI-1e-4)
        try:
            r = least_squares(err, q0, args=(Td,), bounds=(LO,HI),
                              xtol=1e-13, ftol=1e-13, gtol=1e-13, max_nfev=400)
        except Exception:
            continue
        if best is None or r.cost < best.cost:
            best = r
    return best

def target(px, py, pz, tool_z):
    """Build a target pose: tool z-axis points along tool_z (unit), y kept vertical-ish."""
    z = np.array(tool_z, float); z /= np.linalg.norm(z)
    up = np.array([0,0,1.0])
    if abs(np.dot(z, up)) > 0.98:
        up = np.array([0,1.0,0])
    x = np.cross(up, z); x /= np.linalg.norm(x)
    y = np.cross(z, x)
    R = np.column_stack([x, y, z])
    T = np.eye(4); T[:3,:3] = R; T[:3,3] = (px, py, pz)
    return T

# Arm base at (0,0), facing +Y. 45 deg tilted grasp: tool z points
# forward-and-down into the scene:  (0, +0.707, -0.707)
TOOL_Z = (0.0, 0.7071, -0.7071)

WORK = [
    ("red_block",   0.00,  0.30, 0.375, 0.00),
    ("green_block",-0.14,  0.30, 0.375, 0.00),
    ("blue_block",  0.14,  0.30, 0.375, 0.00),
    ("bin_red",     0.22,  0.42, 0.372, 0.02),
    ("bin_blue",   -0.22,  0.42, 0.372, 0.02),
]

print("=== IK 可达性: 基座 (0,0,0.347) 朝 +Y, 45度斜抓 ===")
print("    工具 z 轴 = (0, +0.707, -0.707)  即朝前下方")
print()
worst_pos = 0.0; worst_rot = 0.0; allok = True
for name, x, y, z, up in WORK:
    Td = target(x, y, z + up, TOOL_Z)
    r = ik(Td)
    if r is None:
        print(f"  ✘ {name:12s} 无解"); allok = False; continue
    e = err(r.x, Td)
    dp = np.linalg.norm(e[:3]) * 1000.0
    dr = np.degrees(np.linalg.norm(e[3:]))
    q = np.degrees(r.x)
    status = "✔" if (dp < 1.0 and dr < 1.0) else "✘"
    if status == "✘": allok = False
    worst_pos = max(worst_pos, dp); worst_rot = max(worst_rot, dr)
    print(f"  {status} {name:12s} ({x:+.2f},{y:+.2f})  位置误差 {dp:6.2f} mm  姿态误差 {dr:5.2f}°")
    print(f"      关节角(度): j1={q[0]:7.1f} j2={q[1]:7.1f} j3={q[2]:7.1f} "
          f"j4={q[3]:6.1f} j5={q[4]:6.1f} j6={q[5]:7.1f}")

print()
print(f"最大位置误差 {worst_pos:.2f} mm, 最大姿态误差 {worst_rot:.2f}°")
print("结论:", "全部可达 ✔" if allok else "存在不可达点 ✘")
