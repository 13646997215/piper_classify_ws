#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
==============================================================================
嵌赛抓玩具 — 统一参数配置中心
==============================================================================
所有抓取相关节点都从这里导入参数, 统一管理。

用法:
  from toy_grasp.grasp_config import GRIPPER, PICK_POSE, GRIPPER_OFFSET, T_CAM_EE, ...

修改参数后 symlink-install 下 python 改动即时生效 (无需重新 build)。
==============================================================================
"""

import math
import os
import numpy as np
from tf_transformations import euler_matrix, inverse_matrix, quaternion_from_matrix


# ============================================================
# 1. 手眼标定参数
# ============================================================
# 标定时间: 2026-06-22 18:56 (新标定1, 31次采集)
# 标定含义: T_cam_ee — 相机坐标系相对于末端(link6)坐标系的变换
#   position: 相机光心在末端坐标系中的位置 (m)
#   rpy: 相机坐标系相对于末端坐标系的旋转 (rad)

HAND_EYE_CALIB = {
    # 标定结果文件 (仅元信息记录, 用相对写法便于整体迁移)
    'result_file': 'result/2026-06-22_18-56-28_calibration.json',

    # 相机在末端坐标系中的位置 (m)
    # x: 后方93mm, y: 右侧19mm, z: 上方29mm
    'position': [-0.09294, 0.01914, 0.02915],

    # 相机坐标系相对于末端坐标系的旋转 (rad)
    # Roll: -3.4°, Pitch: +0.2°, Yaw: -90.7°
    'rpy': [-0.05907, 0.00275, -1.58335],

    # 标定元信息
    'date': '2026-06-22',
    'samples': 31,
    'yaw_deg': -90.72,
    'roll_deg': -3.38,
    'pitch_deg': 0.16,
}


# ============================================================
# 2. 手眼标定矩阵（从RPY正确计算）
# ============================================================

def _build_T_cam_ee():
    """
    从标定参数构建 4x4 变换矩阵 T_cam_ee
    T_cam_ee 将相机坐标系中的点映射到 link6 坐标系:
      P_link6 = T_cam_ee @ P_cam
    """
    pos = HAND_EYE_CALIB['position']
    rpy = HAND_EYE_CALIB['rpy']
    roll, pitch, yaw = rpy[0], rpy[1], rpy[2]

    # 旋转矩阵 R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    R = np.array([
        [cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr],
        [sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr],
        [  -sp,             cp*sr,             cp*cr ]
    ])

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = pos
    return T

# T_cam_ee: 相机在link6坐标系中的 4x4 变换矩阵
# P_link6 = T_cam_ee @ P_cam
T_CAM_EE = _build_T_cam_ee()

# T_ee_cam: link6在相机坐标系中的 4x4 变换矩阵 (T_cam_ee 的逆)
# P_cam = T_ee_cam @ P_link6
T_EE_CAM = inverse_matrix(T_CAM_EE)

# 四元数 (xyzw)，用于 ROS2 消息
_quat = quaternion_from_matrix(T_CAM_EE)
HAND_EYE_QUAT = {
    'x': float(_quat[0]),
    'y': float(_quat[1]),
    'z': float(_quat[2]),
    'w': float(_quat[3]),
}


# ============================================================
# 3. 机械臂运动参数
# ============================================================

ARM = {
    # MoveIt语义模型标识（与piper.srdf保持一致）
    'planning_group': 'piper_arm',
    'end_effector_link': 'piper_link6',
    # _compute输出的目标位姿属于Piper基座系。底盘移动后world与基座不再重合，
    # 必须显式使用piper_base_link，否则MoveIt会按world→底盘TF二次变换目标。
    'planning_frame': 'piper_base_link',
    'joint_names': [f'piper_joint{i}' for i in range(1, 7)],

    # 速度/加速度限制 (0~1)
    'max_velocity_scaling': 0.3,
    'max_acceleration_scaling': 0.3,

    # 规划超时 (秒)
    'planning_time': 5.0,
    'num_planning_attempts': 5,

    # 碰撞检测 (True=启用, False=关闭)
    # 注意: 开启后 MoveIt 会检查自碰撞, 可能拒绝原本可行的解。
    #       如果 IK 频繁失败, 先设为 False 调试, 确认坐标链正确后再开启。
    'avoid_collisions': False,

    # 机械臂最大工作距离 (base_link 到 link6, m)
    'max_reach': 0.62,
}

# Home 位姿 (度) — 嵌赛官方夹爪 HOME
HOME_DEG = [0.0, 50.0, -70.0, 0.0, 70.0, 0.0]
HOME_RAD = [math.radians(d) for d in HOME_DEG]

# Home 位夹爪位置 (m) — 睿抗全开值(两爪开口, slider_ctrl范围-0.003~0.05)
HOME_GRIPPER = 0.050

# ============================================================
# 3.1 HOME位 link6 在 base_link 中的坐标 (m)
# ============================================================
# 来源: FK 计算 HOME_DEG 时 link6 的位置
# 用途: COARSE 远距前移的基准位置
# 注意: 如果改了 HOME_DEG，需要同步更新此值!
HOME_LINK6_POS = {
    'x': 0.086,    # link6 在 base_link X 方向 (前方)
    'y': 0.0,      # link6 在 base_link Y 方向 (左右)
    'z': 0.310,    # link6 在 base_link Z 方向 (高度)
}


# ============================================================
# 4. 夹爪参数
# ============================================================

GRIPPER = {
    # 夹爪位置 (m) — slider_ctrl 范围 -0.003~0.05, 两爪开口量
    'pos_open': 0.050,      # 全开 (HOME 全开值)
    'pos_grasp': 0.006,     # 抓取闭合值 (2026-07-07 用户微调: 由0.010收紧到0.006)
    'pos_close': 0.0,       # 全闭

    # 夹爪力矩 (SDK, node 会 clip 到 0.5~3)
    'effort_max': 5.0,
    'effort_default': 5.0,

    # gripper_val_mutiple (睿抗用1)
    'val_multiple': 1,
}

# ============================================================
# 4.1 抱球中心偏移 (link6 → gripper_base)
# ============================================================
# 来源: 嵌赛.睿抗/robot/urdf/robot.urdf 中 gripper_joint 的 origin
# 含义: 抱球中心在 link6 坐标系中的位置
#   x: 前方 79.5mm, y: 0, z: 上方 119.5mm
# 用途: IK求解前，从苹果位置(link6坐标系)减去此偏移，
#       使得 link6 到达"苹果-偏移"位置，抱球中心刚好对准苹果

GRIPPER_OFFSET = {
    'x': 0.0,       # 官方直夹爪, 前方无偏移
    'y': 0.0,       # 左右无偏移
    'z': 0.14,      # 夹持中心在link6 +Z方向 ~140mm, 阶段4实测微调
}


# ============================================================
# 5. 抓取位姿参数
# ============================================================

PICK_POSE = {
    # 抓取朝向 (四元数 xyzw) — 斜向下抓取; link6 +Z 为接近轴(夹爪伸出方向)
    'orientation': {
        'x': 0.0,
        'y': 0.9538,
        'z': 0.0,
        'w': 0.3005,
    },

    # MoveIt IK 目标精度 (m)
    'position_tolerance': 0.01,
}


# ============================================================
# 5.1 偏航对齐参数 (玩具长边方向对齐)
# ============================================================
# 视觉给玩具偏转角 yaw(弧度) → 绕末端接近轴(link6 +Z)旋转夹爪对齐长边中心。
# 原始角: raw = YAW_SIGN*yaw + YAW_OFFSET。
# 执行角: use = fold(raw) 到 [-π/2, +π/2]，利用夹爪/长边的180°等价性。
# 合成: q_new = q_PICK ⊗ q_z(use)  (右乘=绕末端自身Z, intrinsic)
# 目的: 避免只用 0~180° 单向转动；90~180° 会改走 -90~0°，降低IK限位失败。
YAW_ALIGN_ENABLE = True    # 偏航对齐总开关 (False=退回固定姿态)
YAW_SIGN = 1.0             # 方向符号 (真机若转反改 -1.0)
YAW_OFFSET = math.pi / 2   # =1.5708, 爪子垂直→需+90°对齐长边；执行前会折叠到±90°内


# ============================================================
# 5.2 偏航角视觉估计参数 (depth_localizer: 彩色bbox内 自适应颜色分割 + minAreaRect 算长边朝向)
# ============================================================
YAW_ESTIMATE = {
    'enable': True,            # 总开关 (False → 输出 yaw=0)
    'center_frac': 0.3,        # bbox中心采样块占比 (取玩具主色 H/S)
    'h_tol': 8,                # 全局回退值; 细调优先看 family_overrides
    's_tol': 60,               # 全局回退值; 细调优先看 family_overrides
    'min_area_frac': 0.05,     # mask最小面积占bbox比例, 低于则判失败→yaw=0
    'use_depth_filter': False, # 深度辅助过滤开关 (默认关, 纯颜色; 不够再开)
    'depth_tol_mm': 40.0,      # 深度辅助: |depth-中心深度|<此值算玩具 (仅开启时)

    # 2026-07-07 实测调参结果（按颜色族复用到 30/45/60 三个尺寸）
    # green_60   -> h=11 s=55
    # caramel_60 -> h=6  s=61
    # pink_45    -> h=5  s=60
    # 大玩具 cylinder 实测 (2026-07-07):
    #   green_cylinder -> h=14 s=113
    #   red_cylinder   -> h=11 s=119
    #   blue_cylinder  -> h=13 s=56
    # green_cylinder 用精确类名(避免与 green_30/45/60 前缀冲突); red/blue 用前缀
    'family_overrides': {
        'green':          {'h_tol': 11, 's_tol': 55},
        'caramel':        {'h_tol': 6,  's_tol': 61},
        'pink':           {'h_tol': 5,  's_tol': 60},
        'green_cylinder': {'h_tol': 14, 's_tol': 113},
        'red':            {'h_tol': 11, 's_tol': 119},
        'blue':           {'h_tol': 13, 's_tol': 56},
    },
}


def get_yaw_estimate_cfg(class_name=None):
    """按类别名返回生效的 yaw 颜色分割参数。
    规则: 先拷贝全局 YAW_ESTIMATE, 再优先精确匹配类名, 最后回退到颜色族前缀。"""
    cfg = dict(YAW_ESTIMATE)
    fam_map = cfg.get('family_overrides', {})
    if class_name:
        cn = str(class_name)
        # 优先级: 精确类名 > 颜色族前缀 (解决 green_cylinder 与 green_30 共用前缀冲突)
        override = fam_map.get(cn) or fam_map.get(cn.split('_', 1)[0])
        if override:
            cfg.update(override)
    return cfg


# ============================================================
# 6. 相机参数
# ============================================================

CAMERA = {
    # Orbbec Gemini 335
    'model': 'Orbbec Gemini 335',
    'resolution': (640, 480),
    'fps': 30,
    'fov_h': 69,        # 水平视场角 (度)
    'fov_v': 43,        # 垂直视场角 (度)
    'depth_min': 0.25,  # 最小深度 (m)
    'depth_max': 5.0,   # 最大深度 (m)

    # ROS2 话题
    'topic_color': '/camera/color/image_raw',
    'topic_depth': '/camera/depth/image_raw',
    'topic_camera_info': '/camera/color/camera_info',

    # 相机坐标系名称
    'frame_optical': 'camera_color_optical_frame',
    'frame_link': 'camera_link',

    # 启动命令 (不要添加 color_format:=BGR!)
    'launch_cmd': 'ros2 launch orbbec_camera gemini_330_series.launch.py',
}


# ============================================================
# 6.1 工程资源/话题/QoS/节点运行参数
# ============================================================
# 这一节专门收口“其他脚本里原本散落的参数”，让 yolo/depth/grasp/tuner 都只从这里取值。
# 以后你改一个参数脚本，就能全局同步。
PACKAGE = {
    'name': 'toy_grasp',
}

RESOURCE_PATHS = {
    # 相对包共享目录的模型路径；运行时通过 get_package_share_directory() 解析，便于整体迁移到 X5
    'yolo_model_relpath': ('models', 'toy_v1', 'best.pt'),

    # 调参工具的测试报告输出目录：这是运行日志目录，不是工程依赖路径
    # 保持放在 /tmp，避免污染工作区，也方便临时测试后直接清理
    'tuner_report_dir': '/tmp/match_logs',
}

ROS_TOPICS = {
    'camera_color': CAMERA['topic_color'],
    'camera_depth': CAMERA['topic_depth'],
    'camera_info': CAMERA['topic_camera_info'],
    'detection_results': '/detection/results',
    'detection_annotated_image': '/detection/annotated_image',
    'detection_objects': '/detection/objects',
    'joint_ctrl_single': '/joint_ctrl_single',
    'joint_states_single': 'joint_states_single',
    'end_pose': 'end_pose',
    'compute_ik': '/compute_ik',
}

QOS_CONFIG = {
    'sensor_depth': 5,      # 图像/相机信息使用 BEST_EFFORT + depth=5
    'default_queue': 10,    # 普通 String / JointState 发布订阅队列
}

YOLO_NODE = {
    'default_conf': 0.5,
    'default_imgsz': 640,
    'perf_log_every_n': 20,

    # 可视化框颜色 (BGR); 按颜色分组: caramel橙棕 / green绿 / pink粉 / red红 / green深绿 / blue蓝
    'class_colors': {
        0: (0, 165, 255), 1: (0, 140, 230), 2: (0, 120, 200),     # caramel 30/45/60
        3: (0, 220, 0),   4: (0, 180, 0),   5: (0, 140, 0),       # green   30/45/60
        6: (230, 180, 255), 7: (203, 150, 255), 8: (180, 120, 240), # pink 30/45/60
        9: (0, 0, 220),  10: (0, 200, 0),  11: (200, 0, 0),        # red/green/blue cylinder
    },
}

DEPTH_LOCALIZER = {
    'min_valid_depth_mm': 50.0,
    'max_valid_depth_mm': 3000.0,
    'center_depth_sample_radius_px': 8,
    # 检测源帧必须匹配到同一时刻的color/depth，不能拿回调时“最新帧”硬配。
    'frame_buffer_sec': 2.0,
    'frame_sync_tolerance_sec': 0.15,
    'sync_warn_throttle_sec': 5.0,
    # 定位结果用于诊断，不应按相机帧率刷磁盘和终端。
    'result_log_throttle_sec': 2.0,
}

VISUAL_GRASP_RUNTIME = {
    # 发布 /joint_ctrl_single 的频率与速度档位
    'cmd_pub_hz': 20.0,
    'smooth_joint_velocity': 25,
    'hold_joint_velocity': 15,

    # HOME / 运动节奏
    'home_wait_sec': 5.0,
    'approach_dist': 0.05,
    'approach_wait_sec': 2.0,
    'smooth_time_sec': 2.5,
    'recover_wait_sec': 1.5,
    'home_ap_offset_deg': [5, 10, -5, 0, 0, 0],
    'home_w1_sec': 2.0,
    # 用户确认抓取回HOME后立即直连释放CSV，不增加固定等待。
    'home_w2_sec': 0.0,
    # 夹爪完成闭合后保留0.8s建压/持物，随后立即执行安全回撤。
    'post_grasp_hold_sec': 0.8,

    # IK 与坐标补偿
    'ik_timeout_sec': 5,
    'correction_x': 0.0,
    'correction_z': 0.0,
    'correction_y_offset': 0.0,
    'enable_distance_correction': False,
    'coarse_z': 0.35,
    'correction_far_k': 0.18,
    'correction_far_kz': 0.10,
}

COLOR_TUNER = {
    'window_full': 'toy_grasp_color_tuner_full',
    'window_roi': 'toy_grasp_color_tuner_roi',
    'track_h': 'h_tol',
    'track_s': 's_tol',
    'full_window_size': (1280, 720),
    'roi_window_size': (1600, 520),
    'refresh_hz': 20.0,
}


def get_package_share_path(*parts):
    """返回 toy_grasp 安装后 share 目录下的资源路径。
    资源路径统一通过包共享目录解析，避免脚本里写死 /home/hanhan 这类机器相关绝对路径。"""
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory(PACKAGE['name']), *parts)


def get_yolo_model_path():
    """返回 YOLO 默认模型绝对路径（由相对资源路径解析得到）。"""
    return get_package_share_path(*RESOURCE_PATHS['yolo_model_relpath'])


# ============================================================
# 7. YOLO 检测参数
# ============================================================

YOLO = {
    # 模型路径
    # 当前使用 YOLOv8n 检测模型 (非分割!)
    'model_path': 'models/toy_v1/best.pt',

    # 类别定义 (玩具12类 = 9类小积木 + 3类大圆柱体)
    'classes': {
        # 小玩具 (长方体积木, 3色 × 3尺寸)
        0: 'caramel_30', 1: 'caramel_45', 2: 'caramel_60',
        3: 'green_30',   4: 'green_45',   5: 'green_60',
        6: 'pink_30',    7: 'pink_45',    8: 'pink_60',
        # 大玩具 (圆柱体, 3色, 高100mm 底面50mm, 放倒)
        9: 'red_cylinder', 10: 'green_cylinder', 11: 'blue_cylinder',
    },

    # 可抓取的类别 (玩具全抓)
    'pickable_classes': [
        # 小玩具
        'caramel_30', 'caramel_45', 'caramel_60',
        'green_30', 'green_45', 'green_60',
        'pink_30', 'pink_45', 'pink_60',
        # 大玩具 (圆柱体)
        'red_cylinder', 'green_cylinder', 'blue_cylinder',
    ],

    # 不可抓取的类别 (无)
    'forbidden_classes': [],

    # 检测置信度阈值
    'confidence': 0.5,
}


# ============================================================
# 7.1 玩具物理参数 (深度定位用)
# ============================================================
# 大玩具: 圆柱体 高100mm 底面直径50mm, 放倒在地面 → half_thickness=0.050m, 夹爪闭合值=0.020
# 小玩具: 长方体积木 宽30×高20mm               → half_thickness=0.015m, 夹爪闭合值=0.006
# 注意: 大玩具放倒后是横向摆放, 视觉看到的表面是圆柱侧面,
#       此处数值不是理论半径, 而是按真机抓取效果标定出的深度补偿值。
OBJECT = {
    # 小玩具物理参数
    'small': {
        'half_thickness': 0.015,   # 半厚补偿 (m), 质心 = 表面深度 + 此值
        'pos_grasp': 0.006,        # 夹爪闭合值 (m)
        'description': '小长方体 30×20mm',
    },
    # 大玩具物理参数 (圆柱体 高100mm 底面50mm 放倒)
    'big': {
        'half_thickness': 0.050,   # 半厚补偿 (m), 质心 = 表面深度 + 此值 (实测补偿, 2026-07-07)
        'pos_grasp': 0.020,        # 夹爪闭合值 (m)
        'description': '大圆柱体 高100mm 直径50mm 放倒',
    },
}

# 按类别名判断玩具大小: 小玩具=small, 大玩具(cylinder)=big
TOY_SIZE_MAP = {
    'caramel_30': 'small', 'caramel_45': 'small', 'caramel_60': 'small',
    'green_30':   'small', 'green_45':   'small', 'green_60':   'small',
    'pink_30':    'small', 'pink_45':    'small', 'pink_60':    'small',
    'red_cylinder':   'big', 'green_cylinder': 'big', 'blue_cylinder': 'big',
}


def get_toy_size(class_name: str | None) -> str:
    """按 YOLO 类别名或已解析大小返回玩具大小: 'small' 或 'big'。

    depth_localizer 输出的是类别名(red_cylinder/green_45等) + toy_status(big/small)。
    visual_grasp 抓取闭合阶段会直接拿 toy_status 查参数，所以这里必须同时支持：
      - 类别名: red_cylinder -> big, green_45 -> small
      - 已解析大小: big -> big, small -> small
    未知值默认 small，保持安全闭合值。
    """
    if class_name is None:
        return 'small'
    name = str(class_name)
    if name in OBJECT:
        return name
    return TOY_SIZE_MAP.get(name, 'small')


def get_toy_params(class_name: str | None) -> dict:
    """按 YOLO 类别名返回该玩具的 {half_thickness, pos_grasp}。"""
    size = get_toy_size(class_name)
    return dict(OBJECT[size])

# 供 YOLO / 深度节点直接读取的类别名 (配置驱动, 加类别只改上面的 YOLO['classes'])
CLASS_NAMES = YOLO['classes']


# ============================================================
# 8. ArUco 标定板参数
# ============================================================

ARUCO = {
    'marker_id': 582,
    'marker_size': 0.0677,      # 边长 (m)
    'corner_refinement': 'LINES',
}


# ============================================================
# 9. 安全参数
# ============================================================

SAFETY = {
    # 急停时保持位置不失能!
    'emergency_stop_hold_position': True,

    # 速度限制 (0~1)，比赛安全上限30%
    'max_speed_ratio': 0.3,

    # 碰撞检测阈值 (rad)
    'collision_threshold': 0.05,
}


# ============================================================
# 工具函数
# ============================================================

def print_config():
    """打印当前配置摘要"""
    print("=" * 60)
    print("嵌赛抓玩具 — 当前参数配置")
    print("=" * 60)
    print(f"\n手眼标定 ({HAND_EYE_CALIB['date']}, {HAND_EYE_CALIB['samples']}次采集):")
    print(f"  位置: [{HAND_EYE_CALIB['position'][0]*1000:+.1f}, "
          f"{HAND_EYE_CALIB['position'][1]*1000:+.1f}, "
          f"{HAND_EYE_CALIB['position'][2]*1000:+.1f}] mm")
    print(f"\nHOME关节(度): {HOME_DEG}")
    print(f"夹爪: 全开 {GRIPPER['pos_open']*1000:.0f}mm, "
          f"默认抓取 {GRIPPER['pos_grasp']*1000:.0f}mm, "
          f"全闭 {GRIPPER['pos_close']*1000:.0f}mm")
    print(f"抓取点偏移 GRIPPER_OFFSET: {GRIPPER_OFFSET}")
    print(f"抓取位姿(四元数): {PICK_POSE['orientation']}")
    print("玩具物理参数:")
    for size, cfg in OBJECT.items():
        print(f"  {size}: half_thickness={cfg['half_thickness']*1000:.0f}mm, "
              f"pos_grasp={cfg['pos_grasp']*1000:.0f}mm")
    print(f"\nYOLO模型: {YOLO['model_path']}")
    print(f"  类别: {YOLO['classes']}")
    print(f"  可抓: {YOLO['pickable_classes']}")
    print("=" * 60)


if __name__ == '__main__':
    print_config()
