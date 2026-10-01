"""
================================================================================
gazebo.launch.py — 在 Gazebo Classic 中启动 Piper + 分类场景 + ros2_control
================================================================================

节点与启动顺序
  1. gazebo (server + 可选 gui)   加载 classify.world
  2. robot_state_publisher        发布 URDF 与 /tf
  3. spawn_entity.py              从 /robot_description 话题读 URDF 生成模型
                                  -> Gazebo 加载模型时 gazebo_ros2_control 插件
                                     随之创建 /controller_manager
  4. controller_manager/spawner x3  依次激活三个控制器
                                  用 OnProcessExit 挂在 spawn_entity 之后，
                                  保证 controller_manager 已就绪

为什么用 controller_manager/spawner 节点，而不是 ros2 control load_controller
  spawner 是长驻节点，能持续检测 controller_manager 是否可用并在超时后报错；
  ros2 control load_controller 是一次性子进程，失败就静默退出，很难排查。

为什么 spawn_entity 用 -topic robot_description 而不是命令行传 URDF
  避免在两处重复维护同一份 URDF；rsp 展开一次，spawner 从话题取，天然一致。

关于 URDF 文件
  直接使用 urdf/piper_sim.urdf，它是预先展开、剥离了所有 XML 注释、
  网格路径为 file:// 绝对路径的版本。原因见 xacro 文件顶部说明:
    1. gazebo_ros2_control 把整个 URDF 拼成单个命令行参数交给
       rcl_parse_arguments，后者按 YAML 解析，注释里的「冒号+空格」
       会被当成键值对导致解析失败
    2. Gazebo Classic 不支持 package://，只认 file:// 与 model://
  修改 xacro 后请重新执行: bash scripts/tools/build_urdf.sh

关于机械臂姿态
  spawn yaw=0，HOME 朝世界 +X；桌子和工件整体绕世界 Z 轴转 -90 度。

用法
  export ROS_DOMAIN_ID=42
  ros2 launch classify_description gazebo.launch.py
  # 省内存（本机可用内存约 5 GB）
  ros2 launch classify_description gazebo.launch.py gui:=false use_rviz:=false
================================================================================
"""
import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('classify_description')
    gazebo_share = get_package_share_directory('gazebo_ros')
    with open(os.path.join(pkg_share,'config','gemini335.yaml')) as stream:
        sensor_profile = yaml.safe_load(stream)
    rgb, depth = sensor_profile['color'], sensor_profile['depth']
    if any(abs(value) > 1e-12 for value in rgb['distortion']):
        raise ValueError('Nonzero device distortion needs a matching render model; do not change CameraInfo alone')
    if (rgb['width'], rgb['height']) != (depth['width'], depth['height']):
        raise ValueError('Current registration profile requires equal native RGB/depth dimensions')
    registration = Node(package='classify_camera_sim', executable='align_rgbd', output='screen', parameters=[{
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'color_intrinsics': [rgb[k] for k in ('fx','fy','cx','cy')],
        'depth_intrinsics': [depth[k] for k in ('fx','fy','cx','cy')],
        'color_origin_in_depth': depth['color_origin_in_depth_optical_m'],
        'min_z': depth['min_z_m'], 'max_z': depth['far_m'],
        'stereo_baseline': depth['stereo_baseline_m'],
        'width': rgb['width'], 'height': rgb['height'],
        'depth_noise_stddev_m': ParameterValue(LaunchConfiguration('depth_noise_stddev_m'), value_type=float),
        'depth_dropout_probability': ParameterValue(LaunchConfiguration('depth_dropout_probability'), value_type=float),
        'stress_seed': ParameterValue(LaunchConfiguration('stress_seed'), value_type=int),
    }])

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    gui = LaunchConfiguration('gui')
    verbose = LaunchConfiguration('verbose')
    world = LaunchConfiguration('world')

    # ------------------------------------------------------------------
    # URDF 内容：统一用 piper_sim.urdf（file:// 绝对路径）
    #
    # 关键背景: Gazebo Classic 完全不认 package://
    #   strings libgazebo_common.so.11 | grep -c "package://"  ->  0
    #   它只认 file:// 和 model://。
    #
    # 本文件里 rsp 和 spawn_entity 共享同一份 robot_description：
    #   rsp            -> 发布到 /robot_description 话题
    #   spawn_entity   -> 用 -topic robot_description 从该话题取 URDF
    # 所以两者拿到的必然是同一份 URDF，网格路径也必然一致。
    # 若这里改成 package:// 版本，Gazebo 里的 mesh 就会全部静默加载失败，
    # 表现为"模型在列表里但看不见"。
    #
    # RViz 同样认 file:// 绝对路径，因此三个节点统一用这一份即可。
    #
    # 必须用 ParameterValue(..., value_type=str) 显式声明为字符串，
    # 否则 launch 会把 URDF 文本当 YAML 解析并报
    #     "Unable to parse the value of parameter robot_description as yaml."
    # ------------------------------------------------------------------
    with open(os.path.join(pkg_share, 'urdf', 'piper_sim.urdf')) as f:
        robot_description = {'robot_description': ParameterValue(f.read(), value_type=str)}

    # ------------------------------------------------------------------
    # 1. Gazebo
    # ------------------------------------------------------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': world,
            'gui': gui,
            'verbose': verbose,
        }.items(),
    )

    # 本项目全部使用本地 primitive 模型，不访问在线模型数据库
    disable_model_database = SetEnvironmentVariable(
        name='GAZEBO_MODEL_DATABASE_URI', value='')

    # ------------------------------------------------------------------
    # 2. robot_state_publisher
    # ------------------------------------------------------------------
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    # ------------------------------------------------------------------
    # 3. 生成模型
    # ------------------------------------------------------------------
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_piper',
        output='screen',
        arguments=[
            '-entity', 'piper',
            '-topic', 'robot_description',
            #
            # 位置: URDF 的 world_to_base 固定关节把基座锚定在 z=0.347。
            #       因此模型本身在 Gazebo 世界原点生成，HOME 朝世界 +X。
            #
            # z 用 0.347 而不是 0.35 的原因:
            #   base_link 的碰撞盒在 base_link 系里的 z 范围是 [0.003, 0.083]
            #   （box 中心 0.043，尺寸 0.08，再内缩 3mm）。
            #   若把 base_link 原点放在 z = 0.35，碰撞盒底面就在 0.353，
            #   比桌面高 3mm，视觉上表现为"悬空"，物理上也接触不到桌面。
            #   固定关节的 z=0.347 使碰撞盒底面正好落在 z=0.350。
            #
            # 朝向说明 —— 为什么机械臂不旋转、而是转桌子:
            #   任务布局可以用两种等价方式得到:
            #     (a) 机械臂绕基座 Z 转 +90 度（spawn 的 -Y 1.5708）
            #     (b) 场景（桌子/物块/料盒）绕世界原点顺时针转 90 度
            #   两者相对几何相同，但 (a) 会让 base_link 的坐标轴不再与世界轴对齐，
            #   后面 MoveIt 规划要反复用到基座系，(b) 则保持基座系与世界系一致，
            #   推理成本低很多。选 (b)，机械臂 yaw 固定为 0。
            #   场景旋转由 scripts/tools/rotate_scene.py 生成，不依赖 spawn 参数。
            #
            # 基座放在 y=0 而不是靠桌边 y=-0.05 的原因:
            #   场景是绕【世界原点】旋转的，不是绕机械臂基座。
            #   若基座偏在 y=-0.05，物块到基座的距离会从 0.300 变成 0.350
            #   （多 50mm，料盒从 0.474 变成 0.519），等于把工作区整体推远。
            #   基座回到 y=0 后，五个工作点的水平距离为
            #   0.300 / 0.331 / 0.331 / 0.474 / 0.474 m。
            #   旧数值 IK 校验有坐标和姿态误差错误，不能作为可执行轨迹证据。
            '-x', '0.0', '-y', '0.0', '-z', '0.0',
            '-R', '0.0', '-P', '0.0', '-Y', '0.0',
        ],
    )

    # ------------------------------------------------------------------
    # 4. 三个控制器 spawner
    #    --controller-manager-timeout 60 : 给 Gazebo 最多 60 秒启动物理与插件
    # ------------------------------------------------------------------
    def spawner(controller):
        return Node(
            package='controller_manager',
            executable='spawner',
            name=f'spawner_{controller}',
            output='screen',
            arguments=[
                controller,
                '--controller-manager', '/controller_manager',
                '--controller-manager-timeout', '60',
            ],
        )

    spawn_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[
                spawner('piper_joint_state_broadcaster'),
                spawner('piper_arm_controller'),
                spawner('piper_gripper_controller'),
            ],
        )
    )

    # ------------------------------------------------------------------
    # 5. RViz（可选）
    # ------------------------------------------------------------------
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        condition=IfCondition(use_rviz),
        arguments=['-d', os.path.join(pkg_share, 'rviz', 'piper.rviz')],
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('depth_noise_stddev_m', default_value='0.0'),
        DeclareLaunchArgument('depth_dropout_probability', default_value='0.0'),
        DeclareLaunchArgument('stress_seed', default_value='0'),
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='使用 Gazebo 仿真时钟'),
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='是否启动 RViz'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='是否启动 Gazebo 图形界面（关掉省内存）'),
        DeclareLaunchArgument('verbose', default_value='false',
                              description='Gazebo 服务器详细日志'),
        DeclareLaunchArgument('world',
                              default_value=os.path.join(pkg_share, 'worlds',
                                                         'classify.world'),
                              description='Gazebo world 文件绝对路径'),

        disable_model_database,
        gazebo,
        registration,
        robot_state_publisher,
        spawn_robot,
        spawn_controllers,
        rviz,
    ])
