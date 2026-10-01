"""
================================================================================
display.launch.py — 纯可视化：在 RViz 里显示 Piper 机械臂
================================================================================

本阶段（S1）不接 ros2_control、不接 Gazebo，只验证机器人描述本身是否正确：
  · 网格能否正常加载
  · 关节方向 / 限位是否与官方一致（拖动滑块观察）
  · 连杆是否连贯、没有错位

节点构成
  robot_state_publisher     读 URDF，订阅 /joint_states，发布 /tf、/tf_static，
                            并在 /robot_description 话题上发布 URDF
  joint_state_publisher_gui 8 个关节（含两个夹爪）的滑块
  rviz2                     显示 RobotModel

用法
  export ROS_DOMAIN_ID=42
  ros2 launch classify_description display.launch.py
================================================================================
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('classify_description')

    # ------------------------------------------------------------------
    # 读取 URDF 的**文件内容**，而不是文件路径
    #
    #   robot_state_publisher 的 robot_description 参数值必须是 URDF 的 XML 文本。
    #   如果传的是路径字符串，它会拿这个路径去当 XML 解析，得到
    #       Error: Error document empty.
    #       Failed to parse robot description using: urdf_xml_parser/URDFXMLParser
    #   然后抛出未捕获异常，进程直接 abort（exit code -6）。
    #   rsp 挂掉 → /robot_description 没人发布 → 滑块窗口空白、RViz 退出。
    #
    #   若模型是 .xacro，正确写法是用 Command(['xacro ', path]) 让 xacro 先展开。
    # ------------------------------------------------------------------
    urdf_path = os.path.join(pkg_share, 'urdf', 'piper.urdf')
    with open(urdf_path) as f:
        robot_description = {'robot_description': f.read()}

    use_rviz = LaunchConfiguration('use_rviz')
    use_jsp_gui = LaunchConfiguration('use_joint_state_publisher_gui')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='是否启动 RViz'),
        DeclareLaunchArgument(
            'use_joint_state_publisher_gui', default_value='true',
            description='是否启动关节滑块（可拖动观察各关节运动范围）'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(pkg_share, 'rviz', 'piper.rviz'),
            description='RViz 配置文件'),

        # ------------------------------------------------------------------
        # robot_state_publisher：URDF -> /tf，并在 /robot_description 话题发布
        # ------------------------------------------------------------------
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
        ),

        # ------------------------------------------------------------------
        # 关节滑块：8 个关节
        #   joint1..joint6  手臂   旋转
        #   joint7          左指    0 ~  0.035
        #   joint8          右指   -0.035 ~ 0    （与 joint7 镜像，同开同合）
        #
        # 显式传入 robot_description，保证滑块稳定生成。
        # 不传时它依赖订阅 /robot_description 话题；启动顺序不当就会拿到空描述，
        # 表现为窗口里只有 Randomize / Center 两个按钮、没有滑块。
        # ------------------------------------------------------------------
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            condition=IfCondition(use_jsp_gui),
            parameters=[robot_description],
        ),

        # ------------------------------------------------------------------
        # RViz
        # ------------------------------------------------------------------
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(use_rviz),
            arguments=['-d', LaunchConfiguration('rviz_config')],
        ),
    ])
