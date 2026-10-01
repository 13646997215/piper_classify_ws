from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    auto_enable = LaunchConfiguration('piper_auto_enable')
    return LaunchDescription([
        DeclareLaunchArgument('piper_auto_enable', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('orbbec_camera'), 'launch',
                'gemini_330_series.launch.py'])),
            launch_arguments={
                'camera_name': 'camera', 'depth_registration': 'true',
                'enable_color': 'true', 'enable_depth': 'true',
                'color_width': '640', 'color_height': '480',
                'color_fps': '15', 'color_format': 'MJPG',
                'depth_width': '640', 'depth_height': '480',
                'depth_fps': '15',
            }.items()),
        Node(package='piper', executable='piper_single_ctrl',
             name='piper_driver', output='screen',
             parameters=[{
                 'can_port': 'can1',
                 'auto_enable': ParameterValue(auto_enable, value_type=bool),
                 'auto_enable_timeout_sec': 30.0,
                 'feedback_publish_hz': 20.0,
                 'gripper_exist': True, 'gripper_val_mutiple': 1,
             }],
             on_exit=Shutdown(reason='Piper driver exited; pick test cannot continue safely')),
        Node(package='joint_state_mapper', executable='joint_state_mapper_node',
             name='joint_state_mapper', output='screen'),
    ])
