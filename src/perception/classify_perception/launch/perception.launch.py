"""RGB-D localization; simulation and real camera use identical topic names."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share=Path(get_package_share_directory('classify_description'))
    workcell=yaml.safe_load((share/'config/workcell.yaml').read_text())
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',default_value='true'),
        DeclareLaunchArgument('reference_frame',default_value='base_link'),
        DeclareLaunchArgument('optical_frame',default_value='camera_color_optical_frame'),
        DeclareLaunchArgument('sync_tolerance',default_value='0.0',description='0 exact; real RGB-D may require bounded approximate sync'),
        Node(package='classify_perception', executable='detect_blocks', name='detect_blocks', output='screen',
             parameters=[workcell, {
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                 'reference_frame': LaunchConfiguration('reference_frame'),
                 'optical_frame': LaunchConfiguration('optical_frame'),
                 'sync_tolerance': ParameterValue(LaunchConfiguration('sync_tolerance'), value_type=float),
             }]),
    ])
