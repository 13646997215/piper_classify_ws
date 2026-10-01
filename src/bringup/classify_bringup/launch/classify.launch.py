"""Launch the entire simulation; no motion until /classify/start by default."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def include(package, filename, **args):
    path=Path(get_package_share_directory(package))/'launch'/filename
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)),launch_arguments=args.items())

def generate_launch_description():
    description=Path(get_package_share_directory('classify_description'))
    return LaunchDescription([
        DeclareLaunchArgument('gui',default_value='true'),
        DeclareLaunchArgument('rviz',default_value='true'),
        DeclareLaunchArgument('auto_start',default_value='false'),
        DeclareLaunchArgument('plan_only',default_value='false'),
        DeclareLaunchArgument('world',default_value=str(description/'worlds/classify.world')),
        DeclareLaunchArgument('depth_noise_stddev_m',default_value='0.0'),
        DeclareLaunchArgument('depth_dropout_probability',default_value='0.0'),
        DeclareLaunchArgument('stress_seed',default_value='0'),
        include('classify_description','gazebo.launch.py',gui=LaunchConfiguration('gui'),use_rviz='false',
                world=LaunchConfiguration('world'),depth_noise_stddev_m=LaunchConfiguration('depth_noise_stddev_m'),
                depth_dropout_probability=LaunchConfiguration('depth_dropout_probability'),stress_seed=LaunchConfiguration('stress_seed')),
        include('classify_moveit_config','move_group.launch.py',use_rviz=LaunchConfiguration('rviz'),world=LaunchConfiguration('world')),
        include('classify_perception','perception.launch.py'),
        include('classify_task','task.launch.py',world=LaunchConfiguration('world'),auto_start=LaunchConfiguration('auto_start'),plan_only=LaunchConfiguration('plan_only')),
    ])
