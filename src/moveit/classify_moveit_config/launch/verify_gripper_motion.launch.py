"""Plan gripper opening; optionally execute opening and closing in Gazebo."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    urdf = Path(get_package_share_directory('classify_description')) / 'urdf' / 'piper_sim.urdf'
    config = (
        MoveItConfigsBuilder('piper', package_name='classify_moveit_config')
        .robot_description(file_path=str(urdf))
        .planning_pipelines(pipelines=['ompl'], default_planning_pipeline='ompl')
        .trajectory_execution(moveit_manage_controllers=False)
        .to_moveit_configs()
    )
    return LaunchDescription([
        DeclareLaunchArgument('execute', default_value='false'),
        Node(
            package='classify_moveit_config',
            executable='verify_gripper_motion',
            output='screen',
            parameters=[config.to_dict(), {
                'use_sim_time': True,
                'execute': ParameterValue(LaunchConfiguration('execute'), value_type=bool),
            }],
        ),
    ])
