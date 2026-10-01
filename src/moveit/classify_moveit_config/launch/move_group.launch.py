"""MoveIt server for the already-running classify_description Gazebo world."""
from pathlib import Path
import importlib.util

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def setup(context):
    description = Path(get_package_share_directory('classify_description'))
    moveit_share = Path(get_package_share_directory('classify_moveit_config'))
    spec = importlib.util.spec_from_file_location('classify_workcell', Path(__file__).with_name('workcell_config.py'))
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    workcell = module.parameters(description, LaunchConfiguration('world').perform(context))
    urdf = description / 'urdf' / 'piper_sim.urdf'
    config = (
        MoveItConfigsBuilder('piper', package_name='classify_moveit_config')
        .robot_description(file_path=str(urdf))
        .planning_pipelines(pipelines=['ompl'], default_planning_pipeline='ompl')
        .trajectory_execution(moveit_manage_controllers=False)
        .to_moveit_configs()
    )
    params = config.to_dict()
    params.update({
        'use_sim_time': True,
        'allow_trajectory_execution': True,
        'publish_robot_description_semantic': True,
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.5,
        'trajectory_execution.allowed_goal_duration_margin': 1.0,
    })
    # RViz is a client of move_group's monitored scene. Keep server-only
    # publishing/execution parameters off the RViz node to avoid a second
    # publisher on monitored_planning_scene.
    rviz_params = [
        config.robot_description,
        config.robot_description_semantic,
        config.robot_description_kinematics,
        config.joint_limits,
        config.planning_pipelines,
        {'use_sim_time': True},
    ]
    return [
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            name='move_group',
            output='screen',
            parameters=[params],
        ),
        Node(
            package='classify_scene_bridge',
            executable='scene_bridge',
            output='screen',
            parameters=[workcell, {'use_sim_time': True}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='moveit_rviz',
            output='log',
            arguments=['-d', str(moveit_share / 'config' / 'moveit.rviz')],
            parameters=rviz_params,
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ]


def generate_launch_description():
    description=Path(get_package_share_directory('classify_description'))
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='false', description='Open the MoveIt MotionPlanning panel'),
        DeclareLaunchArgument('world', default_value=str(description/'worlds/classify.world')),
        OpaqueFunction(function=setup),
    ])
