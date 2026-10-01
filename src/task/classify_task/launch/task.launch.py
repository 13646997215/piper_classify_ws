"""Task node for an already running simulation + MoveIt + perception."""
from pathlib import Path
import importlib.util
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder

def setup(context):
    description = Path(get_package_share_directory('classify_description'))
    task = Path(get_package_share_directory('classify_task'))
    config = (MoveItConfigsBuilder('piper', package_name='classify_moveit_config')
        .robot_description(file_path=str(description/'urdf/piper_sim.urdf'))
        .planning_pipelines(pipelines=['ompl'], default_planning_pipeline='ompl')
        .trajectory_execution(moveit_manage_controllers=False).to_moveit_configs())
    module_path=Path(get_package_share_directory('classify_moveit_config'))/'launch/workcell_config.py'
    spec=importlib.util.spec_from_file_location('classify_task_inventory',module_path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    inventory={k:v for k,v in module.parameters(description,LaunchConfiguration('world').perform(context)).items()
               if k.startswith('expected_')}
    return [
        Node(package='classify_task', executable='classify_task_node', output='screen',
            parameters=[config.to_dict(), str(task/'config/task.yaml'), inventory, {
                'use_sim_time': True,
                'auto_start': ParameterValue(LaunchConfiguration('auto_start'), value_type=bool),
                'plan_only': ParameterValue(LaunchConfiguration('plan_only'), value_type=bool),
            }])
    ]


def generate_launch_description():
    description=Path(get_package_share_directory('classify_description'))
    return LaunchDescription([
        DeclareLaunchArgument('auto_start',default_value='false'),
        DeclareLaunchArgument('plan_only',default_value='false'),
        DeclareLaunchArgument('world',default_value=str(description/'worlds/classify.world')),
        OpaqueFunction(function=setup),
    ])
