"""Launch the natural-language agent for the running classification demo."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_llm', default_value='false'),
        DeclareLaunchArgument('use_voice', default_value='false'),
        DeclareLaunchArgument('enable_tts', default_value='false'),
        DeclareLaunchArgument('llm_model', default_value='deepseek-chat'),
        DeclareLaunchArgument('llm_base_url', default_value='https://api.deepseek.com'),
        DeclareLaunchArgument('llm_api_key_env', default_value='DEEPSEEK_API_KEY'),
        DeclareLaunchArgument('command', default_value='',
                              description='Run one command and exit; empty = interactive'),
        Node(
            package='classify_agent',
            executable='agent_node',
            name='classify_language_agent',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'use_llm': ParameterValue(LaunchConfiguration('use_llm'),
                                          value_type=bool),
                'use_voice': ParameterValue(LaunchConfiguration('use_voice'),
                                            value_type=bool),
                'enable_tts': ParameterValue(LaunchConfiguration('enable_tts'),
                                             value_type=bool),
                'llm_model': LaunchConfiguration('llm_model'),
                'llm_base_url': LaunchConfiguration('llm_base_url'),
                'llm_api_key_env': LaunchConfiguration('llm_api_key_env'),
                'command': LaunchConfiguration('command'),
            }],
        ),
    ])
