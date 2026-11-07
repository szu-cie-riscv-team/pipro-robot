from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    """Start C15 with the shared canonical drug names and explicit aliases."""
    share = get_package_share_directory('k1muse_manager')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=share + '/config/voice_command.yaml'),
        DeclareLaunchArgument('config_dir', default_value='/home/bianbu/k1muse_config'),
        DeclareLaunchArgument('voice_config_dir', default_value=LaunchConfiguration('config_dir')),
        *[DeclareLaunchArgument(argument, default_value=PathJoinSubstitution([
            LaunchConfiguration('voice_config_dir'), filename])) for argument, filename in [
                ('intent_config_file', 'voice_intent.yaml'), ('cloud_config_file', 'cloud_api.yaml'),
                ('cloud_prompt_file', 'cloud_prompt.yaml'), ('local_config_file', 'local_llm.yaml'),
                ('local_prompt_file', 'local_prompt.yaml')]],
        Node(package='k1muse_manager', executable='voice_command', name='voice_command',
             output='screen', additional_env={'SPACEMIT_PERFER_CORE_ID': '0,1,2,3'},
             parameters=[LaunchConfiguration('params_file'),
                                         {'intent_config_file': LaunchConfiguration('intent_config_file'),
                                          'cloud_config_file': LaunchConfiguration('cloud_config_file'),
                                          'cloud_prompt_file': LaunchConfiguration('cloud_prompt_file'),
                                          'local_config_file': LaunchConfiguration('local_config_file'),
                                          'local_prompt_file': LaunchConfiguration('local_prompt_file')}]),
    ])
