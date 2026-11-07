import platform
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def operator(context):
    configs = LaunchConfiguration('config_dir').perform(context)
    voice = LaunchConfiguration('voice_config_dir').perform(context) or configs
    parameters = [LaunchConfiguration('config'), {'config_dir': configs, 'voice_config_dir': voice}]
    catalog = LaunchConfiguration('drug_catalog_file').perform(context)
    if catalog:
        parameters.append({'drug_catalog_file': catalog})
    return [Node(prefix='taskset -c 4-7' if platform.machine() == "riscv64" else "", package='k1muse_web', executable='web_operator', name='web_operator',
                 parameters=parameters, output='screen')]


def generate_launch_description():
    share = get_package_share_directory('k1muse_web')
    config = LaunchConfiguration('config')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=os.path.join(share, 'config', 'web.yaml')),
        DeclareLaunchArgument('http_port', default_value='8080'),
        DeclareLaunchArgument('config_dir', default_value='/home/bianbu/k1muse_config'),
        DeclareLaunchArgument('voice_config_dir', default_value=''),
        DeclareLaunchArgument('drug_catalog_file', default_value=''),
        OpaqueFunction(function=operator),
        Node(prefix='taskset -c 4-7' if platform.machine() == "riscv64" else "", package='foxglove_bridge', executable='foxglove_bridge',
             parameters=[config], output='screen'),
        ExecuteProcess(prefix='taskset -c 4-7' if platform.machine() == "riscv64" else "", cmd=['python3', '-m', 'http.server', LaunchConfiguration('http_port'),
                            '--bind', '0.0.0.0', '--directory', os.path.join(share, 'web')],
                       output='screen'),
    ])
