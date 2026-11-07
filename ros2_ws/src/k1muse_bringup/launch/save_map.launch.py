"""Save the current /map once using the upstream map saver."""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def save_map(context):
    """Create the output directory and pass an explicit basename to Nav2."""
    maps_dir = LaunchConfiguration('maps_dir').perform(context)
    map_name = LaunchConfiguration('map_name').perform(context)
    if not os.path.isabs(maps_dir):
        raise ValueError('maps_dir must be an absolute path')
    if (not map_name or map_name in ('.', '..') or
            os.path.basename(map_name) != map_name or
            map_name.lower().endswith(('.yaml', '.pgm'))):
        raise ValueError('map_name must be a basename without .yaml/.pgm suffix')
    os.makedirs(maps_dir, exist_ok=True)
    return [Node(
        package='nav2_map_server',
        executable='map_saver_cli',
        arguments=['-t', '/map', '-f', os.path.join(maps_dir, map_name), '--fmt', 'pgm'],
        parameters=[{'map_subscribe_transient_local': True, 'save_map_timeout': 5.0}],
        output='screen',
    )]


def generate_launch_description():
    """Require an explicit map name; save in /home/bianbu/maps by default."""
    return LaunchDescription([
        DeclareLaunchArgument('maps_dir', default_value='/home/bianbu/maps'),
        DeclareLaunchArgument('map_name', description='Map basename without a file suffix.'),
        OpaqueFunction(function=save_map),
    ])
