import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Start online 2D mapping with the existing C01-C03 data flow."""
    bringup_share = get_package_share_directory('k1muse_bringup')
    slam_toolbox_share = get_package_share_directory('slam_toolbox')
    visualize = LaunchConfiguration('visualize')
    use_sim_time = LaunchConfiguration('use_sim_time')

    upstream_parameters = os.path.join(
        slam_toolbox_share, 'config', 'mapper_params_online_async.yaml')
    k1muse_parameters = os.path.join(
        bringup_share, 'config', 'slam_mapping.yaml')
    rviz_config = os.path.join(bringup_share, 'rviz', 'slam_mapping.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock instead of K1 wall time.',
        ),
        DeclareLaunchArgument(
            'visualize',
            default_value='false',
            description='Start RViz with the C04 map, scan, and TF view.',
        ),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                upstream_parameters,
                k1muse_parameters,
                {'use_sim_time': use_sim_time},
            ],
        ),
        Node(
            condition=IfCondition(visualize),
            package='rviz2',
            executable='rviz2',
            name='slam_mapping_rviz',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
