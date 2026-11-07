import platform
"""Start the official RealSense driver with the C08 RGB-only defaults."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Publish RGB8 640x480 at a requested 30 FPS by default."""
    config = os.path.join(
        get_package_share_directory('k1muse_bringup'), 'config', 'camera.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=config),
        Node(prefix='taskset -c 5,6' if platform.machine() == "riscv64" else "",
            package='realsense2_camera',
            executable='realsense2_camera_node',
            namespace='k1muse',
            name='front_camera',
            parameters=[LaunchConfiguration('params_file')],
            output='screen',
        ),
    ])
