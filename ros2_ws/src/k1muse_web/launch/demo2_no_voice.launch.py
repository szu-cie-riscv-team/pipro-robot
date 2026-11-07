"""Compatibility entry: run the shared Demo2 startup without voice."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(Path(get_package_share_directory('k1muse_web')) /
                                          'launch' / 'demo2.launch.py')),
        launch_arguments={'enable_voice': 'false', 'voice_backend': 'config'}.items(),
    )])
