import platform
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Start disabled CV controls; task_manager enables scanning at task boundaries."""
    default_config = get_package_share_directory('k1muse_perception') + '/config/cv.yaml'
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_config,
            description='CV RGB topics, source freshness, scan rate and CPU core.'),
        Node(prefix='taskset -c 4' if platform.machine() == "riscv64" else "",
            package='k1muse_perception',
            executable='cv_perception',
            name='cv_perception',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
