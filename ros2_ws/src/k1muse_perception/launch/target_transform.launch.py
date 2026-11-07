import platform
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_target_transform(context):
    """Preserve YAML ownership unless mounting TF is explicitly overridden."""
    parameters = [LaunchConfiguration('params_file')]
    publish = LaunchConfiguration('publish_mount_tf').perform(context)
    if publish != 'auto':
        parameters.append({'publish_mount_tf': publish == 'true'})
    return [Node(prefix='taskset -c 4-7' if platform.machine() == "riscv64" else "", package='k1muse_perception', executable='target_transform',
                 name='target_transform', namespace='', output='screen', parameters=parameters)]


def generate_launch_description():
    """Start C11 with camera and arm mounts on the shared lift stage."""
    config = get_package_share_directory('k1muse_perception') + '/config/target_transform.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=config,
                              description='C11 target transformation YAML.'),
        DeclareLaunchArgument('publish_mount_tf', default_value='auto',
                              choices=['auto', 'true', 'false'],
                              description='Use YAML or override mounting TF publication.'),
        OpaqueFunction(function=launch_target_transform),
    ])
