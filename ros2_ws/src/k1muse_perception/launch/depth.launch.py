import platform
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.parameter_descriptions import ParameterValue


def launch_depth(context):
    """Select one depth method while leaving YAML authoritative in auto mode."""
    parameters = [LaunchConfiguration('params_file'), {'input_mode': ParameterValue(LaunchConfiguration('input_mode'), value_type=str)}]
    algorithm = LaunchConfiguration('algorithm').perform(context)
    if algorithm != 'auto':
        parameters.append({'algorithm': algorithm})
    node = LifecycleNode(prefix='taskset -c 5,6' if platform.machine() == "riscv64" else "",
        package='k1muse_perception', executable='depth_fusion',
        name='depth_fusion', namespace='', output='screen', parameters=parameters)
    return [node]


def generate_launch_description():
    """Start C10; its executable sequences Lifecycle configure and activate."""
    config = get_package_share_directory('k1muse_perception') + '/config/depth.yaml'
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=config,
                              description='C10 depth parameter YAML.'),
        DeclareLaunchArgument('input_mode', default_value='both', choices=['off', 'marker', 'both']),
        DeclareLaunchArgument('algorithm', default_value='auto',
                              choices=['auto', 'roi_median', 'center_mean', 'yolo_weighted'],
                              description='Use YAML algorithm or override it at launch.'),
        OpaqueFunction(function=launch_depth),
    ])
