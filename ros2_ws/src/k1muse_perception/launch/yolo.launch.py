from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.parameter_descriptions import ParameterValue


def launch_detector(context):
    """Apply an explicit tracking override while leaving YAML authoritative in auto mode."""
    parameters = [LaunchConfiguration('params_file')]
    use_tracker = LaunchConfiguration('use_tracker').perform(context)
    if use_tracker != 'auto':
        parameters.append({'use_tracker': use_tracker == 'true'})
    parameters.append({'enable_perf_stats': ParameterValue(
        LaunchConfiguration('enable_perf_stats'), value_type=bool)})
    detector = LifecycleNode(
        package='k1muse_perception',
        executable='yolo_detector',
        name='yolo_detector',
        namespace='',
        output='screen',
        parameters=parameters,
    )
    return [detector]


def generate_launch_description():
    """Start C08 controls; manager must grant model preparation and inference."""
    default_config = get_package_share_directory('k1muse_perception') + '/config/yolo.yaml'
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_config,
            description='C08 YOLO parameter YAML including deployed model and labels paths.'),
        DeclareLaunchArgument(
            'use_tracker', default_value='auto', choices=['auto', 'true', 'false'],
            description='Use YAML tracking setting (auto), or override it at launch.'),
        DeclareLaunchArgument(
            'enable_perf_stats', default_value='false', choices=['true', 'false'],
            description='Measure processing stages and report every ~5s while processing images.'),
        OpaqueFunction(function=launch_detector),
    ])
