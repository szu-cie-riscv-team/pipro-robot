import platform
from launch import LaunchDescription
import math
from pathlib import Path

from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _config_path(context, name):
    path = LaunchConfiguration(name).perform(context)
    if not Path(path).is_file():
        raise ValueError(f'{name} file does not exist: {path}')
    return path


def _mount_vector(context, name):
    try:
        values = [float(value) for value in LaunchConfiguration(name).perform(context).split()]
    except ValueError as error:
        raise ValueError(f'{name} requires three finite numbers') from error
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        raise ValueError(f'{name} requires three finite numbers')
    return values


def _launch_nodes(context):
    use_ekf = LaunchConfiguration('use_ekf').perform(context) == 'true'
    odometry_config = _config_path(context, 'odometry_config')
    nodes = [
        Node(prefix='taskset -c 6,7' if platform.machine() == "riscv64" else "",
            package='k1muse_communication',
            executable='stm32_communication',
            name='stm32_communication',
            parameters=[{
                'serial_device': LaunchConfiguration('serial_device'),
                'cmd_vel_timeout_ms': LaunchConfiguration('cmd_vel_timeout_ms'),
                'reconnect_period_ms': LaunchConfiguration('reconnect_period_ms'),
            }],
            output='screen',
        ),
        Node(prefix='taskset -c 6,7' if platform.machine() == "riscv64" else "",
            package='k1muse_communication',
            executable='chassis_odometry',
            name='chassis_odometry',
            parameters=[odometry_config, {
                'publish_tf': not use_ekf,
                'odom_frame_id': 'wheel_odom' if use_ekf else 'odom',
            }],
            remappings=[('/odom', '/odom/raw')] if use_ekf else [],
            output='screen',
        ),
    ]
    if not use_ekf:
        return nodes

    # Evaluate IMU/EKF-only settings here so the raw branch needs neither dependency.
    imu_config = _config_path(context, 'imu_config')
    ekf_config = _config_path(context, 'ekf_config')
    xyz = _mount_vector(context, 'imu_xyz')
    rpy = _mount_vector(context, 'imu_rpy')
    if rpy[0] != 0.0 or rpy[1] != 0.0:
        raise ValueError('C02 requires a horizontal IMU with +Z up (roll=pitch=0)')
    nodes.extend([
        Node(prefix='taskset -c 6,7' if platform.machine() == "riscv64" else "",
            package='imu_ros2_device',
            executable='ybimu_driver',
            name='ybimu_node',
            parameters=[imu_config, {
                'port': LaunchConfiguration('imu_device'),
                'frame_id': 'imu_link',
            }],
            output='screen',
        ),
        Node(prefix='taskset -c 6,7' if platform.machine() == "riscv64" else "",
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_mount_tf',
            arguments=[
                '--x', str(xyz[0]), '--y', str(xyz[1]), '--z', str(xyz[2]),
                '--roll', str(rpy[0]), '--pitch', str(rpy[1]), '--yaw', str(rpy[2]),
                '--frame-id', 'base_link', '--child-frame-id', 'imu_link',
            ],
            output='screen',
        ),
        Node(prefix='taskset -c 6,7' if platform.machine() == "riscv64" else "",
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[ekf_config, {
                'two_d_mode': True,
                'publish_tf': True,
                'odom_frame': 'odom',
                'base_link_frame': 'base_link',
                'world_frame': 'odom',
                'odom0': '/odom/raw',
                'imu0': '/imu/data',
            }],
            remappings=[('odometry/filtered', '/odom')],
            output='screen',
        ),
    ])
    return nodes


def generate_launch_description():
    config_dir = PathJoinSubstitution([FindPackageShare('k1muse_bringup'), 'config'])
    return LaunchDescription([
        DeclareLaunchArgument('serial_device'),
        DeclareLaunchArgument('cmd_vel_timeout_ms', default_value='200'),
        DeclareLaunchArgument('reconnect_period_ms', default_value='1000'),
        DeclareLaunchArgument('use_ekf', default_value='false', choices=['false', 'true']),
        DeclareLaunchArgument('imu_device', default_value='/dev/myimu'),
        DeclareLaunchArgument(
            'imu_config', default_value=PathJoinSubstitution([config_dir, 'imu.yaml'])),
        DeclareLaunchArgument(
            'odometry_config',
            default_value=PathJoinSubstitution([config_dir, 'odometry.yaml'])),
        DeclareLaunchArgument(
            'ekf_config', default_value=PathJoinSubstitution([config_dir, 'ekf.yaml'])),
        DeclareLaunchArgument(
            'imu_xyz', default_value='', description='Required with EKF: x y z in metres'),
        DeclareLaunchArgument(
            'imu_rpy', default_value='',
            description='Required with EKF: roll pitch yaw in radians'),
        OpaqueFunction(function=_launch_nodes),
    ])
