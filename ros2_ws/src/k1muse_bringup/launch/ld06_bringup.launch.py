import platform
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'port_name',
            default_value='/dev/mylidar',
            description='LD06 serial device path.',
        ),
        Node(prefix='taskset -c 6,7' if platform.machine() == "riscv64" else "",
            package='ldlidar_stl_ros2',
            executable='ldlidar_stl_ros2_node',
            name='ld06',
            output='screen',
            parameters=[{
                'product_name': 'LDLiDAR_LD06',
                'topic_name': 'scan',
                'frame_id': 'base_laser',
                'port_name': LaunchConfiguration('port_name'),
                'port_baudrate': 230400,
                'laser_scan_dir': True,
                'enable_angle_crop_func': False,
            }],
        ),
        Node(prefix='taskset -c 6,7' if platform.machine() == "riscv64" else "",
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_base_laser',
            arguments=[
                '--x', '0.164284',
                '--y', '0',
                '--z', '0.084',
                '--roll', '0',
                '--pitch', '0',
                '--yaw', '0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'base_laser',
            ],
            output='screen',
        ),
    ])
