from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Start the single-wake KWS, VAD and SenseVoice input chain."""
    config = get_package_share_directory('k1muse_voice') + '/config/voice_input.yaml'
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=config,
            description='C14 ALSA, KWS, Silero VAD, and SenseVoice configuration.'),
        Node(
            package='k1muse_voice', executable='voice_input', name='voice_input',
            namespace='', output='screen',
            parameters=[LaunchConfiguration('params_file')]),
    ])
