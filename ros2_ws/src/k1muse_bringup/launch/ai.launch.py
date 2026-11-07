import platform
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_inference(context):
    """Keep each node's parameter file local to that node in the fixed process."""
    voice = LaunchConfiguration('enable_voice').perform(context)
    yolo = LaunchConfiguration('enable_yolo').perform(context)
    ocr = LaunchConfiguration('enable_ocr').perform(context)
    if ocr == 'true' and yolo != 'true':
        raise ValueError('enable_ocr=true requires enable_yolo=true')
    if voice == 'false' and yolo == 'false':
        return []
    mode = LaunchConfiguration('voice_startup_mode').perform(context)
    return [Node(prefix='taskset -c 4-7' if platform.machine() == "riscv64" else "",
        package='k1muse_manager', executable='ai_inference', output='screen',
        # No global __node remap: each composed node keeps its own name.
        arguments=[
            '--enable-voice', voice, '--enable-yolo', yolo, '--enable-ocr', ocr,
            '--voice-input-params', LaunchConfiguration('voice_input_params'),
            '--voice-command-params', LaunchConfiguration('voice_command_params'),
            '--yolo-params', LaunchConfiguration('yolo_params'),
            '--ocr-params', LaunchConfiguration('ocr_params'),
            '--voice-config-dir', LaunchConfiguration('voice_config_dir'),
            *(['--voice-startup-mode', mode] if voice == 'true' and mode else [])],
        # Read by ggml at library load; ASR/YOLO keep their runtime TCM path.
        # Requires the matching SDK patch in tools/cross/patches.
        additional_env={
            'SPACEMIT_PERFER_CORE_ID': '0,1,2,3',
            'SPACEMIT_DISABLE_TCM': '1',
        },
    )]


def generate_launch_description():
    """Start one manager and one fixed inference process, then prepare serially."""
    manager = get_package_share_directory('k1muse_manager')
    voice = get_package_share_directory('k1muse_voice')
    perception = get_package_share_directory('k1muse_perception')
    voice_enabled = LaunchConfiguration('enable_voice')
    yolo_enabled = LaunchConfiguration('enable_yolo')
    return LaunchDescription([
        DeclareLaunchArgument('enable_voice', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('enable_yolo', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('enable_ocr', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('ocr_params', default_value=perception + '/config/ocr.yaml'),
        DeclareLaunchArgument('config_dir', default_value='/home/bianbu/k1muse_config'),
        DeclareLaunchArgument('voice_config_dir', default_value=LaunchConfiguration('config_dir')),
        DeclareLaunchArgument('voice_startup_mode', default_value='',
                              choices=['', 'rules', 'local', 'cloud'],
                              description='Optional process-local override of the saved intent mode'),
        DeclareLaunchArgument('voice_input_params', default_value=voice + '/config/voice_input.yaml'),
        DeclareLaunchArgument('voice_command_params', default_value=manager + '/config/voice_command.yaml'),
        DeclareLaunchArgument('yolo_params', default_value=perception + '/config/yolo.yaml'),
        DeclareLaunchArgument('drug_catalog_file', default_value=PathJoinSubstitution([
            LaunchConfiguration('config_dir'), 'drug_catalog.yaml'])),
        DeclareLaunchArgument('fetch_config_file', default_value=manager + '/config/fetch_demo.yaml'),
        DeclareLaunchArgument('survey_config_file', default_value=manager + '/config/survey.yaml'),
        DeclareLaunchArgument('warehouse_config_file', default_value='',
                              description='Warehouse JSON loaded once at task_manager startup'),
        DeclareLaunchArgument('enable_fetch_perception', default_value='false', choices=['true', 'false']),
        *[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(perception + '/launch/' + name + '.launch.py'),
            condition=IfCondition(LaunchConfiguration('enable_fetch_perception')))
          for name in ['cv', 'depth', 'target_transform']],
        Node(prefix='taskset -c 4-7' if platform.machine() == "riscv64" else "", package='k1muse_manager', executable='task_manager_node', name='task_manager',
             output='screen', parameters=[{
                 'fetch_config_file': ParameterValue(LaunchConfiguration('fetch_config_file'), value_type=str),
                 'survey_config_file': ParameterValue(LaunchConfiguration('survey_config_file'), value_type=str),
                 'warehouse_config_file': ParameterValue(LaunchConfiguration('warehouse_config_file'), value_type=str),
                 'drug_catalog_file': LaunchConfiguration('drug_catalog_file'),
                 'inference_auto_initialize': True,
                 'inference_enable_voice': ParameterValue(voice_enabled, value_type=bool),
                 'inference_enable_yolo': ParameterValue(yolo_enabled, value_type=bool),
             }]),
        OpaqueFunction(function=launch_inference),
    ])
