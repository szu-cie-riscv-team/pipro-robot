"""Shared Demo1/2 startup with optional voice and intent backend override."""
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    if LaunchConfiguration('start_at_origin_empty').perform(context) != 'true':
        raise ValueError('Set start_at_origin_empty:=true only with the robot at the saved '
                         '建图起点, matching heading, empty gripper and clear ARM workspace. '
                         'Startup resets MCU odometry and moves ARM to ready.')
    args = {name: LaunchConfiguration(name).perform(context) for name in
            ('fetch_config_file', 'warehouse_config_file', 'camera_params_file', 'web_config')}
    for name, value in args.items():
        if not Path(value).is_file():
            raise ValueError(name + ' file missing: ' + value)
    fetch = yaml.safe_load(Path(args['fetch_config_file']).read_text(encoding='utf-8'))
    survey_path = LaunchConfiguration('survey_config_file').perform(context)
    survey = yaml.safe_load(Path(survey_path).read_text(encoding='utf-8')) if survey_path else {}
    voice = LaunchConfiguration('enable_voice').perform(context)
    backend = LaunchConfiguration('voice_backend').perform(context)
    mode = {'config': '', 'local_llm': 'local', 'cloud_llm': 'cloud'}[backend]
    configs = LaunchConfiguration('config_dir').perform(context)
    voice_configs = LaunchConfiguration('voice_config_dir').perform(context) or configs
    catalog = LaunchConfiguration('drug_catalog_file').perform(context) or str(Path(configs) / 'drug_catalog.yaml')
    bringup = get_package_share_directory('k1muse_bringup')
    perception = get_package_share_directory('k1muse_perception')
    web = get_package_share_directory('k1muse_web')

    def include(directory, name, values=None):
        # Common child arguments such as params_file must stay local.
        return GroupAction(actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(Path(directory) / 'launch' / (name + '.launch.py'))),
            launch_arguments=(values or {}).items())], scoped=True)

    nodes = [
        include(web, 'web', {'config': args['web_config'], 'config_dir': configs,
                            'voice_config_dir': voice_configs, 'drug_catalog_file': catalog}),
        include(bringup, 'camera', {'params_file': args['camera_params_file']}),
        include(perception, 'depth', {'input_mode': 'off',
                                    'params_file': LaunchConfiguration('depth_params_file').perform(context)}),
        include(perception, 'target_transform', {
            'params_file': LaunchConfiguration('target_transform_params_file').perform(context)}),
        include(bringup, 'ai', {
            'enable_voice': voice, 'enable_yolo': 'true', 'enable_ocr': 'false',
            'voice_startup_mode': mode if voice == 'true' else '',
            'drug_catalog_file': catalog,
            'config_dir': configs, 'voice_config_dir': voice_configs,
            **{name: LaunchConfiguration(name).perform(context) for name in
               ('voice_input_params', 'voice_command_params', 'yolo_params')},
            'enable_fetch_perception': 'false',
            'fetch_config_file': args['fetch_config_file'],
            'survey_config_file': survey_path,
            'warehouse_config_file': args['warehouse_config_file']}),
    ]
    if not fetch.get('qr', {}).get('no_qr', False) or survey.get('enabled', False):
        nodes.append(include(perception, 'cv', {
            'params_file': LaunchConfiguration('cv_params_file').perform(context)}))
    prepare_args = ['--fetch-config', args['fetch_config_file'], '--start-at-origin-empty']
    if survey.get('enabled', False):
        prepare_args.append('--prepare-survey')
    if voice == 'true':
        prepare_args.append('--enable-voice')
    nodes.append(Node(package='k1muse_bringup', executable='demo2_prepare',
                      arguments=prepare_args,
                      output='screen'))
    return nodes


def generate_launch_description():
    bringup = get_package_share_directory('k1muse_bringup')
    web = get_package_share_directory('k1muse_web')
    voice = get_package_share_directory('k1muse_voice')
    manager = get_package_share_directory('k1muse_manager')
    perception = get_package_share_directory('k1muse_perception')
    return LaunchDescription([
        DeclareLaunchArgument('fetch_config_file', description='Measured Demo2 scene YAML'),
        DeclareLaunchArgument('warehouse_config_file', description='Synced warehouse JSON'),
        DeclareLaunchArgument('survey_config_file', default_value=str(Path(manager) / 'config' / 'survey.yaml')),
        DeclareLaunchArgument('enable_voice', default_value='false', choices=['false', 'true']),
        DeclareLaunchArgument('voice_backend', default_value='config',
                              choices=['config', 'local_llm', 'cloud_llm'],
                              description='config uses the saved mode; explicit backend overrides this run'),
        DeclareLaunchArgument('voice_input_params', default_value=str(
            Path(voice) / 'config' / 'voice_input.yaml')),
        DeclareLaunchArgument('voice_command_params', default_value=str(
            Path(manager) / 'config' / 'voice_command.yaml')),
        DeclareLaunchArgument('yolo_params', default_value=str(Path(perception) / 'config' / 'yolo.yaml')),
        DeclareLaunchArgument('depth_params_file', default_value=str(Path(perception) / 'config' / 'depth.yaml')),
        DeclareLaunchArgument('target_transform_params_file', default_value=str(
            Path(perception) / 'config' / 'target_transform.yaml')),
        DeclareLaunchArgument('cv_params_file', default_value=str(Path(perception) / 'config' / 'cv.yaml')),
        DeclareLaunchArgument('config_dir', default_value='/home/bianbu/k1muse_config'),
        DeclareLaunchArgument('voice_config_dir', default_value=''),
        DeclareLaunchArgument('drug_catalog_file', default_value=''),
        DeclareLaunchArgument('start_at_origin_empty', default_value='false', choices=['false', 'true']),
        DeclareLaunchArgument('camera_params_file', default_value=str(
            Path(bringup) / 'config' / 'camera_demo2.yaml')),
        DeclareLaunchArgument('web_config', default_value=str(Path(web) / 'config' / 'web.yaml')),
        OpaqueFunction(function=start),
    ])
