import platform
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    """Launch the same C05 nodes separately or in the official isolated container."""
    bringup_share = get_package_share_directory('k1muse_bringup')
    nav2_share = get_package_share_directory('nav2_bringup')
    map_yaml = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    visualize = LaunchConfiguration('visualize')
    navigation_autostart = LaunchConfiguration('navigation_autostart')
    navigation_output = LaunchConfiguration('navigation_output')
    use_composition = LaunchConfiguration('use_composition')

    # Costmap child nodes read process-global ROS arguments. Rewrite the complete
    # upstream file, not just the parent node's use_sim_time override.
    upstream_parameters = ParameterFile(RewrittenYaml(
        source_file=os.path.join(nav2_share, 'params', 'nav2_params.yaml'),
        param_rewrites={'use_sim_time': use_sim_time}, convert_types=True))
    common_parameters = [
        upstream_parameters,
        os.path.join(bringup_share, 'config', 'nav2_navigation.yaml'),
        {'use_sim_time': use_sim_time},
    ]
    navigation_nodes = [
        'controller_server', 'planner_server', 'behavior_server',
        'bt_navigator', 'velocity_smoother',
    ]
    # One concrete node list keeps lifecycle names, parameters and velocity routes
    # identical in both deployment modes.
    nodes = [
        ('nav2_map_server', 'map_server', 'map_server', 'nav2_map_server::MapServer',
         common_parameters + [{'yaml_filename': map_yaml}], []),
        ('nav2_amcl', 'amcl', 'amcl', 'nav2_amcl::AmclNode', common_parameters, []),
        ('nav2_lifecycle_manager', 'lifecycle_manager', 'lifecycle_manager_localization',
         'nav2_lifecycle_manager::LifecycleManager', [{
             'use_sim_time': use_sim_time, 'autostart': True,
             'node_names': ['map_server', 'amcl']}], []),
        ('nav2_controller', 'controller_server', 'controller_server',
         'nav2_controller::ControllerServer', common_parameters, [('cmd_vel', '/cmd_vel_nav')]),
        ('nav2_planner', 'planner_server', 'planner_server',
         'nav2_planner::PlannerServer', common_parameters, []),
        ('nav2_behaviors', 'behavior_server', 'behavior_server',
         'behavior_server::BehaviorServer', common_parameters, [('cmd_vel', '/cmd_vel_nav')]),
        ('nav2_bt_navigator', 'bt_navigator', 'bt_navigator',
         'nav2_bt_navigator::BtNavigator', common_parameters, []),
        ('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother',
         'nav2_velocity_smoother::VelocitySmoother', common_parameters,
         [('cmd_vel', '/cmd_vel_nav'), ('cmd_vel_smoothed', navigation_output)]),
        ('nav2_lifecycle_manager', 'lifecycle_manager', 'lifecycle_manager_navigation',
         'nav2_lifecycle_manager::LifecycleManager', [{
             'use_sim_time': use_sim_time, 'autostart': navigation_autostart,
             'node_names': navigation_nodes}], []),
    ]
    return LaunchDescription([
        DeclareLaunchArgument('map', description='Full path to the C04-saved map YAML file.'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('visualize', default_value='false'),
        DeclareLaunchArgument('navigation_autostart', default_value='true'),
        DeclareLaunchArgument('navigation_output', default_value='/cmd_vel'),
        DeclareLaunchArgument(
            'use_composition', default_value='true', choices=['true', 'false'],
            description='Use the official isolated Nav2 container; false keeps separate processes.'),
        *[Node(prefix='taskset -c 4-7' if platform.machine() == "riscv64" else "",
            condition=UnlessCondition(use_composition), package=package,
            executable=executable, name=name, output='screen',
            parameters=parameters, remappings=remappings,
        ) for package, executable, name, _, parameters, remappings in nodes],
        ComposableNodeContainer(prefix='taskset -c 4-7' if platform.machine() == "riscv64" else "",
            condition=IfCondition(use_composition),
            package='rclcpp_components', executable='component_container_isolated',
            name='nav2_container', namespace='', output='screen',
            # Pass both full files to the process as well as each component:
            # local/global_costmap inherit their own blocks from global arguments.
            parameters=common_parameters,
            composable_node_descriptions=[ComposableNode(
                package=package, plugin=plugin, name=name,
                parameters=parameters, remappings=remappings,
            ) for package, _, name, plugin, parameters, remappings in nodes],
        ),
        Node(prefix='taskset -c 4-7' if platform.machine() == "riscv64" else "",
            condition=IfCondition(visualize), package='rviz2', executable='rviz2',
            name='navigation_rviz',
            arguments=['-d', os.path.join(bringup_share, 'rviz', 'navigation.rviz')],
            output='screen',
        ),
    ])
