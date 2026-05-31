"""Start the RC car following mini project.

This launch file starts:
  1. TurtleBot4 localization with hoon_map.
  2. TurtleBot4 Nav2 navigation.
  3. Project nodes for detection, mapping, approaching, and following.
"""

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('rc_car_bringup')
    turtlebot4_navigation_share = get_package_share_directory('turtlebot4_navigation')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([bringup_share, 'maps', 'hoon_map.yaml']),
        description='Full path to the hoon_map YAML file.',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use false on the real TurtleBot4.',
    )
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='robot3',
        description='Robot namespace. TurtleBot4 is currently running as robot3.',
    )
    nav2_params_arg = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=PathJoinSubstitution([
            turtlebot4_navigation_share,
            'config',
            'nav2.yaml',
        ]),
        description='Nav2 parameter file.',
    )
    project_params_arg = DeclareLaunchArgument(
        'project_params_file',
        default_value=PathJoinSubstitution([bringup_share, 'config', 'project.yaml']),
        description='Project node parameters.',
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                turtlebot4_navigation_share,
                'launch',
                'localization.launch.py',
            ])
        ),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                turtlebot4_navigation_share,
                'launch',
                'nav2.launch.py',
            ])
        ),
        launch_arguments={
            'namespace': LaunchConfiguration('namespace'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('nav2_params_file'),
        }.items(),
    )

    project_nodes = [
        Node(
            package='rc_car_follower',
            executable='supervisor_node',
            name='supervisor_node',
            output='screen',
            parameters=[LaunchConfiguration('project_params_file')],
        ),
        Node(
            package='rc_car_follower',
            executable='tripod_trigger_node',
            name='tripod_trigger_node',
            output='screen',
            parameters=[LaunchConfiguration('project_params_file')],
        ),
        Node(
            package='rc_car_follower',
            executable='approach_planner_node',
            name='approach_planner_node',
            output='screen',
            parameters=[LaunchConfiguration('project_params_file')],
        ),
        Node(
            package='rc_car_follower',
            executable='oakd_yolo_depth_node',
            name='oakd_yolo_depth_node',
            output='screen',
            parameters=[LaunchConfiguration('project_params_file')],
        ),
        Node(
            package='rc_car_follower',
            executable='follow_controller_node',
            name='follow_controller_node',
            output='screen',
            parameters=[LaunchConfiguration('project_params_file')],
        ),
    ]

    return LaunchDescription([
        map_arg,
        use_sim_time_arg,
        namespace_arg,
        nav2_params_arg,
        project_params_arg,
        localization,
        nav2,
        *project_nodes,
    ])
