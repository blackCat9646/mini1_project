"""Start only the RC car project nodes.

Use this launch after TurtleBot4 localization and Nav2 are already active.
"""

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('rc_car_bringup')

    project_params_arg = DeclareLaunchArgument(
        'project_params_file',
        default_value=PathJoinSubstitution([bringup_share, 'config', 'project.yaml']),
        description='Project node parameters.',
    )

    start_supervisor_arg = DeclareLaunchArgument(
        'start_supervisor',
        default_value='true',
        description='Start supervisor_node.',
    )
    start_tripod_arg = DeclareLaunchArgument(
        'start_tripod',
        default_value='true',
        description='Start tripod_trigger_node.',
    )
    start_approach_arg = DeclareLaunchArgument(
        'start_approach',
        default_value='true',
        description='Start approach_planner_node.',
    )
    start_oakd_arg = DeclareLaunchArgument(
        'start_oakd',
        default_value='false',
        description='Start oakd_yolo_depth_node. Use true only after Nav2 is stable.',
    )
    start_tracker_arg = DeclareLaunchArgument(
        'start_tracker',
        default_value='false',
        description='Start nav2_target_tracker_node. Use true only after OAK-D test is stable.',
    )

    return LaunchDescription([
        project_params_arg,
        start_supervisor_arg,
        start_tripod_arg,
        start_approach_arg,
        start_oakd_arg,
        start_tracker_arg,
        Node(
            package='rc_car_follower',
            executable='supervisor_node',
            name='supervisor_node',
            output='screen',
            parameters=[LaunchConfiguration('project_params_file')],
            condition=IfCondition(LaunchConfiguration('start_supervisor')),
        ),
        Node(
            package='rc_car_follower',
            executable='tripod_trigger_node',
            name='tripod_trigger_node',
            output='screen',
            parameters=[LaunchConfiguration('project_params_file')],
            condition=IfCondition(LaunchConfiguration('start_tripod')),
        ),
        Node(
            package='rc_car_follower',
            executable='approach_planner_node',
            name='approach_planner_node',
            output='screen',
            parameters=[LaunchConfiguration('project_params_file')],
            condition=IfCondition(LaunchConfiguration('start_approach')),
        ),
        Node(
            package='rc_car_follower',
            executable='oakd_yolo_depth_node',
            name='oakd_yolo_depth_node',
            output='screen',
            parameters=[LaunchConfiguration('project_params_file')],
            condition=IfCondition(LaunchConfiguration('start_oakd')),
        ),
        Node(
            package='rc_car_follower',
            executable='nav2_target_tracker_node',
            name='nav2_target_tracker_node',
            output='screen',
            parameters=[LaunchConfiguration('project_params_file')],
            condition=IfCondition(LaunchConfiguration('start_tracker')),
            remappings=[
                ('/tf', '/robot3/tf'),
                ('/tf_static', '/robot3/tf_static'),
            ],
        ),
    ])
