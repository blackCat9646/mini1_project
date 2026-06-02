"""Start rosbridge and web control nodes."""

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('rc_car_bringup')
    rosbridge_share = get_package_share_directory('rosbridge_server')

    corner_file_arg = DeclareLaunchArgument(
        'corner_file',
        default_value=PathJoinSubstitution([
            bringup_share,
            'config',
            'web_corners.yaml',
        ]),
        description='Four corner waypoint YAML file for web buttons.',
    )

    rosbridge = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            PathJoinSubstitution([
                rosbridge_share,
                'launch',
                'rosbridge_websocket_launch.xml',
            ])
        )
    )

    web_command_node = Node(
        package='rc_car_web',
        executable='web_command_node',
        name='web_command_node',
        output='screen',
        parameters=[{
            'command_topic': '/rc_car/web/command',
            'status_topic': '/rc_car/web/status',
            'corner_file': LaunchConfiguration('corner_file'),
            'navigate_action': '/robot3/navigate_to_pose',
            'dock_action': '/robot3/dock',
            'undock_action': '/robot3/undock',
            'map_frame': 'map',
        }],
    )

    web_pose_node = Node(
        package='rc_car_web',
        executable='web_pose_node',
        name='web_pose_node',
        output='screen',
        parameters=[{
            'pose_topic': '/rc_car/web/robot_pose',
            'amcl_pose_topic': '/robot3/amcl_pose',
            'map_frame': 'map',
            'robot_frame': 'base_link',
            'publish_rate': 5.0,
        }],
        remappings=[
            ('/tf', '/robot3/tf'),
            ('/tf_static', '/robot3/tf_static'),
        ],
    )

    web_map_node = Node(
        package='rc_car_web',
        executable='web_map_node',
        name='web_map_node',
        output='screen',
        parameters=[{
            'map_topic': '/robot3/map',
            'web_map_topic': '/rc_car/web/map',
            'map_service': '/robot3/map_server/map',
            'publish_rate': 1.0,
        }],
    )

    return LaunchDescription([
        corner_file_arg,
        rosbridge,
        web_command_node,
        web_pose_node,
        web_map_node,
    ])
