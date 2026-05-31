"""RViz 실행 파일.

목적:
  watch_area waypoint 좌표를 찍기 위한 전용 RViz 실행
"""

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('rc_car_bringup')

    # RViz 설정 파일 경로
    rviz_config = PathJoinSubstitution([
        bringup_share,
        'rviz',
        'waypoint.rviz',
    ])

    # RViz 노드 실행
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='mini1_waypoint_rviz',
        arguments=['-d', rviz_config],
        remappings=[
            ('/tf', '/robot3/tf'),
            ('/tf_static', '/robot3/tf_static'),
        ],
        output='screen',
    )

    return LaunchDescription([rviz_node])
