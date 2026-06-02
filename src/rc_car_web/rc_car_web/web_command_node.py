"""웹 버튼 명령을 TurtleBot4 액션 명령으로 바꾸는 노드."""

import math
from pathlib import Path

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from irobot_create_msgs.action import Dock, Undock
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String


def yaw_to_quaternion(yaw):
    """지도 yaw 각도에서 쿼터니언 생성."""
    half_yaw = yaw * 0.5
    return {
        'x': 0.0,
        'y': 0.0,
        'z': math.sin(half_yaw),
        'w': math.cos(half_yaw),
    }


class WebCommandNode(Node):
    """웹 명령 수신 및 로봇 액션 호출 노드."""

    def __init__(self):
        super().__init__('web_command_node')

        # 파라미터 선언
        self.declare_parameter('command_topic', '/rc_car/web/command')
        self.declare_parameter('status_topic', '/rc_car/web/status')
        self.declare_parameter('corner_file', '')
        self.declare_parameter('navigate_action', '/robot3/navigate_to_pose')
        self.declare_parameter('dock_action', '/robot3/dock')
        self.declare_parameter('undock_action', '/robot3/undock')
        self.declare_parameter('map_frame', 'map')

        # 파라미터 값 읽기
        command_topic = self.get_parameter('command_topic').value
        status_topic = self.get_parameter('status_topic').value
        self.corner_file = self.get_parameter('corner_file').value
        self.map_frame = self.get_parameter('map_frame').value
        navigate_action = self.get_parameter('navigate_action').value
        dock_action = self.get_parameter('dock_action').value
        undock_action = self.get_parameter('undock_action').value

        # 코너 좌표 파일 읽기
        self.corners = self.load_corners(self.corner_file)

        # 웹 상태 발행자
        self.status_pub = self.create_publisher(String, status_topic, 10)

        # 웹 명령 구독자
        self.command_sub = self.create_subscription(
            String,
            command_topic,
            self.command_callback,
            10,
        )

        # Nav2 이동 액션 클라이언트
        self.navigate_client = ActionClient(
            self,
            NavigateToPose,
            navigate_action,
        )

        # Dock 액션 클라이언트
        self.dock_client = ActionClient(
            self,
            Dock,
            dock_action,
        )

        # Undock 액션 클라이언트
        self.undock_client = ActionClient(
            self,
            Undock,
            undock_action,
        )

        self.publish_status('웹 명령 노드 준비 완료')
        self.get_logger().info(f'웹 명령 토픽: {command_topic}')
        self.get_logger().info(f'코너 좌표 파일: {self.corner_file}')

    def load_corners(self, corner_file):
        """YAML 파일에서 코너 좌표 읽기."""
        if not corner_file:
            self.get_logger().warn('코너 좌표 파일 미설정')
            return {}

        path = Path(corner_file)
        if not path.exists():
            self.get_logger().error(f'코너 좌표 파일 없음: {corner_file}')
            return {}

        with path.open('r', encoding='utf-8') as yaml_file:
            data = yaml.safe_load(yaml_file) or {}

        corners = data.get('corners', {})
        self.get_logger().info(f'코너 좌표 개수: {len(corners)}')
        return corners

    def command_callback(self, msg):
        """웹 명령 처리."""
        command = msg.data.strip()

        # 빈 명령 무시
        if not command:
            return

        self.get_logger().info(f'웹 명령 수신: {command}')

        # Dock 명령 처리
        if command == 'dock':
            self.send_dock_goal()
            return

        # Undock 명령 처리
        if command == 'undock':
            self.send_undock_goal()
            return

        # 코너 이동 명령 처리
        if command in self.corners:
            self.send_corner_goal(command)
            return

        # 알 수 없는 명령 처리
        self.publish_status(f'알 수 없는 명령: {command}')
        self.get_logger().warn(f'알 수 없는 웹 명령: {command}')

    def send_corner_goal(self, corner_name):
        """Nav2 코너 이동 목표 전송."""
        corner = self.corners[corner_name]

        # 목표 자세 메시지 생성
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = corner.get('frame_id', self.map_frame)
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(corner['x'])
        goal_msg.pose.pose.position.y = float(corner['y'])
        goal_msg.pose.pose.position.z = 0.0

        # 목표 방향 설정
        quaternion = yaw_to_quaternion(float(corner.get('yaw', 0.0)))
        goal_msg.pose.pose.orientation.x = quaternion['x']
        goal_msg.pose.pose.orientation.y = quaternion['y']
        goal_msg.pose.pose.orientation.z = quaternion['z']
        goal_msg.pose.pose.orientation.w = quaternion['w']

        # Nav2 서버 대기
        if not self.navigate_client.wait_for_server(timeout_sec=2.0):
            self.publish_status('Nav2 액션 서버 연결 실패')
            self.get_logger().error('Nav2 액션 서버 연결 실패')
            return

        # Nav2 목표 전송
        self.publish_status(f'{corner_name} 이동 요청')
        send_future = self.navigate_client.send_goal_async(goal_msg)
        send_future.add_done_callback(
            lambda future: self.handle_goal_response(future, corner_name)
        )

    def handle_goal_response(self, future, goal_name):
        """Nav2 목표 수락 결과 처리."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.publish_status(f'{goal_name} 목표 거절')
            self.get_logger().warn(f'{goal_name} 목표 거절')
            return

        self.publish_status(f'{goal_name} 이동 시작')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result: self.handle_goal_result(result, goal_name)
        )

    def handle_goal_result(self, future, goal_name):
        """Nav2 이동 완료 결과 처리."""
        status = future.result().status
        self.publish_status(f'{goal_name} 이동 완료 상태: {status}')
        self.get_logger().info(f'{goal_name} 이동 완료 상태: {status}')

    def send_dock_goal(self):
        """Dock 액션 목표 전송."""
        if not self.dock_client.wait_for_server(timeout_sec=2.0):
            self.publish_status('Dock 액션 서버 연결 실패')
            self.get_logger().error('Dock 액션 서버 연결 실패')
            return

        self.publish_status('Dock 요청')
        self.dock_client.send_goal_async(Dock.Goal())

    def send_undock_goal(self):
        """Undock 액션 목표 전송."""
        if not self.undock_client.wait_for_server(timeout_sec=2.0):
            self.publish_status('Undock 액션 서버 연결 실패')
            self.get_logger().error('Undock 액션 서버 연결 실패')
            return

        self.publish_status('Undock 요청')
        self.undock_client.send_goal_async(Undock.Goal())

    def publish_status(self, text):
        """웹 상태 문장 발행."""
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)


def main(args=None):
    """노드 실행 시작점."""
    rclpy.init(args=args)
    node = WebCommandNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
