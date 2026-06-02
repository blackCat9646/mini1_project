"""미니프로젝트 상태 관리 노드.

서비스:
  /rc_car/set_state        상태 변경 요청

출력:
  /rc_car/system_state     현재 상태 알림

역할:
  1. 현재 프로젝트 상태 저장
  2. 외부 명령으로 상태 변경
  3. 상태 메시지 반복 발행
"""

import rclpy
from rclpy.node import Node

from rc_car_interfaces.msg import SystemState
from rc_car_interfaces.srv import SetFollowState


class SupervisorNode(Node):
    """프로젝트 큰 상태 발행 노드."""

    # 허용 상태 목록
    VALID_STATES = {
        'idle',
        'search',
        'approach',
        'follow',
        'stop',
    }

    def __init__(self):
        super().__init__('supervisor_node')

        # 상태 발행 토픽
        self.declare_parameter('state_topic', '/rc_car/system_state')

        # 상태 변경 서비스
        self.declare_parameter('set_state_service', '/rc_car/set_state')

        state_topic = self.get_parameter('state_topic').value
        set_state_service = self.get_parameter('set_state_service').value

        # 초기 상태
        self._state = 'idle'
        self._message = 'System is waiting.'

        # 상태 발행자
        self._publisher = self.create_publisher(SystemState, state_topic, 10)

        # 상태 변경 서비스 서버
        self.create_service(SetFollowState, set_state_service, self._on_set_state)

        # 0.5초 주기 상태 발행
        self.create_timer(0.5, self._publish_state)

        self.get_logger().info(f'상태 발행 토픽: {state_topic}')
        self.get_logger().info(f'상태 변경 서비스: {set_state_service}')

    def _on_set_state(self, request, response):
        """상태 변경 서비스 처리."""
        # 명령어 소문자 정리
        command = request.command.strip().lower()

        # 허용되지 않은 상태 거절
        if command not in self.VALID_STATES:
            response.accepted = False
            response.message = f'Unknown command: {request.command}'
            return response

        # 상태 변경
        self._state = command
        self._message = f'State changed to {command}.'

        # 성공 응답 작성
        response.accepted = True
        response.message = self._message
        return response

    def _publish_state(self):
        """현재 상태 주기 발행."""
        # 상태 메시지 생성
        message = SystemState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.state = self._state
        message.message = self._message

        # 상태 메시지 발행
        self._publisher.publish(message)


def main():
    rclpy.init()
    node = SupervisorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
