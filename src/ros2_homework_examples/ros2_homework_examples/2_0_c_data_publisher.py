#!/usr/bin/env python3
"""문자열 데이터 publisher 노드.

출력:
  /homework/data   std_msgs/String

역할:
  1. 문자열 메시지 생성
  2. /homework/data 토픽 발행
  3. ros2 topic echo 확인용 데이터 제공
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class DataPublisher(Node):
    """문자열 데이터 발행 노드."""

    def __init__(self):
        super().__init__('data_publisher')

        # 발행 토픽 이름
        self._topic_name = '/homework/data'

        # 메시지 번호
        self._count = 0

        # String 메시지 발행자
        self._publisher = self.create_publisher(String, self._topic_name, 10)

        # 1초 주기 타이머
        self.create_timer(1.0, self._publish_data)

        self.get_logger().info(f'발행 토픽: {self._topic_name}')

    def _publish_data(self):
        """문자열 메시지 발행."""
        # 메시지 객체 생성
        message = String()

        # 메시지 내용 작성
        message.data = f'ROS2 데이터 메시지 {self._count}'

        # 메시지 발행
        self._publisher.publish(message)

        # 발행 내용 출력
        self.get_logger().info(f'발행: {message.data}')

        # 메시지 번호 증가
        self._count += 1


def main():
    rclpy.init()
    node = DataPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
