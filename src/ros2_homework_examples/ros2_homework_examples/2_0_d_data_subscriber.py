#!/usr/bin/env python3
"""문자열 데이터 subscriber 노드.

입력:
  /homework/data   std_msgs/String

역할:
  1. /homework/data 토픽 구독
  2. 수신 문자열 출력
  3. publisher/subscriber 연결 확인
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class DataSubscriber(Node):
    """문자열 데이터 구독 노드."""

    def __init__(self):
        super().__init__('data_subscriber')

        # 구독 토픽 이름
        self._topic_name = '/homework/data'

        # String 메시지 구독자
        self.create_subscription(String, self._topic_name, self._on_data, 10)

        self.get_logger().info(f'구독 토픽: {self._topic_name}')

    def _on_data(self, message):
        """문자열 메시지 수신."""
        # 수신 내용 출력
        self.get_logger().info(f'수신: {message.data}')


def main():
    rclpy.init()
    node = DataSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
