#!/usr/bin/env python3
"""이미지 subscriber 노드.

입력:
  /homework/image   sensor_msgs/Image

역할:
  1. /homework/image 토픽 구독
  2. ROS Image의 OpenCV 이미지 변환
  3. 이미지 크기 출력
"""

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class ImageSubscriber(Node):
    """이미지 데이터 구독 노드."""

    def __init__(self):
        super().__init__('image_subscriber')

        # 구독 토픽 이름
        self._topic_name = '/homework/image'

        # ROS Image 변환 도구
        self._bridge = CvBridge()

        # Image 메시지 구독자
        self.create_subscription(Image, self._topic_name, self._on_image, 10)

        self.get_logger().info(f'구독 토픽: {self._topic_name}')

    def _on_image(self, message):
        """이미지 메시지 수신."""
        # ROS Image의 OpenCV 이미지 변환
        image = self._bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')

        # 이미지 높이와 너비 확인
        height, width = image.shape[:2]

        # 이미지 정보 출력
        self.get_logger().info(f'수신 이미지 크기: {width}x{height}')


def main():
    rclpy.init()
    node = ImageSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
