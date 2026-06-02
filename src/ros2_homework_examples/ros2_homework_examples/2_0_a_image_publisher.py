#!/usr/bin/env python3
"""이미지 publisher 노드.

출력:
  /homework/image   sensor_msgs/Image

역할:
  1. OpenCV 이미지 생성
  2. ROS Image 메시지 변환
  3. /homework/image 토픽 발행
"""

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class ImagePublisher(Node):
    """이미지 데이터 발행 노드."""

    def __init__(self):
        super().__init__('image_publisher')

        # 발행 토픽 이름
        self._topic_name = '/homework/image'

        # ROS Image 변환 도구
        self._bridge = CvBridge()

        # 애니메이션 번호
        self._count = 0

        # Image 메시지 발행자
        self._publisher = self.create_publisher(Image, self._topic_name, 10)

        # 0.2초 주기 타이머
        self.create_timer(0.2, self._publish_image)

        self.get_logger().info(f'발행 토픽: {self._topic_name}')

    def _make_image(self):
        """테스트 이미지 생성."""
        # 검은 배경 이미지
        image = np.zeros((240, 320, 3), dtype=np.uint8)

        # 움직이는 원의 x 좌표
        circle_x = 40 + (self._count * 8) % 240

        # 초록색 원 그리기
        cv2.circle(image, (circle_x, 120), 28, (0, 220, 0), -1)

        # 설명 글자 그리기
        cv2.putText(
            image,
            'ROS2 Image',
            (90, 45),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2,
        )
        return image

    def _publish_image(self):
        """이미지 메시지 발행."""
        # OpenCV 이미지 생성
        image = self._make_image()

        # OpenCV 이미지의 ROS Image 변환
        message = self._bridge.cv2_to_imgmsg(image, encoding='bgr8')

        # 현재 시간 기록
        message.header.stamp = self.get_clock().now().to_msg()

        # 프레임 이름 기록
        message.header.frame_id = 'homework_camera'

        # 이미지 메시지 발행
        self._publisher.publish(message)

        # 애니메이션 번호 증가
        self._count += 1


def main():
    rclpy.init()
    node = ImagePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
