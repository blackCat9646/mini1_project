"""삼각대 카메라 픽셀 좌표의 map 좌표 변환 노드.

입력:
  /rc_car/target/tripod_2d   삼각대 화면 속 RC카 위치

출력:
  /rc_car/target/map         지도 위 RC카 위치

핵심 개념:
  1. YOLO 결과: 화면 픽셀 좌표
  2. Nav2 목표: map 미터 좌표
  3. Homography: 픽셀 좌표와 map 좌표의 변환표
"""

import yaml

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node

from rc_car_interfaces.msg import Target2D, Target3D


class PixelToMapNode(Node):
    """삼각대 2D 감지 결과의 map 좌표 변환 노드."""

    def __init__(self):
        super().__init__('pixel_to_map_node')

        # 2D 감지 결과 입력 토픽
        self.declare_parameter('input_topic', '/rc_car/target/tripod_2d')

        # map 좌표 출력 토픽
        self.declare_parameter('output_topic', '/rc_car/target/map')

        # homography 보정 파일
        self.declare_parameter('homography_file', '')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        homography_file = self.get_parameter('homography_file').value

        # homography 행렬 로딩
        self._homography = self._load_homography(homography_file)

        # map target 발행자
        self._publisher = self.create_publisher(Target3D, output_topic, 10)

        # 2D target 구독자
        self.create_subscription(Target2D, input_topic, self._on_target_2d, 10)

        self.get_logger().info(f'2D target 구독 토픽: {input_topic}')
        self.get_logger().info(f'map target 발행 토픽: {output_topic}')

    def _load_homography(self, path):
        """homography 행렬 파일 읽기."""
        # 파일 경로 없음 처리
        if not path:
            self.get_logger().warn('homography_file 비어 있음')
            return None

        try:
            # YAML 파일 읽기
            with open(path, 'r', encoding='utf-8') as yaml_file:
                data = yaml.safe_load(yaml_file)
        except OSError as error:
            self.get_logger().warn(f'homography 파일 읽기 실패: {error}')
            return None

        # 3x3 행렬 확인
        matrix = data.get('homography')
        if not matrix or len(matrix) != 3:
            self.get_logger().warn('homography 3x3 행렬 필요')
            return None

        return matrix

    def _pixel_to_map(self, pixel_x, pixel_y):
        """픽셀 좌표의 map 좌표 변환."""
        # homography 행렬 별칭
        h = self._homography

        # homography 분모 계산
        scale = h[2][0] * pixel_x + h[2][1] * pixel_y + h[2][2]
        if abs(scale) < 1e-6:
            return None

        # map x, y 계산
        map_x = (h[0][0] * pixel_x + h[0][1] * pixel_y + h[0][2]) / scale
        map_y = (h[1][0] * pixel_x + h[1][1] * pixel_y + h[1][2]) / scale
        return map_x, map_y

    def _on_target_2d(self, target_2d):
        # map target 기본 메시지 생성
        target_3d = Target3D()
        target_3d.header.stamp = self.get_clock().now().to_msg()
        target_3d.header.frame_id = 'map'
        target_3d.source = 'tripod_mapper'
        target_3d.class_name = target_2d.class_name
        target_3d.confidence = target_2d.confidence
        target_3d.track_id = target_2d.track_id
        target_3d.distance = -1.0

        # 미감지 또는 보정값 없음 처리
        if not target_2d.detected or self._homography is None:
            target_3d.detected = False
            self._publisher.publish(target_3d)
            return

        # 픽셀 좌표의 map 좌표 변환
        map_point = self._pixel_to_map(target_2d.center_x, target_2d.center_y)
        if map_point is None:
            target_3d.detected = False
            self._publisher.publish(target_3d)
            return

        # map 좌표 감지 결과 작성
        target_3d.detected = True
        target_3d.position = Point(x=map_point[0], y=map_point[1], z=0.0)

        # map target 발행
        self._publisher.publish(target_3d)


def main():
    rclpy.init()
    node = PixelToMapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
