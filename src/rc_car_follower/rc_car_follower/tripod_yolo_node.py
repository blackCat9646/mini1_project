"""삼각대 카메라 YOLO 2D 감지 노드.

입력:
  /tripod/image_raw            삼각대 카메라 영상

출력:
  /rc_car/target/tripod_2d     RC카 2D 위치

역할:
  1. 삼각대 영상 수신
  2. YOLO RC카 감지
  3. 화면 속 RC카 중심 좌표 발행
"""

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

from rc_car_interfaces.msg import Target2D
from rc_car_follower.yolo_helper import biggest_box, load_yolo_model


class TripodYoloNode(Node):
    """삼각대 카메라용 YOLO 감지 노드."""

    def __init__(self):
        super().__init__('tripod_yolo_node')

        # 입력 영상 토픽
        self.declare_parameter('image_topic', '/tripod/image_raw')

        # 2D 감지 결과 토픽
        self.declare_parameter('target_topic', '/rc_car/target/tripod_2d')

        # YOLO 모델 경로
        self.declare_parameter('model_path', '')

        # 찾을 클래스 이름
        self.declare_parameter('target_class_name', 'rc_car')

        # 최소 신뢰도
        self.declare_parameter('confidence_threshold', 0.45)

        image_topic = self.get_parameter('image_topic').value
        target_topic = self.get_parameter('target_topic').value
        model_path = self.get_parameter('model_path').value

        self._target_class_name = self.get_parameter('target_class_name').value
        self._confidence_threshold = self.get_parameter('confidence_threshold').value

        # ROS Image와 OpenCV 이미지 변환 도구
        self._bridge = CvBridge()

        # YOLO 모델 로딩
        self._model = load_yolo_model(self, model_path)

        # 2D target 발행자
        self._publisher = self.create_publisher(Target2D, target_topic, 10)

        # 삼각대 영상 구독자
        self.create_subscription(Image, image_topic, self._on_image, 10)

        self.get_logger().info(f'삼각대 영상 구독 토픽: {image_topic}')
        self.get_logger().info(f'삼각대 감지 결과 발행 토픽: {target_topic}')

    def _empty_detection(self, image_msg):
        """미감지 메시지 생성."""
        # 기본값: 감지 안 됨
        target = Target2D()
        target.header = image_msg.header
        target.detected = False
        target.source = 'tripod_camera'
        target.class_name = self._target_class_name
        target.confidence = 0.0
        target.track_id = -1
        return target

    def _on_image(self, image_msg):
        # 기본 미감지 결과 생성
        target = self._empty_detection(image_msg)

        # YOLO 모델 없음 처리
        if self._model is None:
            self._publisher.publish(target)
            return

        # ROS Image의 OpenCV 이미지 변환
        frame = self._bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')

        # YOLO 추론
        results = self._model.predict(frame, verbose=False)

        # 가장 큰 target 박스 선택
        box = biggest_box(results[0], self._target_class_name)

        # 신뢰도 기준 통과 시 감지 결과 작성
        if box and box['confidence'] >= self._confidence_threshold:
            target.detected = True
            target.class_name = box['class_name']
            target.confidence = box['confidence']
            target.track_id = box['track_id']
            target.center_x = box['center_x']
            target.center_y = box['center_y']
            target.width = box['width']
            target.height = box['height']

        # 2D 감지 결과 발행
        self._publisher.publish(target)


def main():
    rclpy.init()
    node = TripodYoloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
