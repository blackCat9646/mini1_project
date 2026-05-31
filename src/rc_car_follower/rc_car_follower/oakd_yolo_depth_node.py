"""OAK-D 카메라 RC카 감지 및 거리 계산 노드.

입력:
  /robot3/oakd/rgb/preview/image_raw   OAK-D 컬러 영상
  /robot3/oakd/stereo/image_raw        OAK-D 깊이 영상

출력:
  /rc_car/target/oakd_3d               RC카 감지 결과

역할:
  1. 컬러 영상에서 YOLO로 car 박스 찾기
  2. 박스 중심 위치의 깊이값 읽기
  3. RC카 방향과 거리 publish
  4. 오래된 프레임 누적 방지
"""

import math

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from rc_car_interfaces.msg import Target3D
from rc_car_follower.yolo_helper import biggest_box, load_yolo_model, resolve_yolo_device


class OakdYoloDepthNode(Node):
    """근거리 추종용 YOLO + Depth 노드."""

    def __init__(self):
        super().__init__('oakd_yolo_depth_node')

        # 파라미터 기본값
        self.declare_parameter('rgb_topic', '/robot3/oakd/rgb/preview/image_raw')
        self.declare_parameter('depth_topic', '/robot3/oakd/stereo/image_raw')
        self.declare_parameter('target_topic', '/rc_car/target/oakd_3d')
        self.declare_parameter('model_path', '')
        self.declare_parameter('device', 'auto')
        self.declare_parameter('target_class_name', 'car')
        self.declare_parameter('confidence_threshold', 0.35)
        self.declare_parameter('horizontal_fov_deg', 69.0)
        self.declare_parameter('inference_rate', 5.0)
        self.declare_parameter('depth_sample_radius', 4)

        rgb_topic = self.get_parameter('rgb_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        target_topic = self.get_parameter('target_topic').value
        model_path = self.get_parameter('model_path').value

        self._device = resolve_yolo_device(self, self.get_parameter('device').value)
        self._target_class_name = self.get_parameter('target_class_name').value
        self._confidence_threshold = self.get_parameter('confidence_threshold').value
        self._horizontal_fov_rad = math.radians(self.get_parameter('horizontal_fov_deg').value)
        self._depth_sample_radius = int(self.get_parameter('depth_sample_radius').value)
        inference_rate = float(self.get_parameter('inference_rate').value)
        self._bridge = CvBridge()
        self._model = load_yolo_model(self, model_path)
        self._last_rgb_image = None
        self._last_depth_image = None
        self._last_processed_rgb_stamp = None

        # 카메라용 최신 프레임 QoS
        image_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # 추론 타이머 주기
        timer_period = 1.0 / max(inference_rate, 0.1)

        self._publisher = self.create_publisher(Target3D, target_topic, 10)
        self.create_subscription(Image, rgb_topic, self._on_rgb_image, image_qos)
        self.create_subscription(Image, depth_topic, self._on_depth_image, image_qos)
        self.create_timer(timer_period, self._process_latest_rgb_image)

        self.get_logger().info(f'OAK-D RGB 구독 토픽: {rgb_topic}')
        self.get_logger().info(f'OAK-D Depth 구독 토픽: {depth_topic}')
        self.get_logger().info(f'OAK-D 감지 결과 발행 토픽: {target_topic}')
        self.get_logger().info(f'YOLO 추론 주기: {inference_rate:.1f} Hz')
        self.get_logger().info('카메라 QoS: BEST_EFFORT, queue 1')

    def _on_depth_image(self, image_msg):
        # 최신 깊이 영상 저장
        self._last_depth_image = image_msg

    def _on_rgb_image(self, image_msg):
        # 최신 컬러 영상 저장
        self._last_rgb_image = image_msg

    def _empty_target(self, rgb_msg):
        # 미감지 결과 메시지 생성
        target = Target3D()
        target.header.stamp = rgb_msg.header.stamp
        target.header.frame_id = 'base_link'
        target.detected = False
        target.source = 'oakd_depth'
        target.class_name = self._target_class_name
        target.confidence = 0.0
        target.track_id = -1
        target.distance = -1.0
        return target

    def _depth_at_pixel(self, pixel_x, pixel_y, rgb_width, rgb_height):
        """YOLO 박스 중심 픽셀의 깊이값 읽기."""
        if self._last_depth_image is None:
            return None

        # 깊이 영상 변환
        depth = self._bridge.imgmsg_to_cv2(self._last_depth_image)
        depth_height, depth_width = depth.shape[:2]

        # RGB 좌표의 Depth 좌표 변환
        scale_x = depth_width / max(float(rgb_width), 1.0)
        scale_y = depth_height / max(float(rgb_height), 1.0)
        depth_x = pixel_x * scale_x
        depth_y = pixel_y * scale_y

        # 중심 주변 작은 영역 선택
        radius = max(self._depth_sample_radius, 0)
        center_x = max(0, min(int(depth_x), depth_width - 1))
        center_y = max(0, min(int(depth_y), depth_height - 1))
        x1 = max(center_x - radius, 0)
        x2 = min(center_x + radius + 1, depth_width)
        y1 = max(center_y - radius, 0)
        y2 = min(center_y + radius + 1, depth_height)
        depth_patch = depth[y1:y2, x1:x2].astype(np.float32)

        # mm 단위 깊이값의 m 단위 변환
        if depth_patch.size > 0 and float(np.nanmax(depth_patch)) > 20.0:
            depth_patch = depth_patch / 1000.0

        # 정상 깊이값만 선택
        valid_depth = depth_patch[np.isfinite(depth_patch)]
        valid_depth = valid_depth[valid_depth > 0.05]
        valid_depth = valid_depth[valid_depth < 10.0]

        if valid_depth.size == 0:
            return None

        # 튀는 한 픽셀 방지용 중앙값
        return float(np.median(valid_depth))

    def _process_latest_rgb_image(self):
        # 최신 컬러 영상 없음 처리
        if self._last_rgb_image is None:
            return

        rgb_msg = self._last_rgb_image
        rgb_stamp = (rgb_msg.header.stamp.sec, rgb_msg.header.stamp.nanosec)
        if rgb_stamp == self._last_processed_rgb_stamp:
            return
        self._last_processed_rgb_stamp = rgb_stamp

        # 기본값: 미감지 상태
        target = self._empty_target(rgb_msg)

        # YOLO 모델 없음 처리
        if self._model is None:
            self._publisher.publish(target)
            return

        # ROS 영상 메시지의 OpenCV 영상 변환
        frame = self._bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        image_height = frame.shape[0]
        image_width = frame.shape[1]

        # YOLO car 감지
        results = self._model.predict(frame, device=self._device, verbose=False)
        box = biggest_box(results[0], self._target_class_name)

        # 미감지 또는 낮은 신뢰도 처리
        if not box or box['confidence'] < self._confidence_threshold:
            self._publisher.publish(target)
            return

        # RC카 중심 거리 계산
        distance = self._depth_at_pixel(
            box['center_x'],
            box['center_y'],
            image_width,
            image_height,
        )
        if distance is None:
            self._publisher.publish(target)
            return

        # 좌우 각도 오차 계산
        image_center_x = image_width / 2.0
        normalized_error = (image_center_x - box['center_x']) / image_center_x
        yaw_error = normalized_error * (self._horizontal_fov_rad / 2.0)

        # 감지 결과 메시지 작성
        target.detected = True
        target.class_name = box['class_name']
        target.confidence = box['confidence']
        target.track_id = box['track_id']
        target.distance = distance
        target.yaw_error = yaw_error
        target.position = Point(
            x=distance * math.cos(yaw_error),
            y=distance * math.sin(yaw_error),
            z=0.0,
        )
        self._publisher.publish(target)


def main():
    rclpy.init()
    node = OakdYoloDepthNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
