"""고정 USB 웹캠 기반 RC카 등장 신호 노드."""

import time
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool

from rc_car_follower.yolo_helper import (
    biggest_box,
    class_ids_for_name,
    load_yolo_model,
    resolve_yolo_device,
)


class TripodTriggerNode(Node):
    """고정 웹캠으로 RC카 등장 여부를 판단하는 노드."""

    def __init__(self):
        super().__init__('tripod_trigger_node')

        # USB 웹캠 장치 번호
        self.declare_parameter('device_id', 2)

        # USB 재연결에도 유지되는 웹캠 장치 경로 또는 auto
        self.declare_parameter('camera_device', '')

        # YOLO 모델 이름 또는 직접 학습한 모델 경로
        self.declare_parameter('model_path', 'yolov8n.pt')

        # YOLO 추론 장치
        self.declare_parameter('device', 'auto')

        # 추적할 YOLO 클래스 이름
        self.declare_parameter('target_class_name', 'car')

        # car 감지 최소 신뢰도
        self.declare_parameter('confidence_threshold', 0.35)

        # 오탐 방지용 연속 감지 프레임 수
        self.declare_parameter('required_consecutive_detections', 5)

        # 깜빡임 방지용 연속 미감지 프레임 수
        self.declare_parameter('allowed_consecutive_misses', 10)

        # 웹캠 읽기 주기
        self.declare_parameter('frame_rate', 10.0)

        # 웹캠 요청 해상도
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)

        # 디버깅용 화면 표시 여부
        self.declare_parameter('show_window', False)

        # 네트워크 보호용 디버그 영상 발행 여부
        self.declare_parameter('publish_debug_image', False)
        self.declare_parameter('publish_debug_compressed_image', False)

        # RC카 등장 신호 토픽
        self.declare_parameter('detected_topic', '/rc_car/webcam_detected')

        # 디버깅용 웹캠 YOLO 화면 토픽
        self.declare_parameter(
            'debug_image_topic',
            '/rc_car/debug/tripod_image',
        )

        # 디버깅용 압축 웹캠 YOLO 화면 토픽
        self.declare_parameter(
            'debug_compressed_image_topic',
            '/rc_car/debug/tripod_image/compressed',
        )

        self._device_id = int(self.get_parameter('device_id').value)
        self._camera_device = str(
            self.get_parameter('camera_device').value
        ).strip()
        self._model_path = self.get_parameter('model_path').value
        self._device = resolve_yolo_device(
            self,
            self.get_parameter('device').value,
        )
        self._target_class_name = self.get_parameter('target_class_name').value
        self._confidence_threshold = float(
            self.get_parameter('confidence_threshold').value
        )
        self._required_count = int(
            self.get_parameter('required_consecutive_detections').value
        )
        self._allowed_miss_count = int(
            self.get_parameter('allowed_consecutive_misses').value
        )
        self._frame_rate = float(self.get_parameter('frame_rate').value)
        self._image_width = int(self.get_parameter('image_width').value)
        self._image_height = int(self.get_parameter('image_height').value)
        self._show_window = bool(self.get_parameter('show_window').value)
        self._publish_debug_image_enabled = bool(
            self.get_parameter('publish_debug_image').value
        )
        self._publish_debug_compressed_image_enabled = bool(
            self.get_parameter('publish_debug_compressed_image').value
        )
        self._needs_debug_frame = (
            self._show_window
            or self._publish_debug_image_enabled
            or self._publish_debug_compressed_image_enabled
        )
        detected_topic = self.get_parameter('detected_topic').value
        debug_image_topic = self.get_parameter('debug_image_topic').value
        debug_compressed_image_topic = self.get_parameter(
            'debug_compressed_image_topic'
        ).value

        # ROS 이미지 변환 도구
        self._bridge = CvBridge()

        # 연속 감지 성공 횟수
        self._consecutive_detect_count = 0

        # 연속 감지 실패 횟수
        self._consecutive_miss_count = 0

        # 현재 trigger 상태
        self._detected_now = False

        # 로그 출력 시간 제한
        self._last_log_time = 0.0

        self._model = load_yolo_model(self, self._model_path)
        self._target_class_ids = class_ids_for_name(
            self,
            self._model,
            self._target_class_name,
        )
        self._camera = self._open_camera()
        self._publisher = self.create_publisher(Bool, detected_topic, 10)
        debug_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._debug_image_publisher = None
        if self._publish_debug_image_enabled:
            self._debug_image_publisher = self.create_publisher(
                Image,
                debug_image_topic,
                debug_qos,
            )

        self._debug_compressed_image_publisher = None
        if self._publish_debug_compressed_image_enabled:
            self._debug_compressed_image_publisher = self.create_publisher(
                CompressedImage,
                debug_compressed_image_topic,
                debug_qos,
            )

        timer_period = 1.0 / max(self._frame_rate, 1.0)
        self.create_timer(timer_period, self._timer_callback)

        self.get_logger().info(f'웹캠 감지 신호 토픽: {detected_topic}')
        if self._debug_image_publisher is not None:
            self.get_logger().info(f'웹캠 디버그 화면 토픽: {debug_image_topic}')
        if self._debug_compressed_image_publisher is not None:
            self.get_logger().info(
                f'웹캠 압축 디버그 화면 토픽: {debug_compressed_image_topic}'
            )
        self.get_logger().info(f'웹캠 장치: {self._camera_source_text()}')
        self.get_logger().info('디버그 영상 발행: 기본 OFF')

    def _camera_source(self):
        """OpenCV에 넘길 웹캠 입력 선택."""
        if self._camera_device and self._camera_device != 'auto':
            return self._camera_device

        return self._device_id

    def _camera_source_text(self):
        """로그용 웹캠 입력 이름."""
        if self._camera_device:
            return self._camera_device

        return f'/dev/video{self._device_id}'

    def _available_camera_devices(self):
        """현재 보이는 카메라 장치 목록."""
        devices = []

        by_id_dir = Path('/dev/v4l/by-id')
        if by_id_dir.exists():
            devices.extend(str(path) for path in sorted(by_id_dir.iterdir()))

        video_devices = sorted(Path('/dev').glob('video*'))
        devices.extend(str(path) for path in video_devices)
        return devices

    def _log_available_camera_devices(self):
        """웹캠 열기 실패 시 확인할 장치 목록 출력."""
        devices = self._available_camera_devices()
        if not devices:
            self.get_logger().error('사용 가능한 /dev/video* 장치 없음')
            return

        self.get_logger().error('사용 가능한 카메라 장치:')
        for device in devices:
            self.get_logger().error(f'  {device}')

    def _open_auto_camera(self):
        """보이는 카메라 장치 중 열리는 장치 선택."""
        for device in self._available_camera_devices():
            camera = cv2.VideoCapture(device)
            camera.set(cv2.CAP_PROP_FRAME_WIDTH, self._image_width)
            camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self._image_height)

            if camera.isOpened():
                self._camera_device = device
                self.get_logger().info(f'웹캠 자동 선택: {device}')
                return camera

            camera.release()

        self.get_logger().error('자동 웹캠 선택 실패')
        self._log_available_camera_devices()
        return None

    def _open_camera(self):
        """USB 웹캠 열기."""
        if self._camera_device == 'auto':
            return self._open_auto_camera()

        camera_source = self._camera_source()
        camera_source_text = self._camera_source_text()

        camera = cv2.VideoCapture(camera_source)
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, self._image_width)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self._image_height)

        if not camera.isOpened():
            self.get_logger().error(f'웹캠 열기 실패: {camera_source_text}')
            self._log_available_camera_devices()
            return None

        self.get_logger().info(f'웹캠 열기 완료: {camera_source_text}')
        return camera

    def _publish_detected(self, detected):
        """RC카 등장 신호 발행."""
        message = Bool()
        message.data = detected
        self._publisher.publish(message)

    def _timer_callback(self):
        """웹캠 프레임 처리 주기."""
        if self._model is None or self._camera is None:
            self._publish_detected(False)
            return

        frame_ok, frame = self._camera.read()
        if not frame_ok:
            self.get_logger().warn('웹캠 프레임 읽기 실패')
            self._publish_detected(False)
            return

        car_detected, debug_frame = self._detect_car(frame)
        self._update_trigger_state(car_detected)
        self._publish_detected(self._detected_now)
        if debug_frame is not None:
            self._draw_state_text(debug_frame)
            self._publish_debug_image(debug_frame)
            self._publish_debug_compressed_image(debug_frame)
            self._show_debug_window(debug_frame)
        self._log_state()

    def _detect_car(self, frame):
        """YOLO car 감지 판단 및 표시 화면 생성."""
        results = self._model.predict(
            frame,
            classes=self._target_class_ids,
            conf=self._confidence_threshold,
            device=self._device,
            verbose=False,
        )

        # target class만 trigger로 사용
        box = biggest_box(results[0], self._target_class_name)
        car_detected = bool(
            box and box['confidence'] >= self._confidence_threshold
        )

        # 디버그 영상이 필요할 때만 박스 화면 생성
        debug_frame = None
        if self._needs_debug_frame:
            debug_frame = results[0].plot()

        return car_detected, debug_frame

    def _update_trigger_state(self, car_detected):
        """연속 감지와 연속 미감지 기준 적용."""
        if car_detected:
            self._consecutive_detect_count += 1
            self._consecutive_miss_count = 0
        else:
            self._consecutive_detect_count = 0
            self._consecutive_miss_count += 1

        # trigger 켜짐 기준
        if self._consecutive_detect_count >= self._required_count:
            self._detected_now = True

        # trigger 꺼짐 기준
        if self._consecutive_miss_count >= self._allowed_miss_count:
            self._detected_now = False

    def _draw_state_text(self, frame):
        """디버깅용 상태 글자 표시."""
        status_text = 'DETECTED' if self._detected_now else 'WAITING'
        cv2.putText(
            frame,
            status_text,
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1.0,
            (0, 220, 0),
            2,
        )

    def _publish_debug_image(self, frame):
        """디버깅용 웹캠 화면 발행."""
        if self._debug_image_publisher is None:
            return

        image_msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        image_msg.header.stamp = self.get_clock().now().to_msg()
        image_msg.header.frame_id = 'tripod_camera'
        self._debug_image_publisher.publish(image_msg)

    def _publish_debug_compressed_image(self, frame):
        """디버깅용 압축 웹캠 화면 발행."""
        if self._debug_compressed_image_publisher is None:
            return

        compressed_msg = self._bridge.cv2_to_compressed_imgmsg(
            frame,
            dst_format='jpg',
        )
        compressed_msg.header.stamp = self.get_clock().now().to_msg()
        compressed_msg.header.frame_id = 'tripod_camera'
        self._debug_compressed_image_publisher.publish(compressed_msg)

    def _show_debug_window(self, frame):
        """디버깅용 웹캠 창 표시."""
        if not self._show_window:
            return

        cv2.imshow('tripod_trigger_node', frame)
        cv2.waitKey(1)

    def _log_state(self):
        """1초 간격 상태 로그."""
        now = time.monotonic()
        if now - self._last_log_time < 1.0:
            return

        self._last_log_time = now
        self.get_logger().info(
            f'RC카 감지 상태: {self._detected_now}, '
            f'연속 감지: {self._consecutive_detect_count}/{self._required_count}, '
            f'연속 미감지: '
            f'{self._consecutive_miss_count}/{self._allowed_miss_count}'
        )

    def destroy_node(self):
        """웹캠 자원 정리."""
        if self._camera is not None:
            self._camera.release()
        if self._show_window:
            cv2.destroyAllWindows()
        super().destroy_node()


def main():
    rclpy.init()
    node = TripodTriggerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
