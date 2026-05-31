"""고정 USB 웹캠 기반 RC카 등장 신호 노드."""

import time

import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from rc_car_follower.yolo_helper import resolve_yolo_device


class TripodTriggerNode(Node):
    """고정 웹캠으로 RC카 등장 여부를 판단하는 노드."""

    COCO_CAR_CLASS_ID = 2

    def __init__(self):
        super().__init__('tripod_trigger_node')

        # USB 웹캠 장치 번호
        self.declare_parameter('device_id', 2)

        # YOLO 기본 COCO 모델 이름
        self.declare_parameter('model_path', 'yolov8n.pt')

        # YOLO 추론 장치
        self.declare_parameter('device', 'auto')

        # COCO car 감지 최소 신뢰도
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

        # RC카 등장 신호 토픽
        self.declare_parameter('detected_topic', '/rc_car/webcam_detected')

        self._device_id = int(self.get_parameter('device_id').value)
        self._model_path = self.get_parameter('model_path').value
        self._device = resolve_yolo_device(self, self.get_parameter('device').value)
        self._confidence_threshold = float(self.get_parameter('confidence_threshold').value)
        self._required_count = int(self.get_parameter('required_consecutive_detections').value)
        self._allowed_miss_count = int(self.get_parameter('allowed_consecutive_misses').value)
        self._frame_rate = float(self.get_parameter('frame_rate').value)
        self._image_width = int(self.get_parameter('image_width').value)
        self._image_height = int(self.get_parameter('image_height').value)
        self._show_window = bool(self.get_parameter('show_window').value)
        detected_topic = self.get_parameter('detected_topic').value

        # 연속 감지 성공 횟수
        self._consecutive_detect_count = 0

        # 연속 감지 실패 횟수
        self._consecutive_miss_count = 0

        # 현재 trigger 상태
        self._detected_now = False

        # 로그 출력 시간 제한
        self._last_log_time = 0.0

        self._model = self._load_yolo_model()
        self._camera = self._open_camera()
        self._publisher = self.create_publisher(Bool, detected_topic, 10)

        timer_period = 1.0 / max(self._frame_rate, 1.0)
        self.create_timer(timer_period, self._timer_callback)

        self.get_logger().info(f'웹캠 감지 신호 토픽: {detected_topic}')
        self.get_logger().info(f'웹캠 장치 번호: /dev/video{self._device_id}')

    def _load_yolo_model(self):
        """YOLO 모델 로딩."""
        try:
            from ultralytics import YOLO
        except ImportError:
            self.get_logger().error('ultralytics 패키지 없음')
            return None

        try:
            model = YOLO(self._model_path)
        except Exception as error:
            self.get_logger().error(f'YOLO 모델 로딩 실패: {error}')
            return None

        self.get_logger().info(f'YOLO 모델 로딩 완료: {self._model_path}')
        return model

    def _open_camera(self):
        """USB 웹캠 열기."""
        camera = cv2.VideoCapture(self._device_id)
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, self._image_width)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self._image_height)

        if not camera.isOpened():
            self.get_logger().error(f'웹캠 열기 실패: /dev/video{self._device_id}')
            return None

        self.get_logger().info('웹캠 열기 완료')
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

        car_detected = self._detect_car(frame)
        self._update_trigger_state(car_detected)
        self._publish_detected(self._detected_now)
        self._show_debug_window(frame)
        self._log_state()

    def _detect_car(self, frame):
        """YOLO car 감지 판단."""
        results = self._model.predict(
            frame,
            classes=[self.COCO_CAR_CLASS_ID],
            conf=self._confidence_threshold,
            device=self._device,
            verbose=False,
        )

        return len(results[0].boxes) > 0

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

    def _show_debug_window(self, frame):
        """디버깅용 웹캠 화면."""
        if not self._show_window:
            return

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
            f'연속 미감지: {self._consecutive_miss_count}/{self._allowed_miss_count}'
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
