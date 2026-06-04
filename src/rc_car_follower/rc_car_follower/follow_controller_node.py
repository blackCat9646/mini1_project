"""RC카 근거리 추종 제어 노드.

입력:
  /rc_car/target/oakd_3d   OAK-D RC카 위치
  /robot3/scan             라이다 거리

출력:
  /robot3/cmd_vel          로봇 속도 명령

규칙:
  1. follow_enabled가 false이면 항상 정지
  2. RC카가 멀면 천천히 전진
  3. RC카가 가까우면 정지
  4. RC카가 좌우로 치우치면 회전
  5. 라이다 앞쪽이 위험하면 즉시 정지
"""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from rc_car_interfaces.msg import Target3D


class FollowControllerNode(Node):
    """RC카를 천천히 따라가는 제어 노드."""

    def __init__(self):
        super().__init__('follow_controller_node')

        # 토픽 이름
        self.declare_parameter('target_topic', '/rc_car/target/oakd_3d')
        self.declare_parameter('scan_topic', '/robot3/scan')
        self.declare_parameter('cmd_vel_topic', '/robot3/cmd_vel')

        # 추종 기능 켜기/끄기
        self.declare_parameter('follow_enabled', False)

        # 목표 유지 거리
        self.declare_parameter('desired_distance', 0.70)

        # 너무 가까운 거리
        self.declare_parameter('stop_distance', 0.70)

        # 라이다 전방 안전 거리
        self.declare_parameter('front_safety_distance', 0.30)

        # 감지 끊김 판단 시간
        self.declare_parameter('target_timeout_sec', 1.0)

        # 저속 테스트용 최대 속도
        self.declare_parameter('max_linear_speed', 0.08)
        self.declare_parameter('max_angular_speed', 0.40)

        target_topic = self.get_parameter('target_topic').value
        scan_topic = self.get_parameter('scan_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self._desired_distance = self.get_parameter('desired_distance').value
        self._stop_distance = self.get_parameter('stop_distance').value
        self._front_safety_distance = self.get_parameter(
            'front_safety_distance'
        ).value
        self._target_timeout_sec = self.get_parameter(
            'target_timeout_sec'
        ).value
        self._max_linear_speed = self.get_parameter('max_linear_speed').value
        self._max_angular_speed = self.get_parameter('max_angular_speed').value

        self._last_target = None
        self._last_target_time = 0.0
        self._front_is_safe = True
        self._follow_was_enabled = False

        self._cmd_publisher = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.create_subscription(Target3D, target_topic, self._on_target, 10)
        self.create_subscription(LaserScan, scan_topic, self._on_scan, 10)
        self.create_timer(0.10, self._control_loop)

        self.get_logger().info(f'RC카 위치 구독 토픽: {target_topic}')
        self.get_logger().info(f'라이다 구독 토픽: {scan_topic}')
        self.get_logger().info(f'속도 명령 발행 토픽: {cmd_vel_topic}')
        self.get_logger().info(
            '추종 시작 명령: '
            'ros2 param set /follow_controller_node follow_enabled true'
        )
        self.get_logger().info(
            '추종 정지 명령: '
            'ros2 param set /follow_controller_node follow_enabled false'
        )

    def _on_target(self, target):
        # 최신 RC카 위치 저장
        self._last_target = target
        self._last_target_time = time.monotonic()

    def _on_scan(self, scan):
        """전방 60도 라이다 안전 확인."""
        front_ranges = []
        for index, distance in enumerate(scan.ranges):
            angle = scan.angle_min + index * scan.angle_increment
            if abs(angle) > math.radians(30.0):
                continue
            if math.isfinite(distance):
                front_ranges.append(distance)

        if not front_ranges:
            self._front_is_safe = True
            return

        self._front_is_safe = min(front_ranges) > self._front_safety_distance

    def _stop(self):
        # 정지 명령 발행
        self._cmd_publisher.publish(Twist())

    def _control_loop(self):
        # follow_enabled 파라미터 확인
        follow_enabled = bool(self.get_parameter('follow_enabled').value)

        if follow_enabled != self._follow_was_enabled:
            self._follow_was_enabled = follow_enabled
            self.get_logger().info(f'추종 상태: {follow_enabled}')

        if not follow_enabled:
            self._stop()
            return

        # RC카 위치 없음 처리
        if self._last_target is None:
            self._stop()
            return

        # 오래된 감지 또는 미감지 처리
        target_age = time.monotonic() - self._last_target_time
        target_is_old = target_age > self._target_timeout_sec
        if target_is_old or not self._last_target.detected:
            self._stop()
            return

        # 라이다 위험 처리
        if not self._front_is_safe:
            self._stop()
            return

        # 거리 오차 계산
        distance_error = self._last_target.distance - self._desired_distance
        twist = Twist()

        # 전진 속도 계산
        if self._last_target.distance > self._stop_distance:
            twist.linear.x = max(
                0.0,
                min(0.6 * distance_error, self._max_linear_speed),
            )

        # 회전 속도 계산
        twist.angular.z = max(
            -self._max_angular_speed,
            min(1.5 * self._last_target.yaw_error, self._max_angular_speed),
        )
        self._cmd_publisher.publish(twist)


def main():
    rclpy.init()
    node = FollowControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
