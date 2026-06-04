"""Nav2 기반 RC카 추적 노드.

입력:
  /rc_car/target/oakd_3d          OAK-D RC카 위치
  /rc_car/nav2_tracking_enabled   추적 시작/정지 신호

출력:
  /robot3/navigate_to_pose        Nav2 이동 goal
  /rc_car/return_to_watch_area    RC카 분실 시 관찰 지점 복귀 요청

규칙:
  1. tracking_enabled가 false이면 Nav2 goal 전송 금지
  2. RC카가 보이면 RC카 앞 0.70m 지점까지 Nav2로 이동
  3. RC카가 너무 가까우면 진행 중인 Nav2 goal 취소
  4. RC카를 잃어버리면 진행 중인 Nav2 goal 취소
  5. RC카 미감지 프레임이 누적되면 관찰 지점 복귀 요청
  6. 실제 장애물 회피는 Nav2 costmap과 controller 담당
"""

import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from irobot_create_msgs.msg import AudioNote, AudioNoteVector
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener

from rc_car_interfaces.msg import Target3D


class Nav2TargetTrackerNode(Node):
    """OAK-D 감지 결과를 Nav2 추적 goal로 바꾸는 노드."""

    def __init__(self):
        super().__init__('nav2_target_tracker_node')

        # 토픽과 action 이름
        self.declare_parameter('target_topic', '/rc_car/target/oakd_3d')
        self.declare_parameter('enable_topic', '/rc_car/nav2_tracking_enabled')
        self.declare_parameter('return_topic', '/rc_car/return_to_watch_area')
        self.declare_parameter('navigate_action', '/robot3/navigate_to_pose')

        # 좌표계 이름
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_link')

        # 추적 기본 상태
        self.declare_parameter('tracking_enabled', False)

        # RC카와 유지할 거리
        self.declare_parameter('desired_distance', 0.70)

        # 이 거리보다 가까우면 Nav2 이동 goal 취소
        self.declare_parameter('stop_distance', 0.70)

        # 이 거리보다 멀 때만 새 Nav2 goal 생성
        self.declare_parameter('approach_margin', 0.15)

        # 감지 끊김 판단 시간
        self.declare_parameter('target_timeout_sec', 1.0)

        # Nav2 goal 갱신 주기
        self.declare_parameter('goal_update_period', 1.0)

        # goal 변화가 작을 때 재전송 방지 거리
        self.declare_parameter('goal_change_distance', 0.20)

        # RC카 분실 시 관찰 지점 복귀 기준
        self.declare_parameter('return_when_lost', True)
        self.declare_parameter('return_missed_frames', 5)

        # 추적 중 RC카 감지 알림음
        self.declare_parameter('audio_topic', '/robot3/cmd_audio')
        self.declare_parameter('beep_on_tracking', True)

        target_topic = self.get_parameter('target_topic').value
        enable_topic = self.get_parameter('enable_topic').value
        return_topic = self.get_parameter('return_topic').value
        navigate_action = self.get_parameter('navigate_action').value
        audio_topic = self.get_parameter('audio_topic').value

        self._map_frame = self.get_parameter('map_frame').value
        self._robot_frame = self.get_parameter('robot_frame').value
        self._tracking_enabled = bool(
            self.get_parameter('tracking_enabled').value
        )
        self._desired_distance = float(
            self.get_parameter('desired_distance').value
        )
        self._stop_distance = float(self.get_parameter('stop_distance').value)
        self._approach_margin = float(
            self.get_parameter('approach_margin').value
        )
        self._target_timeout_sec = float(
            self.get_parameter('target_timeout_sec').value
        )
        self._goal_change_distance = float(
            self.get_parameter('goal_change_distance').value
        )
        goal_update_period = float(
            self.get_parameter('goal_update_period').value
        )
        self._return_when_lost = bool(
            self.get_parameter('return_when_lost').value
        )
        self._return_missed_frames = int(
            self.get_parameter('return_missed_frames').value
        )
        self._beep_on_tracking = bool(
            self.get_parameter('beep_on_tracking').value
        )

        # 최신 RC카 위치
        self._last_target = None
        self._last_target_time = 0.0

        # Nav2 goal 상태
        self._goal_handle = None
        self._cancel_in_progress = False
        self._pending_goal = None
        self._last_goal_xy = None
        self._beep_published = False
        self._missed_detection_count = 0
        self._return_requested = False

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._action_client = ActionClient(
            self,
            NavigateToPose,
            navigate_action,
        )
        self._audio_publisher = self.create_publisher(
            AudioNoteVector,
            audio_topic,
            10,
        )
        self._return_publisher = self.create_publisher(
            Bool,
            return_topic,
            10,
        )

        self.create_subscription(Target3D, target_topic, self._on_target, 10)
        self.create_subscription(Bool, enable_topic, self._on_enable, 10)
        self.create_timer(max(goal_update_period, 0.2), self._tracking_loop)

        self.get_logger().info(f'RC카 위치 구독 토픽: {target_topic}')
        self.get_logger().info(f'추적 enable 토픽: {enable_topic}')
        self.get_logger().info(f'관찰 지점 복귀 요청 토픽: {return_topic}')
        self.get_logger().info(f'Nav2 이동 action: {navigate_action}')
        self.get_logger().info(f'추적 감지 알림음 토픽: {audio_topic}')
        self.get_logger().info(f'초기 Nav2 추적 상태: {self._tracking_enabled}')

    def _on_target(self, target):
        # 최신 RC카 위치 저장
        self._last_target = target
        self._last_target_time = time.monotonic()

        if not self._tracking_enabled:
            self._missed_detection_count = 0
            return

        if target.detected:
            self._missed_detection_count = 0
            return

        self._missed_detection_count += 1
        self.get_logger().info(
            'RC카 미감지 프레임: '
            f'{self._missed_detection_count}/{self._return_missed_frames}'
        )
        if self._missed_detection_count >= self._return_missed_frames:
            self._request_watch_area_return()

    def _on_enable(self, message):
        # watch_area 도착 후 추적 시작
        self._tracking_enabled = bool(message.data)
        self.set_parameters([
            Parameter(
                'tracking_enabled',
                Parameter.Type.BOOL,
                self._tracking_enabled,
            )
        ])
        self.get_logger().info(f'Nav2 추적 상태: {self._tracking_enabled}')
        self._missed_detection_count = 0
        self._return_requested = False
        if not self._tracking_enabled:
            self._cancel_current_goal()
            self._beep_published = False

    def _tracking_loop(self):
        # 수동 파라미터 변경 반영
        self._tracking_enabled = bool(
            self.get_parameter('tracking_enabled').value
        )

        if not self._tracking_enabled:
            return

        if not self._target_is_fresh_and_detected():
            self._cancel_current_goal()
            return

        self._publish_tracking_beep()

        stop_limit = self._stop_distance + self._approach_margin
        if self._last_target.distance <= stop_limit:
            self._cancel_current_goal()
            return

        goal = self._make_follow_goal(self._last_target)
        if goal is None:
            return

        if not self._goal_changed_enough(goal):
            return

        self._send_or_replace_goal(goal)

    def _target_is_fresh_and_detected(self):
        """최근 OAK-D target이 실제 감지인지 확인."""
        if self._last_target is None:
            return False

        target_age = time.monotonic() - self._last_target_time
        if target_age > self._target_timeout_sec:
            return False

        return bool(self._last_target.detected)

    def _make_follow_goal(self, target):
        """RC카 앞 0.70m 지점의 map goal 생성."""
        target_x = float(target.position.x)
        target_y = float(target.position.y)
        target_distance = math.hypot(target_x, target_y)
        if target_distance <= 0.01:
            return None

        # base_link 기준 목표 위치
        move_distance = max(target_distance - self._desired_distance, 0.0)
        unit_x = target_x / target_distance
        unit_y = target_y / target_distance
        goal_base_x = unit_x * move_distance
        goal_base_y = unit_y * move_distance

        try:
            transform = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._robot_frame,
                rclpy.time.Time(),
            )
        except TransformException as error:
            self.get_logger().warn(f'TF 조회 실패: {error}')
            return None

        goal_map_x, goal_map_y = self._base_point_to_map(
            goal_base_x,
            goal_base_y,
            transform,
        )
        target_map_x, target_map_y = self._base_point_to_map(
            target_x,
            target_y,
            transform,
        )

        # goal 지점에서 RC카를 바라보는 방향
        yaw = math.atan2(target_map_y - goal_map_y, target_map_x - goal_map_x)

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self._map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = goal_map_x
        goal.pose.pose.position.y = goal_map_y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        return goal

    def _base_point_to_map(self, x, y, transform):
        """base_link 점의 map 좌표 변환."""
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = self._yaw_from_quaternion(rotation)
        map_x = translation.x + math.cos(yaw) * x - math.sin(yaw) * y
        map_y = translation.y + math.sin(yaw) * x + math.cos(yaw) * y
        return map_x, map_y

    def _yaw_from_quaternion(self, quaternion):
        """quaternion의 yaw 계산."""
        siny_cosp = 2.0 * (
            quaternion.w * quaternion.z + quaternion.x * quaternion.y
        )
        cosy_cosp = 1.0 - 2.0 * (
            quaternion.y * quaternion.y + quaternion.z * quaternion.z
        )
        return math.atan2(siny_cosp, cosy_cosp)

    def _goal_changed_enough(self, goal):
        """작은 goal 변화 무시."""
        goal_xy = (
            goal.pose.pose.position.x,
            goal.pose.pose.position.y,
        )
        if self._last_goal_xy is None:
            return True

        change = math.hypot(
            goal_xy[0] - self._last_goal_xy[0],
            goal_xy[1] - self._last_goal_xy[1],
        )
        return change >= self._goal_change_distance

    def _send_or_replace_goal(self, goal):
        """Nav2 goal 전송 또는 교체."""
        if not self._action_client.server_is_ready():
            self.get_logger().warn('Nav2 action server 대기 중')
            return

        if self._goal_handle is not None:
            self._pending_goal = goal
            self._cancel_current_goal()
            return

        self._send_goal(goal)

    def _send_goal(self, goal):
        """Nav2 goal 전송."""
        self._last_goal_xy = (
            goal.pose.pose.position.x,
            goal.pose.pose.position.y,
        )
        self.get_logger().info(
            'Nav2 추적 goal 전송: '
            f"x={self._last_goal_xy[0]:.2f}, y={self._last_goal_xy[1]:.2f}"
        )
        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

    def _cancel_current_goal(self):
        """진행 중인 Nav2 goal 취소."""
        if self._goal_handle is None or self._cancel_in_progress:
            return

        self._cancel_in_progress = True
        future = self._goal_handle.cancel_goal_async()
        future.add_done_callback(self._on_cancel_done)

    def _on_goal_response(self, future):
        """Nav2 goal 수락 확인."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Nav2 추적 goal 거절')
            self._goal_handle = None
            return

        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        """Nav2 goal 종료 처리."""
        status = future.result().status
        self.get_logger().info(f'Nav2 추적 goal 종료 상태: {status}')
        self._goal_handle = None
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._last_goal_xy = None

    def _on_cancel_done(self, future):
        """Nav2 goal 취소 후 대기 goal 전송."""
        self._cancel_in_progress = False
        self._goal_handle = None
        next_goal = self._pending_goal
        self._pending_goal = None
        if next_goal is not None and self._tracking_enabled:
            self._send_goal(next_goal)

    def _request_watch_area_return(self):
        """RC카 분실 시 관찰 지점 복귀 요청."""
        if not self._return_when_lost:
            return

        if self._return_requested:
            return

        self._return_requested = True
        self._cancel_current_goal()

        message = Bool()
        message.data = True
        self._return_publisher.publish(message)
        self.get_logger().warn('RC카 분실로 관찰 지점 복귀 요청')

    def _audio_note(self, frequency):
        """짧은 알림음 하나 생성."""
        note = AudioNote()
        note.frequency = int(frequency)
        note.max_runtime = Duration(sec=0, nanosec=300000000)
        return note

    def _publish_tracking_beep(self):
        """추적 중 RC카가 처음 보일 때만 알림음 발행."""
        if not self._beep_on_tracking:
            return

        if self._beep_published:
            return

        self._beep_published = True
        sound = AudioNoteVector()
        sound.notes = [
            self._audio_note(880),
            self._audio_note(440),
            self._audio_note(880),
            self._audio_note(440),
        ]
        self._audio_publisher.publish(sound)


def main():
    rclpy.init()
    node = Nav2TargetTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
