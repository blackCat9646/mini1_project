"""웹캠 trigger 기반 관찰 지점 이동 노드."""

import math

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool


class ApproachPlannerNode(Node):
    """웹캠 감지 신호를 Nav2 이동 goal로 바꾸는 노드."""

    def __init__(self):
        super().__init__('approach_planner_node')

        # 웹캠 RC카 감지 신호 토픽
        self.declare_parameter('trigger_topic', '/rc_car/webcam_detected')

        # 관찰 지점 waypoint 파일
        self.declare_parameter(
            'waypoint_file',
            '/home/rokey/mini1_ws/src/rc_car_bringup/config/watch_area_waypoint.yaml',
        )

        # TurtleBot4 Nav2 action 이름
        self.declare_parameter('navigate_action', '/robot3/navigate_to_pose')

        # Nav2 기반 OAK-D 추적 켜기/끄기 토픽
        self.declare_parameter('tracking_enable_topic', '/rc_car/nav2_tracking_enabled')

        # goal 완료 후 trigger가 다시 false가 되었을 때 재출발 허용 여부
        self.declare_parameter('rearm_when_trigger_is_false', True)

        trigger_topic = self.get_parameter('trigger_topic').value
        waypoint_file = self.get_parameter('waypoint_file').value
        navigate_action = self.get_parameter('navigate_action').value
        tracking_enable_topic = self.get_parameter('tracking_enable_topic').value

        self._rearm_when_trigger_is_false = bool(
            self.get_parameter('rearm_when_trigger_is_false').value
        )

        # 관찰 지점 좌표
        self._waypoint = self._load_watch_area_waypoint(waypoint_file)

        # 중복 goal 전송 방지 상태
        self._goal_in_progress = False
        self._goal_sent = False

        # 이전 trigger 상태
        self._last_trigger_value = False

        self._action_client = ActionClient(self, NavigateToPose, navigate_action)
        self._tracking_enable_publisher = self.create_publisher(
            Bool,
            tracking_enable_topic,
            10,
        )
        self.create_subscription(Bool, trigger_topic, self._on_trigger, 10)

        self.get_logger().info(f'웹캠 trigger 토픽: {trigger_topic}')
        self.get_logger().info(f'관찰 지점 waypoint 파일: {waypoint_file}')
        self.get_logger().info(f'Nav2 이동 action: {navigate_action}')
        self.get_logger().info(f'OAK-D Nav2 추적 enable 토픽: {tracking_enable_topic}')

    def _publish_tracking_enabled(self, enabled):
        """OAK-D Nav2 추적 시작/정지 신호 발행."""
        message = Bool()
        message.data = enabled
        self._tracking_enable_publisher.publish(message)
        self.get_logger().info(f'OAK-D Nav2 추적 enable: {enabled}')

    def _load_watch_area_waypoint(self, waypoint_file):
        """관찰 지점 waypoint 파일 읽기."""
        try:
            with open(waypoint_file, 'r', encoding='utf-8') as file:
                data = yaml.safe_load(file)
        except OSError as error:
            self.get_logger().error(f'waypoint 파일 읽기 실패: {error}')
            return None

        watch_area = data.get('watch_area', {})
        required_keys = ['frame_id', 'x', 'y', 'yaw']
        missing_keys = [key for key in required_keys if key not in watch_area]
        if missing_keys:
            self.get_logger().error(f'waypoint 필수 값 없음: {missing_keys}')
            return None

        self.get_logger().info(
            '관찰 지점 로딩 완료: '
            f"x={watch_area['x']:.2f}, y={watch_area['y']:.2f}, yaw={watch_area['yaw']:.2f}"
        )
        return watch_area

    def _on_trigger(self, message):
        """웹캠 trigger 수신 처리."""
        trigger_is_on = bool(message.data)

        # trigger false 시 다음 테스트 준비
        if not trigger_is_on:
            self._last_trigger_value = False
            if self._rearm_when_trigger_is_false and not self._goal_in_progress:
                self._goal_sent = False
            return

        # false에서 true로 바뀌는 순간만 사용
        is_rising_edge = trigger_is_on and not self._last_trigger_value
        self._last_trigger_value = True

        if not is_rising_edge:
            return

        # waypoint 파일 문제 시 이동 금지
        if self._waypoint is None:
            self.get_logger().error('관찰 지점 없음으로 goal 전송 불가')
            return

        # 이미 이동 중일 때 중복 goal 금지
        if self._goal_in_progress:
            self.get_logger().info('이미 Nav2 이동 중')
            return

        # 이미 goal을 보낸 뒤 trigger가 계속 true인 경우 중복 goal 금지
        if self._goal_sent:
            self.get_logger().info('이미 관찰 지점 goal 전송 완료')
            return

        self._send_watch_area_goal()

    def _send_watch_area_goal(self):
        """관찰 지점 Nav2 goal 전송."""
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self._waypoint['frame_id']
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(self._waypoint['x'])
        goal.pose.pose.position.y = float(self._waypoint['y'])
        goal.pose.pose.position.z = 0.0

        # yaw 값의 quaternion 변환
        yaw = float(self._waypoint['yaw'])
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(
            '관찰 지점 이동 goal 전송: '
            f"x={goal.pose.pose.position.x:.2f}, "
            f"y={goal.pose.pose.position.y:.2f}, "
            f"yaw={yaw:.2f}"
        )

        self._goal_in_progress = True
        self._goal_sent = True
        self._publish_tracking_enabled(False)
        self._action_client.wait_for_server()
        goal_future = self._action_client.send_goal_async(goal)
        goal_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        """Nav2 goal 수락 여부 확인."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Nav2 goal 거절')
            self._goal_in_progress = False
            return

        self.get_logger().info('Nav2 goal 수락')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        """Nav2 goal 결과 확인."""
        status = future.result().status
        self.get_logger().info(f'Nav2 goal 종료 상태: {status}')
        self._goal_in_progress = False
        self._publish_tracking_enabled(status == GoalStatus.STATUS_SUCCEEDED)


def main():
    rclpy.init()
    node = ApproachPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
