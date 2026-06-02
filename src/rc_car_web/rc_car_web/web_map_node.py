"""ROS 지도 토픽을 웹에서 보기 쉬운 토픽으로 다시 발행하는 노드."""

import rclpy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class WebMapNode(Node):
    """웹 표시용 지도 재발행 노드."""

    def __init__(self):
        super().__init__('web_map_node')

        # 원본 지도 토픽
        self.declare_parameter('map_topic', '/map')

        # 웹 표시용 지도 토픽
        self.declare_parameter('web_map_topic', '/rc_car/web/map')

        # map_server 지도 요청 서비스
        self.declare_parameter('map_service', '/robot3/map_server/map')

        # 재발행 주기
        self.declare_parameter('publish_rate', 1.0)

        map_topic = self.get_parameter('map_topic').value
        web_map_topic = self.get_parameter('web_map_topic').value
        self._map_service = self.get_parameter('map_service').value
        publish_rate = float(self.get_parameter('publish_rate').value)

        # 마지막으로 받은 지도 저장 공간
        self._latest_map = None

        # 지도 서비스 요청 진행 상태
        self._map_request_future = None

        # /map 수신용 QoS
        # 지도는 늦게 켜진 노드도 받을 수 있게 TRANSIENT_LOCAL로 구독
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # 웹 발행용 QoS
        # rosbridge가 쉽게 받을 수 있게 일반 VOLATILE 토픽으로 반복 발행
        web_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        # 원본 지도 구독자
        self.create_subscription(OccupancyGrid, map_topic, self._map_callback, map_qos)

        # 웹용 지도 발행자
        self._publisher = self.create_publisher(OccupancyGrid, web_map_topic, web_qos)

        # map_server 지도 요청 클라이언트
        self._map_client = self.create_client(GetMap, self._map_service)

        # 지도 반복 발행 타이머
        self.create_timer(1.0 / max(publish_rate, 0.1), self._publish_latest_map)

        # 지도 서비스 요청 타이머
        self.create_timer(2.0, self._request_map_from_service)

        self.get_logger().info(f'원본 지도 토픽: {map_topic}')
        self.get_logger().info(f'웹 지도 토픽: {web_map_topic}')
        self.get_logger().info(f'지도 요청 서비스: {self._map_service}')

    def _map_callback(self, map_msg):
        """원본 지도 저장."""
        if not self._is_valid_map(map_msg):
            self.get_logger().warn(
                f'빈 지도 무시: {map_msg.info.width} x {map_msg.info.height}'
            )
            return

        self._latest_map = map_msg
        self.get_logger().info(
            f'지도 수신 완료: {map_msg.info.width} x {map_msg.info.height}'
        )

    def _publish_latest_map(self):
        """저장된 지도 반복 발행."""
        if self._latest_map is None:
            return

        self._publisher.publish(self._latest_map)

    def _request_map_from_service(self):
        """map_server 서비스에서 지도 직접 요청."""
        if self._latest_map is not None:
            return

        if self._map_request_future is not None and not self._map_request_future.done():
            return

        if not self._map_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().warn(f'지도 서비스 대기 중: {self._map_service}')
            return

        self.get_logger().info('지도 서비스 요청')
        self._map_request_future = self._map_client.call_async(GetMap.Request())
        self._map_request_future.add_done_callback(self._handle_map_service_response)

    def _handle_map_service_response(self, future):
        """지도 서비스 응답 처리."""
        try:
            response = future.result()
        except Exception as error:
            self.get_logger().error(f'지도 서비스 응답 실패: {error}')
            return

        self._latest_map = response.map
        if not self._is_valid_map(self._latest_map):
            self.get_logger().warn(
                f'빈 지도 서비스 응답 무시: '
                f'{self._latest_map.info.width} x {self._latest_map.info.height}'
            )
            self._latest_map = None
            self._map_request_future = None
            return

        self.get_logger().info(
            f'지도 서비스 수신 완료: '
            f'{self._latest_map.info.width} x {self._latest_map.info.height}'
        )

    def _is_valid_map(self, map_msg):
        """사용 가능한 지도 여부 확인."""
        if map_msg.info.width == 0:
            return False

        if map_msg.info.height == 0:
            return False

        if len(map_msg.data) == 0:
            return False

        return True


def main(args=None):
    """노드 실행 시작점."""
    rclpy.init(args=args)
    node = WebMapNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
