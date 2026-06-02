"""로봇의 현재 위치를 웹 표시용 토픽으로 바꾸는 노드."""

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


class WebPoseNode(Node):
    """TF 기반 로봇 위치 발행 노드."""

    def __init__(self):
        super().__init__('web_pose_node')

        # 파라미터 선언
        self.declare_parameter('pose_topic', '/rc_car/web/robot_pose')
        self.declare_parameter('amcl_pose_topic', '/robot3/amcl_pose')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('publish_rate', 5.0)

        # 파라미터 값 읽기
        pose_topic = self.get_parameter('pose_topic').value
        amcl_pose_topic = self.get_parameter('amcl_pose_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        publish_rate = float(self.get_parameter('publish_rate').value)

        # AMCL 기반 마지막 위치
        self._last_amcl_pose = None

        # TF 저장소 및 수신기
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 웹 표시용 위치 발행자
        self.pose_pub = self.create_publisher(PoseStamped, pose_topic, 10)

        # AMCL 위치 구독자
        self.create_subscription(
            PoseWithCovarianceStamped,
            amcl_pose_topic,
            self._amcl_pose_callback,
            10,
        )

        # 주기 실행 타이머
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_robot_pose)

        self.get_logger().info(f'웹 위치 토픽: {pose_topic}')
        self.get_logger().info(f'AMCL 위치 토픽: {amcl_pose_topic}')
        self.get_logger().info(f'위치 기준 프레임: {self.map_frame}')
        self.get_logger().info(f'로봇 프레임: {self.robot_frame}')

    def _amcl_pose_callback(self, pose_msg):
        """AMCL 위치 저장."""
        self._last_amcl_pose = pose_msg

    def publish_robot_pose(self):
        """현재 로봇 위치 발행."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.robot_frame,
                rclpy.time.Time(),
            )
        except TransformException as error:
            self.get_logger().debug(f'TF 대기 중: {error}')
            self._publish_amcl_pose()
            return

        # TF 값을 PoseStamped로 변환
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.map_frame
        pose_msg.pose.position.x = transform.transform.translation.x
        pose_msg.pose.position.y = transform.transform.translation.y
        pose_msg.pose.position.z = transform.transform.translation.z
        pose_msg.pose.orientation = transform.transform.rotation

        self.pose_pub.publish(pose_msg)

    def _publish_amcl_pose(self):
        """TF 실패 시 AMCL 위치 발행."""
        if self._last_amcl_pose is None:
            return

        pose_msg = PoseStamped()
        pose_msg.header = self._last_amcl_pose.header
        pose_msg.pose = self._last_amcl_pose.pose.pose
        self.pose_pub.publish(pose_msg)


def main(args=None):
    """노드 실행 시작점."""
    rclpy.init(args=args)
    node = WebPoseNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
