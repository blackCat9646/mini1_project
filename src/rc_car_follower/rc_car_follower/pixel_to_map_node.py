"""Convert a tripod-camera pixel into a map coordinate.

Input:
  /rc_car/target/tripod_2d   Target2D

Output:
  /rc_car/target/map         Target3D

The important idea:
  YOLO gives a pixel like (320, 240).
  Nav2 needs a map point like (1.2 m, -0.4 m).
  A homography matrix is the "translation table" between those two worlds.
"""

import yaml

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node

from rc_car_interfaces.msg import Target2D, Target3D


class PixelToMapNode(Node):
    """Turn tripod camera detections into map-frame target points."""

    def __init__(self):
        super().__init__('pixel_to_map_node')

        self.declare_parameter('input_topic', '/rc_car/target/tripod_2d')
        self.declare_parameter('output_topic', '/rc_car/target/map')
        self.declare_parameter('homography_file', '')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        homography_file = self.get_parameter('homography_file').value

        self._homography = self._load_homography(homography_file)
        self._publisher = self.create_publisher(Target3D, output_topic, 10)
        self.create_subscription(Target2D, input_topic, self._on_target_2d, 10)

        self.get_logger().info(f'Subscribed 2D target: {input_topic}')
        self.get_logger().info(f'Publishing map target: {output_topic}')

    def _load_homography(self, path):
        """Load a 3x3 homography matrix from YAML."""
        if not path:
            self.get_logger().warn('homography_file is empty. Map target will be invalid.')
            return None

        try:
            with open(path, 'r', encoding='utf-8') as yaml_file:
                data = yaml.safe_load(yaml_file)
        except OSError as error:
            self.get_logger().warn(f'Cannot read homography file: {error}')
            return None

        matrix = data.get('homography')
        if not matrix or len(matrix) != 3:
            self.get_logger().warn('homography must be a 3x3 matrix.')
            return None

        return matrix

    def _pixel_to_map(self, pixel_x, pixel_y):
        """Apply the homography formula by hand so it is easy to explain."""
        h = self._homography
        scale = h[2][0] * pixel_x + h[2][1] * pixel_y + h[2][2]
        if abs(scale) < 1e-6:
            return None

        map_x = (h[0][0] * pixel_x + h[0][1] * pixel_y + h[0][2]) / scale
        map_y = (h[1][0] * pixel_x + h[1][1] * pixel_y + h[1][2]) / scale
        return map_x, map_y

    def _on_target_2d(self, target_2d):
        target_3d = Target3D()
        target_3d.header.stamp = self.get_clock().now().to_msg()
        target_3d.header.frame_id = 'map'
        target_3d.source = 'tripod_mapper'
        target_3d.class_name = target_2d.class_name
        target_3d.confidence = target_2d.confidence
        target_3d.track_id = target_2d.track_id
        target_3d.distance = -1.0

        if not target_2d.detected or self._homography is None:
            target_3d.detected = False
            self._publisher.publish(target_3d)
            return

        map_point = self._pixel_to_map(target_2d.center_x, target_2d.center_y)
        if map_point is None:
            target_3d.detected = False
            self._publisher.publish(target_3d)
            return

        target_3d.detected = True
        target_3d.position = Point(x=map_point[0], y=map_point[1], z=0.0)
        self._publisher.publish(target_3d)


def main():
    rclpy.init()
    node = PixelToMapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
