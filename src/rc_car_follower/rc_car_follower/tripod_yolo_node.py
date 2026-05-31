"""Detect the RC car from the fixed tripod camera.

Input:
  /tripod/image_raw      sensor_msgs/Image

Output:
  /rc_car/target/tripod_2d   rc_car_interfaces/Target2D

This node only answers one question:
  "Where is the RC car in the tripod camera image?"
"""

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

from rc_car_interfaces.msg import Target2D
from rc_car_follower.yolo_helper import biggest_box, load_yolo_model


class TripodYoloNode(Node):
    """YOLO detector for the camera mounted on a tripod."""

    def __init__(self):
        super().__init__('tripod_yolo_node')

        self.declare_parameter('image_topic', '/tripod/image_raw')
        self.declare_parameter('target_topic', '/rc_car/target/tripod_2d')
        self.declare_parameter('model_path', '')
        self.declare_parameter('target_class_name', 'rc_car')
        self.declare_parameter('confidence_threshold', 0.45)

        image_topic = self.get_parameter('image_topic').value
        target_topic = self.get_parameter('target_topic').value
        model_path = self.get_parameter('model_path').value

        self._target_class_name = self.get_parameter('target_class_name').value
        self._confidence_threshold = self.get_parameter('confidence_threshold').value
        self._bridge = CvBridge()
        self._model = load_yolo_model(self, model_path)

        self._publisher = self.create_publisher(Target2D, target_topic, 10)
        self.create_subscription(Image, image_topic, self._on_image, 10)

        self.get_logger().info(f'Subscribed tripod image: {image_topic}')
        self.get_logger().info(f'Publishing tripod target: {target_topic}')

    def _empty_detection(self, image_msg):
        """Create a message that clearly says: no RC car was found."""
        target = Target2D()
        target.header = image_msg.header
        target.detected = False
        target.source = 'tripod_camera'
        target.class_name = self._target_class_name
        target.confidence = 0.0
        target.track_id = -1
        return target

    def _on_image(self, image_msg):
        target = self._empty_detection(image_msg)

        if self._model is None:
            self._publisher.publish(target)
            return

        frame = self._bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        results = self._model.predict(frame, verbose=False)
        box = biggest_box(results[0], self._target_class_name)

        if box and box['confidence'] >= self._confidence_threshold:
            target.detected = True
            target.class_name = box['class_name']
            target.confidence = box['confidence']
            target.track_id = box['track_id']
            target.center_x = box['center_x']
            target.center_y = box['center_y']
            target.width = box['width']
            target.height = box['height']

        self._publisher.publish(target)


def main():
    rclpy.init()
    node = TripodYoloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
