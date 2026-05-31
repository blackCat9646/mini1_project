"""Simple state manager for the mini project.

Service:
  /rc_car/set_state        SetFollowState

Output:
  /rc_car/system_state     SystemState

The first version keeps state management small and visible. Later, this node
can enable/disable other nodes, cancel Nav2 goals, or switch between search,
approach, and follow modes.
"""

import rclpy
from rclpy.node import Node

from rc_car_interfaces.msg import SystemState
from rc_car_interfaces.srv import SetFollowState


class SupervisorNode(Node):
    """Publish the current high-level project state."""

    VALID_STATES = {
        'idle',
        'search',
        'approach',
        'follow',
        'stop',
    }

    def __init__(self):
        super().__init__('supervisor_node')

        self.declare_parameter('state_topic', '/rc_car/system_state')
        self.declare_parameter('set_state_service', '/rc_car/set_state')

        state_topic = self.get_parameter('state_topic').value
        set_state_service = self.get_parameter('set_state_service').value

        self._state = 'idle'
        self._message = 'System is waiting.'
        self._publisher = self.create_publisher(SystemState, state_topic, 10)
        self.create_service(SetFollowState, set_state_service, self._on_set_state)
        self.create_timer(0.5, self._publish_state)

        self.get_logger().info(f'Publishing system state: {state_topic}')
        self.get_logger().info(f'State service ready: {set_state_service}')

    def _on_set_state(self, request, response):
        command = request.command.strip().lower()
        if command not in self.VALID_STATES:
            response.accepted = False
            response.message = f'Unknown command: {request.command}'
            return response

        self._state = command
        self._message = f'State changed to {command}.'
        response.accepted = True
        response.message = self._message
        return response

    def _publish_state(self):
        message = SystemState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.state = self._state
        message.message = self._message
        self._publisher.publish(message)


def main():
    rclpy.init()
    node = SupervisorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
