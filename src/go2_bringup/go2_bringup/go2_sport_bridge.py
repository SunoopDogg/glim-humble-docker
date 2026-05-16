#!/usr/bin/env python3
"""go2_sport_bridge — /cmd_vel (Twist) → unitree Go2 sport Move command (DDS).

Converts ROS2 geometry_msgs/Twist to a unitree_api/Request with JSON parameter
and publishes on /api/sport/request (Go2 sport mode API). Only linear.x (forward)
and angular.z (yaw) are mapped; lateral velocity (y) is always 0.

API reference: unitree_ros2/example/src/include/common/ros2_sport_client.h
  ROBOT_SPORT_API_ID_MOVE = 1008
  parameter = JSON {"x": vx, "y": vy, "z": vyaw}
"""
import json

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import Twist

try:
    from unitree_api.msg import Request
    _HAS_UNITREE = True
except ImportError:  # pragma: no cover — only missing before colcon build
    _HAS_UNITREE = False

SPORT_CMD_TOPIC = '/api/sport/request'
ROBOT_SPORT_API_ID_MOVE = 1008


def twist_to_sport_params(linear_x: float, angular_z: float) -> dict:
    """Convert Twist velocity scalars to unitree_api/Request field values.

    Pure function — no ROS dependency, fully unit-testable.
    Returns dict with 'api_id' (int) and 'parameter' (JSON string).
    """
    return {
        'api_id': ROBOT_SPORT_API_ID_MOVE,
        'parameter': json.dumps({'x': float(linear_x), 'y': 0.0, 'z': float(angular_z)}),
    }


def clamp_twist(linear_x: float, angular_z: float,
                max_vx: float, max_vyaw: float) -> tuple:
    """Clamp forward/yaw velocity to +/- caps. Pure — no ROS dependency."""
    vx = max(-max_vx, min(max_vx, float(linear_x)))
    vyaw = max(-max_vyaw, min(max_vyaw, float(angular_z)))
    return vx, vyaw


class Go2SportBridge(Node):
    def __init__(self):
        super().__init__('go2_sport_bridge')
        if not _HAS_UNITREE:
            self.get_logger().error(
                'unitree_api not found — build with colcon first. '
                'Node running but will not publish.'
            )
        self._sub = self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)
        self._pub = (
            self.create_publisher(Request, SPORT_CMD_TOPIC, 10)
            if _HAS_UNITREE else None
        )
        self.get_logger().info(f'go2_sport_bridge: /cmd_vel → {SPORT_CMD_TOPIC}')

    def _on_cmd_vel(self, msg: Twist):
        if self._pub is None:
            return
        params = twist_to_sport_params(msg.linear.x, msg.angular.z)
        req = Request()
        req.header.identity.api_id = params['api_id']
        req.parameter = params['parameter']
        self._pub.publish(req)


def main():
    rclpy.init()
    node = Go2SportBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
