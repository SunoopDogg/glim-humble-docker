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
from std_srvs.srv import SetBool

try:
    from unitree_api.msg import Request
    _HAS_UNITREE = True
except ImportError:  # pragma: no cover — only missing before colcon build
    _HAS_UNITREE = False

SPORT_CMD_TOPIC = '/api/sport/request'
ROBOT_SPORT_API_ID_MOVE = 1008

# Sport mode-string → api_id. Mirrors ~/ros2_ws go2_driver::handleMode + go2_api_id.hpp.
# Mode requests carry NO JSON parameter (unlike MOVE) — parameter is always ''.
ROBOT_SPORT_MODE_API_ID = {
    'damp': 1001,
    'balance_stand': 1002,
    'stop_move': 1003,
    'stand_up': 1004,
    'stand_down': 1005,
    'recovery_stand': 1006,
}


def mode_to_sport_request(mode: str):
    """Map a sport mode-string to a unitree_api/Request payload.

    Pure — no ROS dependency. Returns {'api_id': int, 'parameter': ''} for a known
    mode, or None for an unknown one (caller logs + ignores). Mode requests have an
    empty parameter; only MOVE carries the {x,y,z} JSON.
    """
    api_id = ROBOT_SPORT_MODE_API_ID.get(mode)
    if api_id is None:
        return None
    return {'api_id': api_id, 'parameter': ''}


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


def gated_velocity(enabled: bool, linear_x: float, angular_z: float,
                   max_vx: float, max_vyaw: float) -> tuple:
    """Resolve the (vx, vyaw) to command: (0, 0) when disabled, else clamped.

    Pure — encapsulates the full motion decision so the node stays thin and the
    enable-gate is unit-testable without ROS.
    """
    if not enabled:
        return 0.0, 0.0
    return clamp_twist(linear_x, angular_z, max_vx, max_vyaw)


class Go2SportBridge(Node):
    def __init__(self):
        super().__init__('go2_sport_bridge')
        if not _HAS_UNITREE:
            self.get_logger().error(
                'unitree_api not found — build with colcon first. '
                'Node running but will not publish.'
            )
        self.declare_parameter('enabled', False)
        self.declare_parameter('max_vx', 0.3)
        self.declare_parameter('max_vyaw', 0.5)
        self.declare_parameter('cmd_timeout', 0.5)
        self.enabled = self.get_parameter('enabled').value
        self.max_vx = self.get_parameter('max_vx').value
        self.max_vyaw = self.get_parameter('max_vyaw').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value
        self._last_cmd = self.get_clock().now()

        self._sub = self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)
        self._pub = (
            self.create_publisher(Request, SPORT_CMD_TOPIC, 10)
            if _HAS_UNITREE else None
        )
        self._enable_srv = self.create_service(SetBool, '~/enable', self._on_enable)
        self._watchdog = self.create_timer(0.1, self._on_watchdog)
        self.get_logger().info(
            f'go2_sport_bridge: /cmd_vel → {SPORT_CMD_TOPIC} '
            f'(enabled={self.enabled}, max_vx={self.max_vx}, max_vyaw={self.max_vyaw})'
        )

    def _publish_move(self, vx: float, vyaw: float):
        if self._pub is None:
            return
        params = twist_to_sport_params(vx, vyaw)
        req = Request()
        req.header.identity.api_id = params['api_id']
        req.parameter = params['parameter']
        self._pub.publish(req)

    def _on_cmd_vel(self, msg: Twist):
        self._last_cmd = self.get_clock().now()
        vx, vyaw = gated_velocity(
            self.enabled, msg.linear.x, msg.angular.z, self.max_vx, self.max_vyaw
        )
        self._publish_move(vx, vyaw)

    def _on_enable(self, request, response):
        self.enabled = request.data
        if not self.enabled:
            self._publish_move(0.0, 0.0)  # active stop on disable
        response.success = True
        response.message = f'drive enabled={self.enabled}'
        self.get_logger().info(response.message)
        return response

    def _on_watchdog(self):
        if not self.enabled:
            return
        age = (self.get_clock().now() - self._last_cmd).nanoseconds * 1e-9
        if age > self.cmd_timeout:
            self._publish_move(0.0, 0.0)  # stop if cmd_vel stale (Nav2 silent/crashed)


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
