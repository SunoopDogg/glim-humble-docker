"""Republish the TF map->base_link transform as a PoseStamped in the map frame.

A 10 Hz timer looks up the latest map->base_link transform and emits it on
/go2/map_pose. The PoseStamped is stamped with the TRANSFORM's own time (not
wall-clock now()) so RGB<->pose alignment in the recorded bag stays honest.
TF lookup failures (stack still converging, no map yet) are throttled-WARN'd and
skipped -- the node never crashes.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener, LookupException, \
    ConnectivityException, ExtrapolationException

from go2_rgb_odom_recorder.pose_math import quat_to_yaw


class PoseFromTf(Node):
    def __init__(self):
        super().__init__('pose_from_tf')
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.pose_topic = self.declare_parameter('pose_topic', '/go2/map_pose').value
        rate = float(self.declare_parameter('rate', 10.0).value)

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.pub = self.create_publisher(PoseStamped, self.pose_topic, 10)
        self.timer = self.create_timer(1.0 / rate, self._on_timer)
        self.get_logger().info(
            f'pose_from_tf: {self.map_frame}->{self.base_frame} @ {rate} Hz '
            f'-> {self.pose_topic}')

    def _on_timer(self):
        try:
            tf = self.buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            self.get_logger().warn(
                f'no {self.map_frame}->{self.base_frame} TF yet: {exc}',
                throttle_duration_sec=2.0)
            return

        t = tf.transform.translation
        q = tf.transform.rotation
        msg = PoseStamped()
        msg.header.stamp = tf.header.stamp          # transform's own time, not now()
        msg.header.frame_id = self.map_frame
        msg.pose.position.x = t.x
        msg.pose.position.y = t.y
        msg.pose.position.z = t.z
        msg.pose.orientation = q
        self.pub.publish(msg)
        self.get_logger().debug(
            f'x={t.x:.3f} y={t.y:.3f} theta={quat_to_yaw(q.x, q.y, q.z, q.w):.3f}')


def main(args=None):
    rclpy.init(args=args)
    node = PoseFromTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
