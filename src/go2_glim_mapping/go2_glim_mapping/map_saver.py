#!/usr/bin/env python3
"""map_saver — subscribe GLIM's latched global map and save it to PLY + PCD.

GLIM (node name "glim_ros") publishes its global point-cloud map on
``/glim_ros/map`` with RELIABLE + TRANSIENT_LOCAL (latched) QoS. This node holds
the most recent map and writes it to disk:
  * on a ``~/save_map`` (std_srvs/Trigger) service call, and
  * automatically on shutdown.
PLY + PCD writing lives in :mod:`go2_glim_mapping.pointcloud_io` (no Open3D dep).

Note: ``/glim_ros/map`` only populates after a submap finalizes (see
config_sub_mapping_*.json: max_num_keyframes / max_keyframe_overlap). The
``/tmp/dump`` factor-graph dump that glim_rosnode writes on shutdown is always
produced regardless.

Params:
  map_topic   (str)  default /glim_ros/map
  output_dir  (str)  default /tmp
  basename    (str)  default glim_map
"""
import os

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger

from go2_glim_mapping.pointcloud_io import read_xyz, write_ply, write_pcd

try:
    from rclpy._rclpy_pybind11 import RCLError
except ImportError:  # pragma: no cover
    RCLError = ()


class MapSaver(Node):
    def __init__(self):
        super().__init__('map_saver')
        self.map_topic = self.declare_parameter('map_topic', '/glim_ros/map').value
        self.output_dir = self.declare_parameter('output_dir', '/tmp').value
        self.basename = self.declare_parameter('basename', 'glim_map').value
        self._latest = None

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.sub = self.create_subscription(PointCloud2, self.map_topic, self._on_map, qos)
        self.srv = self.create_service(Trigger, '~/save_map', self._on_save)
        self.get_logger().info(
            f'map_saver: listening {self.map_topic}, output {self.output_dir}/{self.basename}.{{ply,pcd}}; '
            f'call ~/save_map or stop to write')

    def _on_map(self, msg):
        self._latest = msg

    def _save(self):
        if self._latest is None:
            return 0, 'no map received yet'
        pts = read_xyz(self._latest)
        if len(pts) == 0:
            return 0, 'map cloud has no x/y/z'
        os.makedirs(self.output_dir, exist_ok=True)
        ply = os.path.join(self.output_dir, self.basename + '.ply')
        pcd = os.path.join(self.output_dir, self.basename + '.pcd')
        write_ply(ply, pts)
        write_pcd(pcd, pts)
        return len(pts), f'{ply} + {pcd}'

    def _on_save(self, req, resp):
        n, where = self._save()
        resp.success = n > 0
        resp.message = f'saved {n} points -> {where}' if n else where
        self.get_logger().info(resp.message)
        return resp


def main():
    rclpy.init()
    node = MapSaver()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass  # SIGINT: rclpy may shut the context mid-spin; we still save below
    finally:
        n, where = node._save()
        node.get_logger().info(f'shutdown save: {n} points -> {where}' if n else f'shutdown: {where}')
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
