#!/usr/bin/env python3
"""Publish an ascii .pcd as a latched PointCloud2 in the 'map' frame (RViz prior-map overlay)."""
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def read_ascii_pcd_xyz(path):
    with open(path) as f:
        lines = f.readlines()
    start = next(i for i, l in enumerate(lines) if l.startswith('DATA')) + 1
    pts = np.array([[float(v) for v in l.split()[:3]] for l in lines[start:] if l.strip()],
                   dtype=np.float32)
    return pts


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else 'maps/glim_map.pcd'
    pts = read_ascii_pcd_xyz(path)
    rclpy.init()
    node = Node('prior_map_pub')
    qos = QoSProfile(depth=1)
    qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
    qos.reliability = ReliabilityPolicy.RELIABLE
    pub = node.create_publisher(PointCloud2, '/prior_map', qos)

    msg = PointCloud2()
    msg.header = Header(frame_id='map')
    msg.height = 1
    msg.width = pts.shape[0]
    msg.fields = [PointField(name=n, offset=i * 4, datatype=PointField.FLOAT32, count=1)
                  for i, n in enumerate(('x', 'y', 'z'))]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * pts.shape[0]
    msg.is_dense = True
    msg.data = pts.tobytes()
    node.get_logger().info(f'publishing {pts.shape[0]} points from {path} on /prior_map (latched)')

    def tick():
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)
    node.create_timer(1.0, tick)
    tick()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
