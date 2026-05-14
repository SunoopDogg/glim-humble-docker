#!/usr/bin/env python3
"""ply_to_pcd — convert a PLY point cloud to PCD.

For the offline export route: GLIM saves a factor-graph dump to /tmp/dump on
shutdown; you open it in ``ros2 run glim_ros offline_viewer`` (File > Save >
Export Points) to get a PLY, then run this to get a PCD for downstream tools.

Uses Open3D if installed (preserves more fields); otherwise falls back to a
dependency-free ascii PLY -> ascii PCD converter (x/y/z only, via pointcloud_io).

Usage: ros2 run go2_glim_mapping ply_to_pcd <in.ply> [out.pcd]
"""
import sys

from go2_glim_mapping.pointcloud_io import read_ply_xyz, write_pcd


def main():
    if len(sys.argv) < 2:
        print('usage: ply_to_pcd <in.ply> [out.pcd]')
        sys.exit(1)
    src = sys.argv[1]
    dst = sys.argv[2] if len(sys.argv) > 2 else src.rsplit('.', 1)[0] + '.pcd'

    try:
        import open3d as o3d
        pc = o3d.io.read_point_cloud(src)
        if pc.has_points():
            o3d.io.write_point_cloud(dst, pc)
            print(f'[open3d] {src} -> {dst} ({len(pc.points)} points)')
            return
        print(f'[open3d] {src} has no points')
        return
    except ImportError:
        pts = read_ply_xyz(src)
        write_pcd(dst, pts)
        print(f'[fallback xyz] {src} -> {dst} ({len(pts)} points; install open3d to keep intensity/color)')


if __name__ == '__main__':
    main()
