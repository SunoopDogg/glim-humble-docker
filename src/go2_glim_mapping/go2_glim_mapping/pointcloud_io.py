"""Shared point-cloud IO: read PointCloud2 -> (N,3) array, write PLY / PCD.

Used by both map_saver (live map) and ply_to_pcd (offline conversion) so the
PLY/PCD formats live in exactly one place. Bulk numpy I/O, no Open3D dependency.
"""
import numpy as np
from sensor_msgs_py.point_cloud2 import read_points


def read_xyz(msg):
    """PointCloud2 -> (N, 3) float64 array of x,y,z (NaNs dropped)."""
    arr = read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
    return np.stack([arr['x'], arr['y'], arr['z']], axis=-1).astype(np.float64)


def write_ply(path, pts):
    """Write an (N,3) array to an ascii PLY."""
    with open(path, 'w') as f:
        f.write('ply\nformat ascii 1.0\n')
        f.write(f'element vertex {len(pts)}\n')
        f.write('property float x\nproperty float y\nproperty float z\nend_header\n')
        np.savetxt(f, pts, fmt='%g')


def write_pcd(path, pts):
    """Write an (N,3) array to an ascii PCD (v0.7)."""
    with open(path, 'w') as f:
        f.write('# .PCD v0.7 - Point Cloud Data\nVERSION 0.7\n')
        f.write('FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n')
        f.write(f'WIDTH {len(pts)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n')
        f.write(f'POINTS {len(pts)}\nDATA ascii\n')
        np.savetxt(f, pts, fmt='%g')


def read_ply_xyz(path):
    """Read x,y,z from an ascii PLY into an (N,3) array (fallback PLY->PCD path)."""
    with open(path) as f:
        if not f.readline().startswith('ply'):
            raise ValueError('not a PLY file')
        fmt, n, order = '', 0, []
        in_vertex = False
        for line in f:
            t = line.split()
            if t[0] == 'format':
                fmt = t[1]
            elif t[0] == 'element':
                in_vertex = (t[1] == 'vertex')
                if in_vertex:
                    n = int(t[2])
            elif t[0] == 'property' and in_vertex:
                order.append(t[-1])
            elif t[0] == 'end_header':
                break
        if fmt != 'ascii':
            raise ValueError(f'only ascii PLY supported in fallback (got {fmt}); install open3d')
        cols = (order.index('x'), order.index('y'), order.index('z'))
        body = np.loadtxt(f, max_rows=n)
    return body[:, cols] if body.ndim == 2 else body[list(cols)].reshape(1, 3)
