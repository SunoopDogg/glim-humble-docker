"""Offline pcd -> Nav2 OccupancyGrid (pgm + yaml) for the static global costmap.

The prior map is static, so the global costmap is generated once from the GLIM
.pcd (no live converter). Points are filtered to a height band (drop floor/ceiling)
and rasterized to a 2D occupancy grid, written in nav2_map_server's pgm+yaml format.
Live dynamic obstacles are handled separately by STVL on the local costmap.

  ros2 run go2_glim_navigation pcd_to_costmap --pcd maps/glim_map.pcd \
      --out maps/glim_costmap --resolution 0.05 --z-min 0.2 --z-max 2.0
"""
import argparse

OCCUPIED = 100
FREE = 0


def rasterize_occupancy(points, resolution, z_min, z_max):
    """Rasterize (x,y,z) points to a 2D occupancy grid over the height band [z_min,z_max].

    Returns (grid, origin_x, origin_y, width, height) where grid[row][col] is OCCUPIED
    (100) if any in-band point falls in that cell else FREE (0); row indexes +y, col +x;
    origin is the obstacle min corner (map frame). Raises ValueError if no point is in band.
    """
    obstacles = [(x, y) for (x, y, z) in points if z_min <= z <= z_max]
    if not obstacles:
        raise ValueError(f"no points in height band [{z_min}, {z_max}]")
    xs = [x for x, _ in obstacles]
    ys = [y for _, y in obstacles]
    min_x, min_y = min(xs), min(ys)
    width = int((max(xs) - min_x) / resolution) + 1
    height = int((max(ys) - min_y) / resolution) + 1
    grid = [[FREE] * width for _ in range(height)]
    for x, y in obstacles:
        col = int((x - min_x) / resolution)
        row = int((y - min_y) / resolution)
        grid[row][col] = OCCUPIED
    return grid, min_x, min_y, width, height


def read_pcd_xyz(path):
    """Read (x,y,z) tuples from an ASCII PCD with leading x y z fields."""
    points = []
    with open(path) as f:
        in_data = False
        for line in f:
            if not in_data:
                if line.startswith('DATA'):
                    if 'ascii' not in line:
                        raise ValueError("only ASCII PCD is supported")
                    in_data = True
                continue
            parts = line.split()
            if len(parts) >= 3:
                points.append((float(parts[0]), float(parts[1]), float(parts[2])))
    return points


def write_costmap(grid, origin_x, origin_y, resolution, out_basename):
    """Write grid as nav2_map_server pgm (P5) + yaml. occupied->0(black), free->254."""
    height = len(grid)
    width = len(grid[0])
    pgm_path = out_basename + '.pgm'
    yaml_path = out_basename + '.yaml'
    with open(pgm_path, 'wb') as f:
        f.write(f"P5\n{width} {height}\n255\n".encode())
        # PGM row 0 is top; map_server flips so image bottom = grid row 0 (min y).
        for row in reversed(grid):
            f.write(bytes(0 if c == OCCUPIED else 254 for c in row))
    import os
    with open(yaml_path, 'w') as f:
        f.write(
            f"image: {os.path.basename(pgm_path)}\n"
            f"resolution: {resolution}\n"
            f"origin: [{origin_x}, {origin_y}, 0.0]\n"
            "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
    return pgm_path, yaml_path


def main(argv=None):
    ap = argparse.ArgumentParser(description="pcd -> Nav2 occupancy grid (pgm+yaml)")
    ap.add_argument('--pcd', required=True)
    ap.add_argument('--out', required=True, help='output basename (writes .pgm + .yaml)')
    ap.add_argument('--resolution', type=float, default=0.05)
    ap.add_argument('--z-min', type=float, default=0.2)
    ap.add_argument('--z-max', type=float, default=2.0)
    a = ap.parse_args(argv)
    pts = read_pcd_xyz(a.pcd)
    grid, ox, oy, w, h = rasterize_occupancy(pts, a.resolution, a.z_min, a.z_max)
    pgm, yaml_path = write_costmap(grid, ox, oy, a.resolution, a.out)
    print(f"wrote {pgm} ({w}x{h}) and {yaml_path}")


if __name__ == '__main__':
    main()
