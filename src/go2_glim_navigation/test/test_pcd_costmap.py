import pytest

from go2_glim_navigation.pcd_costmap import rasterize_occupancy


def test_marks_obstacle_cells_occupied_rest_free():
    # two obstacle points 1.0 m apart in x at z in band; resolution 0.5 -> 3 cols
    pts = [(0.0, 0.0, 1.0), (1.0, 0.0, 1.0)]
    grid, ox, oy, w, h = rasterize_occupancy(pts, resolution=0.5, z_min=0.2, z_max=2.0)
    assert (w, h) == (3, 1)
    assert (ox, oy) == (0.0, 0.0)
    assert grid[0][0] == 100   # obstacle at min corner
    assert grid[0][2] == 100   # obstacle at +1.0 m -> col 2
    assert grid[0][1] == 0     # gap is free


def test_z_band_filters_floor_and_ceiling():
    pts = [(0.0, 0.0, -0.1),   # floor, below band -> dropped
           (0.0, 0.0, 1.0),    # wall, in band
           (0.0, 0.0, 3.0)]    # ceiling, above band -> dropped
    grid, ox, oy, w, h = rasterize_occupancy(pts, resolution=0.5, z_min=0.2, z_max=2.0)
    assert (w, h) == (1, 1)
    assert grid[0][0] == 100


def test_origin_is_obstacle_min_corner():
    pts = [(-2.0, -3.0, 1.0), (-1.0, -3.0, 1.0)]
    grid, ox, oy, w, h = rasterize_occupancy(pts, resolution=1.0, z_min=0.2, z_max=2.0)
    assert (ox, oy) == (-2.0, -3.0)


def test_no_obstacles_in_band_raises():
    pts = [(0.0, 0.0, -0.5), (1.0, 1.0, 5.0)]
    with pytest.raises(ValueError):
        rasterize_occupancy(pts, resolution=0.5, z_min=0.2, z_max=2.0)
