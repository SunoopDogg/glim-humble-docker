"""Pure-logic helpers for the navigation launch (no ROS imports, unit-testable).

GLIM reads config from a directory and loads its prior map from a "dump" directory
(factor graph + per-submap data) produced by mapping. Nav2 reads a params YAML.
Both are prepared/validated here with plain file/dict ops so the launch logic is
testable without spinning up ROS.
"""
import os


def validate_pcd_map(pcd_path):
    """Return pcd_path if it is a non-empty .pcd file, else raise a precise error.

    icp_localization localizes against a .pcd reference map (pcd_filepath), and the
    same file feeds the offline costmap export. GLIM already writes maps/glim_map.pcd,
    so failing fast here surfaces a missing/empty map at launch instead of as an
    opaque ICP crash.
    """
    if not os.path.exists(pcd_path):
        raise FileNotFoundError(f"pcd map does not exist: {pcd_path}")
    if os.path.isdir(pcd_path):
        raise IsADirectoryError(f"pcd map must be a .pcd file, not a dir: {pcd_path}")
    if not pcd_path.endswith('.pcd'):
        raise ValueError(f"pcd map must be a .pcd file: {pcd_path}")
    if os.path.getsize(pcd_path) == 0:
        raise ValueError(f"pcd map is empty: {pcd_path}")
    return pcd_path


def prepare_nav2_params(src_yaml, dst_yaml, use_sim_time, robot_radius=None):
    """Copy src_yaml->dst_yaml, set use_sim_time on every node, optionally override
    robot_radius on both costmaps; return dst_yaml. Never mutates src_yaml.

    Recurses the nested Nav2 param structure so the single committed params file
    serves both sim (use_sim_time=true) and real (false) without a duplicate file.
    """
    import yaml

    with open(src_yaml) as f:
        data = yaml.safe_load(f)

    def _patch(node):
        if isinstance(node, dict):
            params = node.get('ros__parameters')
            if isinstance(params, dict):
                if 'use_sim_time' in params:
                    params['use_sim_time'] = use_sim_time
                if robot_radius is not None and 'robot_radius' in params:
                    params['robot_radius'] = robot_radius
            for v in node.values():
                _patch(v)

    _patch(data)
    os.makedirs(os.path.dirname(dst_yaml), exist_ok=True)
    with open(dst_yaml, 'w') as f:
        yaml.safe_dump(data, f)
    return dst_yaml
