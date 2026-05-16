"""Pure-logic helpers for the navigation launch (no ROS imports, unit-testable).

GLIM reads config from a directory and loads its prior map from a "dump" directory
(factor graph + per-submap data) produced by mapping. Nav2 reads a params YAML.
Both are prepared/validated here with plain file/dict ops so the launch logic is
testable without spinning up ROS.
"""
import os


def validate_map_path(map_path):
    """Return map_path if it is a GLIM dump dir, else raise a precise error.

    A GLIM dump has a graph file (graph.bin/graph.txt) at its root plus numbered
    submap subdirs. Failing fast here surfaces the "must build a map first" gotcha
    at launch instead of as an opaque GLIM crash.
    """
    if not os.path.exists(map_path):
        raise FileNotFoundError(f"map_path does not exist: {map_path}")
    if not os.path.isdir(map_path):
        raise NotADirectoryError(f"map_path must be a GLIM dump dir, not a file: {map_path}")
    has_graph = any(os.path.exists(os.path.join(map_path, f))
                    for f in ('graph.bin', 'graph.txt'))
    if not has_graph:
        raise ValueError(f"map_path is not a GLIM dump (no graph.bin/graph.txt): {map_path}")
    return map_path


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
