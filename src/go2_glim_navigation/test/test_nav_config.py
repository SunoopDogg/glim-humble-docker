import pytest
import yaml

from go2_glim_navigation.nav_config import prepare_nav2_params, validate_map_path


def _make_dump(tmp_path):
    """Minimal GLIM dump signature: graph.bin/.txt + values.bin + one numbered submap."""
    (tmp_path / 'graph.bin').write_bytes(b'\x00')
    (tmp_path / 'graph.txt').write_text('graph')
    (tmp_path / 'values.bin').write_bytes(b'\x00')
    submap = tmp_path / '000000'
    submap.mkdir()
    (submap / 'data.txt').write_text('submap')
    return tmp_path


def test_valid_dump_returns_path(tmp_path):
    d = _make_dump(tmp_path)
    assert validate_map_path(str(d)) == str(d)


def test_missing_dir_raises(tmp_path):
    with pytest.raises(FileNotFoundError):
        validate_map_path(str(tmp_path / 'nope'))


def test_dir_without_graph_raises(tmp_path):
    (tmp_path / '000000').mkdir()
    with pytest.raises(ValueError):
        validate_map_path(str(tmp_path))


def test_file_instead_of_dir_raises(tmp_path):
    f = tmp_path / 'x.pcd'
    f.write_text('not a dir')
    with pytest.raises(NotADirectoryError):
        validate_map_path(str(f))


def _write_params(tmp_path):
    src = tmp_path / 'nav2_params.yaml'
    src.write_text(yaml.safe_dump({
        'planner_server': {'ros__parameters': {'use_sim_time': True}},
        'controller_server': {'ros__parameters': {'use_sim_time': True}},
        'local_costmap': {'local_costmap': {'ros__parameters': {
            'use_sim_time': True, 'robot_radius': 0.3}}},
    }))
    return src


def test_prepare_sets_use_sim_time_on_all_nodes(tmp_path):
    src = _write_params(tmp_path)
    dst = tmp_path / 'eff'
    out = prepare_nav2_params(str(src), str(dst / 'p.yaml'), use_sim_time=False)
    data = yaml.safe_load(open(out))
    assert data['planner_server']['ros__parameters']['use_sim_time'] is False
    assert data['controller_server']['ros__parameters']['use_sim_time'] is False
    assert (data['local_costmap']['local_costmap']['ros__parameters']
            ['use_sim_time'] is False)


def test_prepare_overrides_robot_radius_when_given(tmp_path):
    src = _write_params(tmp_path)
    dst = tmp_path / 'eff'
    out = prepare_nav2_params(str(src), str(dst / 'p.yaml'),
                              use_sim_time=False, robot_radius=0.42)
    data = yaml.safe_load(open(out))
    assert (data['local_costmap']['local_costmap']['ros__parameters']
            ['robot_radius'] == 0.42)


def test_prepare_leaves_committed_file_untouched(tmp_path):
    src = _write_params(tmp_path)
    before = src.read_text()
    prepare_nav2_params(str(src), str(tmp_path / 'eff' / 'p.yaml'), use_sim_time=False)
    assert src.read_text() == before
