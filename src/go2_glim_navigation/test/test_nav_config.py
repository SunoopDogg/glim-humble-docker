import pytest
import yaml

from go2_glim_navigation.nav_config import (
    prepare_icp_params,
    prepare_nav2_params,
    validate_pcd_map,
)


def test_valid_pcd_returns_path(tmp_path):
    f = tmp_path / 'glim_map.pcd'
    f.write_text('# .PCD\n1 2 3\n')
    assert validate_pcd_map(str(f)) == str(f)


def test_missing_pcd_raises(tmp_path):
    with pytest.raises(FileNotFoundError):
        validate_pcd_map(str(tmp_path / 'nope.pcd'))


def test_dir_raises(tmp_path):
    with pytest.raises(IsADirectoryError):
        validate_pcd_map(str(tmp_path))


def test_non_pcd_extension_raises(tmp_path):
    f = tmp_path / 'map.ply'
    f.write_text('ply')
    with pytest.raises(ValueError):
        validate_pcd_map(str(f))


def test_empty_pcd_raises(tmp_path):
    f = tmp_path / 'empty.pcd'
    f.write_bytes(b'')
    with pytest.raises(ValueError):
        validate_pcd_map(str(f))


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


def _write_icp_params(tmp_path):
    src = tmp_path / 'node_params.yaml'
    src.write_text(yaml.safe_dump({
        '/icp_localization': {'ros__parameters': {
            'pcd_file_path': '/old/test.pcd',
            'input_filters_config_path': 'config/input_filteres_mid360.yaml',
            'icp_localization_ros2': {
                'range_data_topic': '/cloud_registered_body',
                'imu_data_topic': '/imu/data',
                'odometry_data_topic': '/Odometry',
                'is_use_odometry': True,
            },
        }},
    }))
    return src


def test_prepare_icp_sets_map_topics_and_filters(tmp_path):
    src = _write_icp_params(tmp_path)
    out = prepare_icp_params(
        str(src), str(tmp_path / 'eff' / 'icp.yaml'),
        pcd_path='/maps/glim_map.pcd', points_topic='/ouster/points',
        imu_topic='/ouster/imu', odom_topic='/rko_lio/odometry',
        input_filters_path='/share/input_filters_ouster_os1.yaml')
    p = yaml.safe_load(open(out))['/icp_localization']['ros__parameters']
    assert p['pcd_file_path'] == '/maps/glim_map.pcd'
    assert p['input_filters_config_path'] == '/share/input_filters_ouster_os1.yaml'
    inner = p['icp_localization_ros2']
    assert inner['range_data_topic'] == '/ouster/points'
    assert inner['imu_data_topic'] == '/ouster/imu'
    assert inner['odometry_data_topic'] == '/rko_lio/odometry'
    assert inner['is_use_odometry'] is True   # preserved


def test_prepare_icp_leaves_committed_file_untouched(tmp_path):
    src = _write_icp_params(tmp_path)
    before = src.read_text()
    prepare_icp_params(
        str(src), str(tmp_path / 'eff' / 'icp.yaml'),
        pcd_path='/m.pcd', points_topic='/p', imu_topic='/i',
        odom_topic='/o', input_filters_path='/f.yaml')
    assert src.read_text() == before
