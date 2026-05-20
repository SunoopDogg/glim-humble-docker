import json
import os

import pytest

from go2_glim_mapping.launch_config import (
    load_extrinsic_yaml,
    prepare_config,
    resolve_map_paths,
    resolve_mode,
)


def _make_src(tmp_path):
    src = tmp_path / "glim"
    src.mkdir()
    (src / "config_sensors.json").write_text(json.dumps({
        "sensors": {"global_shutter_lidar": True, "T_lidar_imu": [-0.1, 0, -0.1, 0, 0, 0, 1]}
    }))
    (src / "config_ros.json").write_text(json.dumps({
        "glim_ros": {"extension_modules": ["libmemory_monitor.so", "librviz_viewer.so"]}
    }))
    return str(src)


def test_sim_no_viewer_returns_src_untouched(tmp_path):
    src = _make_src(tmp_path)
    dst = str(tmp_path / "out")
    assert prepare_config(src, dst, profile='sim', viewer=False) == src
    assert not os.path.exists(dst)


def test_real_profile_patches_sensors(tmp_path):
    src = _make_src(tmp_path)
    dst = str(tmp_path / "out")
    out = prepare_config(src, dst, profile='real',
                         t_lidar_imu=[0.0, 0.0, 0.03, 0.0, 0.0, 1.0, 0.0])
    assert out == dst
    sensors = json.loads(open(os.path.join(dst, "config_sensors.json")).read())
    assert sensors["sensors"]["global_shutter_lidar"] is False
    assert sensors["sensors"]["T_lidar_imu"] == [0.0, 0.0, 0.03, 0.0, 0.0, 1.0, 0.0]
    # committed source must be untouched
    src_sensors = json.loads(open(os.path.join(src, "config_sensors.json")).read())
    assert src_sensors["sensors"]["global_shutter_lidar"] is True


def test_viewer_inserts_standard_viewer_module(tmp_path):
    src = _make_src(tmp_path)
    dst = str(tmp_path / "out")
    prepare_config(src, dst, profile='sim', viewer=True)
    ros = json.loads(open(os.path.join(dst, "config_ros.json")).read())
    assert any("standard_viewer" in m for m in ros["glim_ros"]["extension_modules"])


def test_real_plus_viewer_compose(tmp_path):
    src = _make_src(tmp_path)
    dst = str(tmp_path / "out")
    prepare_config(src, dst, profile='real', viewer=True,
                   t_lidar_imu=[0, 0, 0, 0, 0, 1, 0])
    sensors = json.loads(open(os.path.join(dst, "config_sensors.json")).read())
    ros = json.loads(open(os.path.join(dst, "config_ros.json")).read())
    assert sensors["sensors"]["global_shutter_lidar"] is False
    assert any("standard_viewer" in m for m in ros["glim_ros"]["extension_modules"])


def test_bad_extrinsic_length_raises(tmp_path):
    src = _make_src(tmp_path)
    dst = str(tmp_path / "out")
    with pytest.raises(ValueError):
        prepare_config(src, dst, profile='real', t_lidar_imu=[1, 2, 3])


def test_load_extrinsic_yaml_reads_tum_list(tmp_path):
    p = tmp_path / "calib.yaml"
    p.write_text("# comment\nT_lidar_imu: [0.0, 0.0, 0.03, 0.0, 0.0, 1.0, 0.0]\n")
    assert load_extrinsic_yaml(str(p)) == [0.0, 0.0, 0.03, 0.0, 0.0, 1.0, 0.0]


def test_resolve_map_paths_default():
    out, dump = resolve_map_paths('/root/glim-humble-docker/maps', 'glim_map')
    assert out == '/root/glim-humble-docker/maps/glim_map'
    assert dump == '/root/glim-humble-docker/maps/glim_map/dump'


def test_resolve_map_paths_named():
    out, dump = resolve_map_paths('/maps', 'room_a')
    assert out == '/maps/room_a'
    assert dump == '/maps/room_a/dump'


def test_resolve_mode_sim():
    assert resolve_mode('sim') == ('/points', '/imu', 'sim', True)


def test_resolve_mode_real():
    assert resolve_mode('real') == ('/ouster/points', '/ouster/imu', 'real', False)


def test_resolve_mode_topics():
    assert resolve_mode('topics') == (None, None, 'real', False)


def test_resolve_mode_unknown_raises():
    with pytest.raises(ValueError):
        resolve_mode('bag')
