import json

import pytest

from go2_bringup.go2_sport_bridge import twist_to_sport_params, ROBOT_SPORT_API_ID_MOVE


def test_forward_velocity():
    p = twist_to_sport_params(linear_x=0.5, angular_z=0.0)
    assert p['api_id'] == ROBOT_SPORT_API_ID_MOVE
    params = json.loads(p['parameter'])
    assert params['x'] == pytest.approx(0.5)
    assert params['y'] == 0.0
    assert params['z'] == 0.0


def test_turn_only():
    p = twist_to_sport_params(linear_x=0.0, angular_z=0.3)
    params = json.loads(p['parameter'])
    assert params['x'] == 0.0
    assert params['z'] == pytest.approx(0.3)


def test_combined_motion():
    p = twist_to_sport_params(linear_x=0.5, angular_z=-0.2)
    params = json.loads(p['parameter'])
    assert params['x'] == pytest.approx(0.5)
    assert params['z'] == pytest.approx(-0.2)


def test_zero_gives_zero():
    p = twist_to_sport_params(linear_x=0.0, angular_z=0.0)
    params = json.loads(p['parameter'])
    assert params['x'] == 0.0
    assert params['y'] == 0.0
    assert params['z'] == 0.0


def test_lateral_is_always_zero():
    p = twist_to_sport_params(linear_x=0.5, angular_z=0.3)
    params = json.loads(p['parameter'])
    assert params['y'] == 0.0


from go2_bringup.go2_sport_bridge import clamp_twist


def test_clamp_passthrough_within_caps():
    assert clamp_twist(0.2, 0.3, max_vx=0.3, max_vyaw=0.5) == (0.2, 0.3)


def test_clamp_limits_forward():
    assert clamp_twist(1.5, 0.0, max_vx=0.3, max_vyaw=0.5) == (0.3, 0.0)


def test_clamp_limits_reverse_and_yaw_sign():
    vx, vyaw = clamp_twist(-1.5, -2.0, max_vx=0.3, max_vyaw=0.5)
    assert vx == -0.3
    assert vyaw == -0.5


from go2_bringup.go2_sport_bridge import gated_velocity


def test_gate_disabled_forces_zero():
    assert gated_velocity(False, 0.5, 0.4, max_vx=0.3, max_vyaw=0.5) == (0.0, 0.0)


def test_gate_enabled_clamps():
    assert gated_velocity(True, 1.5, 0.4, max_vx=0.3, max_vyaw=0.5) == (0.3, 0.4)


from go2_bringup.go2_sport_bridge import mode_to_sport_request


def test_stand_up_maps_to_1004_empty_param():
    p = mode_to_sport_request('stand_up')
    assert p['api_id'] == 1004
    assert p['parameter'] == ''


def test_balance_stand_maps_to_1002():
    assert mode_to_sport_request('balance_stand')['api_id'] == 1002


def test_stop_move_maps_to_1003():
    assert mode_to_sport_request('stop_move')['api_id'] == 1003


def test_damp_maps_to_1001():
    assert mode_to_sport_request('damp')['api_id'] == 1001


def test_mode_param_is_always_empty():
    for mode in ('stand_up', 'balance_stand', 'stop_move', 'damp'):
        assert mode_to_sport_request(mode)['parameter'] == ''


def test_unknown_mode_returns_none():
    assert mode_to_sport_request('moonwalk') is None


def test_mode_is_whitespace_tolerant():
    # caller strips, but the map itself uses exact keys; verify exact-key behavior
    assert mode_to_sport_request('stand_up') is not None
    assert mode_to_sport_request('stand up') is None
