from go2_rgb_odom_recorder.bag_trim import keep_rgb

# Pose timestamps (ns) at 10 Hz: 1.0s, 1.1s, 1.5s, 1.6s. Note the 0.3s gap
# between 1.1s and 1.5s (icp tracking loss). tol = 0.1s = 100_000_000 ns.
POSES = [1_000_000_000, 1_100_000_000, 1_500_000_000, 1_600_000_000]
TOL = 100_000_000


def test_front_pose_less_dropped():
    # RGB at 0.5s, well before the first pose -> no pose within tol -> drop.
    assert keep_rgb(500_000_000, POSES, TOL) is False


def test_matched_kept():
    # RGB at 1.05s, 50 ms from the 1.0s/1.1s poses -> keep.
    assert keep_rgb(1_050_000_000, POSES, TOL) is True


def test_mid_gap_dropped():
    # RGB at 1.3s, 200 ms from 1.1s and 1.5s (the tracking-loss gap) -> drop.
    assert keep_rgb(1_300_000_000, POSES, TOL) is False


def test_tail_pose_less_dropped():
    # RGB at 2.0s, 400 ms after the last pose -> drop.
    assert keep_rgb(2_000_000_000, POSES, TOL) is False


def test_boundary_exactly_tol_kept():
    # RGB exactly tol away from a pose (1.0s + 0.1s = 1.1s pose, or 0.9s) -> keep (<=).
    assert keep_rgb(900_000_000, POSES, TOL) is True


def test_empty_poses_drops_all():
    assert keep_rgb(1_000_000_000, [], TOL) is False
