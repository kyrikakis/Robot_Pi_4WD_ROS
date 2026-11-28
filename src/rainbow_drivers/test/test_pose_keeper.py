"""Tests of pose_keeper's pose checks and its file."""

import math
from types import SimpleNamespace

from rainbow_drivers.pose_keeper import has_moved, is_sure, load_pose, save_pose, yaw_of


def cov(pos_std, yaw_std):
    c = [0.0] * 36
    c[0] = c[7] = pos_std ** 2
    c[35] = yaw_std ** 2
    return c


def test_sure_only_when_tight():
    assert is_sure(cov(0.1, 0.1), 0.25, 0.2)
    assert not is_sure(cov(0.3, 0.1), 0.25, 0.2)
    assert not is_sure(cov(0.1, 0.3), 0.25, 0.2)
    # A fresh pose estimate (0.25 m) is not saved again until AMCL has settled it
    assert not is_sure(cov(0.25, 0.26), 0.25, 0.2)


def test_yaw_of():
    q = SimpleNamespace(x=0.0, y=0.0, z=math.sin(0.5), w=math.cos(0.5))
    assert abs(yaw_of(q) - 1.0) < 1e-9


def test_has_moved():
    assert has_moved(None, (0, 0, 0))
    assert not has_moved((1.0, 1.0, 3.13), (1.01, 1.0, -3.13))   # across +-pi
    assert has_moved((1.0, 1.0, 0.0), (1.05, 1.0, 0.0))
    assert has_moved((1.0, 1.0, 0.0), (1.0, 1.0, 0.1))


def test_save_and_load(tmp_path):
    path = str(tmp_path / 'last_pose' / 'home.yaml')
    save_pose(path, 'home', (1.2346, -0.5, 1.5))
    assert load_pose(path, 'home') == (1.235, -0.5, 1.5)
    assert load_pose(path, 'office') is None        # another map's pose
    assert load_pose(str(tmp_path / 'none.yaml'), 'home') is None


def test_load_rejects_a_broken_file(tmp_path):
    path = tmp_path / 'home.yaml'
    path.write_text('map: home\nx: [\n')
    assert load_pose(str(path), 'home') is None
    path.write_text('just text')
    assert load_pose(str(path), 'home') is None
