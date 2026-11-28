import math

import numpy as np
from rainbow_web.grids import (clamp, compact_scan, compose, costmap_rgba, drive_command,
                               map_rgba, quaternion_from_yaw, thin_path,
                               yaw_from_quaternion)


def test_yaw_round_trip():
    for yaw in (-3.0, -1.0, 0.0, 0.5, 2.9):
        assert math.isclose(yaw_from_quaternion(*quaternion_from_yaw(yaw)), yaw, abs_tol=1e-9)


def test_compose():
    x, y, yaw = compose((1.0, 2.0, math.pi / 2), (1.0, 0.0, 0.1))
    assert math.isclose(x, 1.0, abs_tol=1e-9)
    assert math.isclose(y, 3.0, abs_tol=1e-9)
    assert math.isclose(yaw, math.pi / 2 + 0.1)


def test_map_rgba_flips_and_colours():
    # 2 x 2 grid: row 0 (bottom) = free, unknown; row 1 (top) = occupied, free
    img = map_rgba([0, -1, 100, 0], 2, 2)
    assert img.shape == (2, 2, 4)
    assert tuple(img[0, 0]) == (24, 24, 24, 255)     # top-left: occupied
    assert tuple(img[1, 0]) == (232, 232, 232, 255)  # bottom-left: free
    assert img[1, 1, 3] == 0                         # unknown: transparent


def test_costmap_rgba():
    img = costmap_rgba([0, 50, 99, 100], 4, 1)
    assert img[0, 0, 3] == 0
    assert 0 < img[0, 1, 3] < 210
    assert tuple(img[0, 2]) == (60, 0, 255, 210)
    assert tuple(img[0, 3]) == (255, 0, 255, 220)
    assert img.dtype == np.uint8


def test_compact_scan():
    step, values = compact_scan([1.0, float('inf'), 0.05, 2.345] * 200, 0.1, 12.0, 360)
    assert step == 3
    assert len(values) == math.ceil(800 / 3)
    assert values[0] == 100
    assert values[1] == 235   # index 3
    assert 0 in values


def test_thin_path_keeps_ends():
    pts = [(i * 0.01, 0.0) for i in range(101)]
    out = thin_path(pts, 0.05)
    assert out[0] == (0.0, 0.0)
    assert out[-1] == (1.0, 0.0)
    assert len(out) == 21


def test_drive_command_limits():
    assert drive_command(1, 0, 1, 0.3, 0.1, 1.0) == (0.3, 0.0)
    lin, ang = drive_command(-5, 3, 1, 0.3, 0.1, 1.0)
    assert lin == -0.1 and ang == 1.0
    lin, ang = drive_command(1, 1, 0.2, 0.3, 0.1, 1.0)
    assert math.isclose(lin, 0.06) and ang == 0.5   # turning keeps half speed
    assert clamp(2, 0, 1) == 1
