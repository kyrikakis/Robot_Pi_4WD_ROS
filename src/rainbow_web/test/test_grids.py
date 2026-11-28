import math

import numpy as np
from rainbow_web.grids import (clamp, compact_scan, compose, costmap_rgba, drive_command,
                               jpeg_from_cdr, map_rgba, quaternion_from_yaw, thin_path,
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


def test_jpeg_from_cdr():
    try:
        from rclpy.serialization import serialize_message
        from sensor_msgs.msg import CompressedImage
    except ImportError:
        return
    for frame_id, fmt in (('', 'jpeg'), ('camera_link', 'rgb8; jpeg compressed bgr8'),
                          ('abc', 'x')):
        msg = CompressedImage(format=fmt, data=bytes(range(256)) * 3)
        msg.header.frame_id = frame_id
        msg.header.stamp.sec = 12
        assert jpeg_from_cdr(serialize_message(msg)) == bytes(range(256)) * 3


def test_drive_command_limits():
    assert drive_command(1, 0, 1, 0.3, 0.1, 1.0) == (0.3, 0.0)
    lin, ang = drive_command(-5, 3, 1, 0.3, 0.1, 1.0)
    assert lin == -0.1 and ang == 1.0
    lin, ang = drive_command(1, 1, 0.2, 0.3, 0.1, 1.0)
    assert math.isclose(lin, 0.06) and ang == 0.5   # turning keeps half speed
    assert clamp(2, 0, 1) == 1


def test_split_access_units():
    from rainbow_web.grids import AUD, is_keyframe, split_access_units
    sps, idr, p = b'\x00\x00\x00\x01\x67ab', b'\x00\x00\x01\x65cd', b'\x00\x00\x01\x41ef'
    stream = AUD + b'\x10' + sps + idr + AUD + b'\x30' + p + AUD + b'\x30'
    units, rest = split_access_units(stream)
    assert units == [AUD + b'\x10' + sps + idr, AUD + b'\x30' + p]
    assert rest == AUD + b'\x30'                       # not complete until the next one
    assert is_keyframe(units[0]) and not is_keyframe(units[1])
    assert split_access_units(b'\x00\x00') == ([], b'\x00\x00')
