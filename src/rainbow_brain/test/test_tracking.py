"""Tests for the tracking logic (no ROS needed): python3 -m pytest src/rainbow_brain/test."""

import pytest
from rainbow_brain.tracking import head_target, labels, matches, normalize, pick, pixel_to_angles


def test_normalize_aliases():
    assert normalize('Me') == 'face'
    assert normalize('the ball') == 'sports ball'
    assert normalize('  My  Teddy  ') == 'teddy bear'
    assert normalize('my phone') == 'cell phone'
    assert normalize('backpack') == 'backpack'
    assert normalize('Stefanos') == 'stefanos'


def test_labels():
    assert labels('face: Stefanos') == {'face', 'stefanos'}
    assert labels('face: unknown') == {'face'}
    assert labels('sports ball') == {'sports ball'}
    assert labels('') == set()
    # COCO's whole-body 'person' must not count as a face
    assert labels('person') == {'person'}
    assert not matches('face', 'person')
    assert not matches('me', 'person')


def test_matches():
    assert matches('face', 'face: Stefanos')
    assert matches('me', 'face: unknown')
    assert matches('Stefanos', 'face: Stefanos')
    assert not matches('Maria', 'face: Stefanos')
    assert matches('ball', 'sports ball')
    assert not matches('ball', 'face: unknown')


def test_pick_largest_then_nearest():
    a, b = (100, 100, 50, 50), (500, 300, 80, 80)
    assert pick([a, b]) == b
    assert pick([a, b], previous=(120, 90)) == a
    assert pick([]) is None


def test_pixel_to_angles():
    # Centre: straight ahead
    assert pixel_to_angles(640, 360, 1280, 720, 66, 41) == pytest.approx((0.0, 0.0))
    # The image's left edge is half the FOV to the left (pan +)
    pan, tilt = pixel_to_angles(0, 360, 1280, 720, 66, 41)
    assert pan == pytest.approx(33.0)
    # The bottom edge is half the vertical FOV down (tilt +)
    pan, tilt = pixel_to_angles(640, 720, 1280, 720, 66, 41)
    assert tilt == pytest.approx(20.5)
    # Normalized boxes (the Hailo's): the same with width = height = 1
    assert pixel_to_angles(0.0, 0.5, 1.0, 1.0, 66, 41) == pytest.approx((33.0, 0.0))
    assert pixel_to_angles(0.5, 1.0, 1.0, 1.0, 66, 41) == pytest.approx((0.0, 20.5))


def test_head_target_uses_pose_at_frame_time():
    assert head_target(10.0, 0.0, 0.5, -0.5) is None          # inside the deadband
    assert head_target(10.0, 5.0, 20.0, -10.0, gain=0.5) == pytest.approx((20.0, 0.0))
