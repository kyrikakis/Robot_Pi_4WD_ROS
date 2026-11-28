"""Tests of map_tf_keeper's choice of transforms."""

from geometry_msgs.msg import TransformStamped
from rainbow_drivers.map_tf_keeper import find_map_odom
from tf2_msgs.msg import TFMessage


def tf(parent, child, sec=10, nanosec=0):
    t = TransformStamped()
    t.header.frame_id = parent
    t.child_frame_id = child
    t.header.stamp.sec = sec
    t.header.stamp.nanosec = nanosec
    return t


def test_finds_amcls_map_to_odom():
    msg = TFMessage(transforms=[tf('odom', 'base_footprint'), tf('map', 'odom')])
    assert find_map_odom(msg, set()).child_frame_id == 'odom'


def test_ignores_other_transforms():
    assert find_map_odom(TFMessage(transforms=[tf('odom', 'base_footprint')]), set()) is None


def test_ignores_its_own_republished_transform():
    msg = TFMessage(transforms=[tf('map', 'odom', 12, 5)])
    assert find_map_odom(msg, {(12, 5)}) is None
    assert find_map_odom(msg, {(12, 6)}) is not None
