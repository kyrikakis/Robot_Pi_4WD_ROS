"""Tests of lidar_power's decisions."""

from geometry_msgs.msg import Twist
from rainbow_drivers.lidar_power import is_motion, lidar_reasons, running_programs

STATUS = """\
rainbow_localization             STOPPED   Nov 26 07:32 PM
rainbow_slam                     STARTING
"""


def reasons(now=100.0, mapping=False, last_motion=None, wanted_at=None, lifted=False):
    """lidar_reasons with the default delays (30 s rest, 3 s wanted)."""
    return lidar_reasons(now, mapping, last_motion, 30.0, wanted_at, 3.0, lifted)


def test_running_programs():
    assert running_programs(STATUS) == ['rainbow_slam']
    assert running_programs('') == []


def test_idle_robot_keeps_it_off():
    assert reasons() == []


def test_motion_keeps_it_on_for_the_rest_delay():
    assert reasons(last_motion=75.0) == ['moving']
    assert reasons(last_motion=69.0) == []


def test_a_fresh_request_turns_it_on():
    assert reasons(wanted_at=98.0) == ['asked']
    assert reasons(wanted_at=96.0) == []


def test_lifted_wins_over_motion_and_requests():
    assert reasons(last_motion=99.0, wanted_at=99.0, lifted=True) == []


def test_mapping_wins_over_everything():
    assert reasons(mapping=True, lifted=True) == ['mapping']


def test_is_motion():
    t = Twist()
    assert not is_motion(t)
    t.angular.z = 0.5
    assert is_motion(t)
    t.angular.z = 0.0
    t.linear.y = 0.3  # skid-steer: ignored by the ESP
    assert not is_motion(t)
