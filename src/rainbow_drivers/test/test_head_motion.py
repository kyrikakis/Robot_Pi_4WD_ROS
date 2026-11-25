"""Tests for the head motion profile (no ROS): python3 -m pytest src/rainbow_drivers/test."""

import pytest
from rainbow_drivers.head_motion import HeadMotion

DT = 0.02


def head(**kwargs):
    args = {'speed': 30.0, 'accel': 60.0, 'settle_base': 0.2, 'settle_per_deg': 0.005}
    args.update(kwargs)
    return HeadMotion((-40.0, 38.0), (-36.0, 19.0), **args)


def run(motion, seconds, t0=0.0, record=None):
    """Step for up to `seconds`; returns the time it stopped moving and settling, or None."""
    t = t0
    for _ in range(int(round(seconds / DT))):
        t += DT
        busy = motion.step(t, DT)
        if record is not None:
            record.append((t, motion.position))
        if not busy:
            return t
    return None


def test_reaches_target_and_respects_limits():
    m = head()
    trace = []
    assert m.set_target(30.0, -10.0) == (30.0, -10.0)
    done = run(m, 5.0, record=trace)
    assert done is not None
    assert m.position == (30.0, -10.0)
    speeds = [abs(b[1][0] - a[1][0]) / DT for a, b in zip(trace, trace[1:])]
    assert max(speeds) <= 30.0 + 1e-6
    accels = [abs(b - a) / DT for a, b in zip(speeds, speeds[1:])]
    assert max(accels) <= 60.0 + 1e-6
    # Pan never goes past its target
    assert max(p[0] for _, p in trace) <= 30.0


def test_trapezoid_duration():
    # 30 deg at 30 deg/s with 60 deg/s^2: 0.5 s up, 0.5 s cruise, 0.5 s down = 1.5 s,
    # then settle 0.2 + 0.005 * 30 = 0.35 s
    m = head()
    m.set_target(30.0, 0.0)
    done = run(m, 5.0)
    assert done == pytest.approx(1.5 + 0.35, abs=0.1)


def test_axes_arrive_together():
    m = head()
    m.set_target(30.0, 10.0)
    arrived = {}
    t = 0.0
    while t < 5.0:
        t += DT
        m.step(t, DT)
        for name, axis in (('pan', m.pan), ('tilt', m.tilt)):
            if axis.still and name not in arrived:
                arrived[name] = t
    assert arrived['pan'] == pytest.approx(arrived['tilt'], abs=0.1)


def test_clamped_to_limits():
    m = head()
    assert m.set_target(90.0, -90.0) == (38.0, -36.0)
    run(m, 10.0)
    assert m.position == (38.0, -36.0)


def test_retarget_while_moving_is_continuous():
    m = head()
    m.set_target(30.0, 0.0)
    trace = []
    run(m, 0.6, record=trace)
    assert m.pan.vel > 0.0
    m.set_target(-20.0, 0.0)   # reverse mid-move: must slow down first, no jump
    run(m, 5.0, t0=trace[-1][0], record=trace)
    assert m.position == (-20.0, 0.0)
    steps = [abs(b[1][0] - a[1][0]) for a, b in zip(trace, trace[1:])]
    assert max(steps) <= 30.0 * DT + 1e-6


def test_stream_of_targets_follows_smoothly():
    # Following a face: a new target every 0.1 s, moving slowly
    m = head()
    t = 0.0
    for i in range(30):
        m.set_target(i * 0.5, 0.0)
        for _ in range(5):
            t += DT
            m.step(t, DT)
    assert m.position[0] == pytest.approx(14.5, abs=2.0)


def test_stop_brakes_without_jump():
    m = head()
    m.set_target(38.0, 0.0)
    run(m, 0.8)
    pos = m.position[0]
    vel = m.pan.vel
    stop_at = m.stop()[0]
    assert stop_at == pytest.approx(pos + vel * vel / (2 * 60.0))
    run(m, 3.0, t0=0.8)
    assert m.position[0] == pytest.approx(stop_at)
    assert m.pan.vel == 0.0


def test_settle_time_scales_with_move():
    small, large = head(), head()
    small.set_target(5.0, 0.0)
    large.set_target(35.0, 0.0)
    t_small = run(small, 5.0)
    t_large = run(large, 5.0)
    assert t_large > t_small
    # Once settled, nothing to do
    assert not large.step(10.0, DT)
    assert large.settled


def test_same_target_again_does_not_unsettle():
    m = head()
    m.set_target(10.0, 0.0)
    run(m, 5.0)
    m.set_target(10.0, 0.0)
    assert m.settled
    assert not m.step(20.0, DT)


def test_custom_speed():
    m = head()
    m.set_target(30.0, 0.0, speed=10.0)
    trace = []
    run(m, 10.0, record=trace)
    speeds = [abs(b[1][0] - a[1][0]) / DT for a, b in zip(trace, trace[1:])]
    assert max(speeds) <= 10.0 + 1e-6


def test_fast_default_no_overshoot():
    # The robot's defaults (400 deg/s, 4000 deg/s^2): 30 deg in ~0.2 s, no overshoot
    m = head(speed=400.0, accel=4000.0)
    m.set_target(30.0, 0.0)
    trace = []
    done = run(m, 2.0, record=trace)
    moving = [t for t, p in trace if p[0] < 30.0]
    assert 0.12 <= moving[-1] <= 0.3
    assert max(p[0] for _, p in trace) <= 30.0
    assert done == pytest.approx(moving[-1] + 0.35, abs=0.08)
