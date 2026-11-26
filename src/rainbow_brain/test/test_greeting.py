"""Tests for when the head turns to someone: python3 -m pytest src/rainbow_brain/test."""

from rainbow_brain.greeting import Attention


def run(a, seconds, start=0.0, seen=True, idle=True, enabled=True, busy_head=False, step=0.1):
    """Frames every step s; returns [(time, change)]."""
    out = []
    for i in range(int(round(seconds / step))):
        t = round(start + i * step, 6)     # no drift from adding 0.1 again and again
        if seen:
            a.seen(t)
        change = a.update(t, idle, enabled, busy_head)
        if change:
            out.append((round(t, 1), change))
    return out


def test_walks_in_looked_at_after_1_s_for_20_s():
    a = Attention()
    assert run(a, 30.0) == [(1.0, 'start'), (21.0, 'end')]


def test_one_look_per_presence():
    a = Attention()
    run(a, 30.0)                                    # looked at once, still there
    assert run(a, 60.0, start=30.0) == []
    # Leaves for 15 s (more than gone_after), comes back: a new look
    run(a, 15.0, start=90.0, seen=False)
    assert run(a, 3.0, start=105.0) == [(106.0, 'start')]


def test_a_short_gap_is_the_same_presence():
    a = Attention()
    run(a, 25.0)
    run(a, 5.0, start=25.0, seen=False)             # out of view 5 s only
    assert run(a, 5.0, start=30.0) == []


def test_not_while_driving_off_or_head_busy():
    for kwargs in ({'idle': False}, {'enabled': False}, {'busy_head': True}):
        a = Attention()
        assert run(a, 10.0, **kwargs) == []


def test_a_drive_ends_the_look():
    a = Attention()
    run(a, 5.0)                                     # looking since 1 s
    assert run(a, 1.0, start=5.0, idle=False)[0] == (5.0, 'end')


def test_switched_off_ends_the_look():
    a = Attention()
    run(a, 5.0)
    assert run(a, 1.0, start=5.0, enabled=False)[0] == (5.0, 'end')


def test_asked_to_watch_for_people_looks_again_at_once():
    a = Attention()
    run(a, 30.0)                                    # looked at, still there
    assert run(a, 10.0, start=30.0) == []
    a.rearm()                                       # "watch for people"
    assert run(a, 1.0, start=40.0)[0] == (40.0, 'start')


def test_a_conversation_extends_the_look():
    a = Attention()
    run(a, 5.0)                                     # looking since 1 s
    a.extend(5.0, 45.0)
    assert run(a, 60.0, start=5.0) == [(50.0, 'end')]
