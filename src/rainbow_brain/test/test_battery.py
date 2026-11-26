"""Tests for the low-battery mode logic: python3 -m pytest src/rainbow_brain/test."""

from rainbow_brain.battery import BatteryMonitor


def feed(m, readings, start=0.0, period=1.0):
    """Readings at 1 Hz (like /battery); returns [(time, change)] for the changes."""
    out = []
    for i, v in enumerate(readings):
        t = start + i * period
        change = m.update(t, v)
        if change:
            out.append((t, change))
    return out


def test_low_only_after_30_s_at_or_below_7():
    m = BatteryMonitor()
    assert feed(m, [7.0] * 30) == []           # 0..29 s: not yet
    assert m.update(30.0, 7.0) == 'low'
    assert m.low


def test_dips_while_driving_do_not_count():
    m = BatteryMonitor()
    # Sags to 6.9 for 20 s while driving, recovers to 7.2 at rest, sags again
    readings = ([6.9] * 20 + [7.2] * 5) * 4
    assert feed(m, readings) == []
    assert not m.low


def test_ends_only_after_a_minute_at_or_above_7_6():
    m = BatteryMonitor()
    feed(m, [6.8] * 31)
    assert m.low
    assert feed(m, [7.4] * 120, start=40.0) == []      # charging, not there yet
    assert feed(m, [7.7] * 60, start=200.0) == []       # 200..259 s
    assert m.update(260.0, 7.7) == 'ok'
    assert not m.low


def test_missing_readings_change_nothing():
    m = BatteryMonitor()
    assert feed(m, [None, 0.0] * 50) == []
    assert not m.low
