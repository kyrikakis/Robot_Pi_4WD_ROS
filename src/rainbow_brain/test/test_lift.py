"""Tests for pick-up detection: python3 -m pytest src/rainbow_brain/test."""

import math

from rainbow_brain.lift import LiftDetector

LEVEL = (0.0, 0.0, 0.0, 1.0)
RATE = 50.0


def tilted(deg):
    """Return the quaternion of a roll by deg."""
    h = math.radians(deg) / 2
    return (math.sin(h), 0.0, 0.0, math.cos(h))


def feed(d, samples, start=0.0, moving=False):
    """Feed (q, accel, gyro) samples at 50 Hz; return [(t, change)]."""
    out = []
    for i, (q, a, g) in enumerate(samples):
        t = start + i / RATE
        change = d.update(t, q, a, g, moving)
        if change:
            out.append((round(t, 2), change))
    return out


def still(seconds, q=LEVEL):
    """Return samples of a robot at rest."""
    return [(q, (0.0, 0.0, 0.02), (0.0, 0.0, 0.0))] * int(seconds * RATE)


def lift(up_ms2=2.0, accel_s=0.3, coast_s=0.5):
    """Return samples of a lift: accelerate up, coast, decelerate to a stop."""
    n, c = int(accel_s * RATE), int(coast_s * RATE)
    g = (0.0, 0.0, 0.1)
    return ([(LEVEL, (0.0, 0.0, up_ms2), g)] * n + [(LEVEL, (0.0, 0.0, 0.0), g)] * c
            + [(LEVEL, (0.0, 0.0, -up_ms2), g)] * n)


def test_rest_changes_nothing():
    """Rest changes nothing."""
    assert feed(LiftDetector(), still(10)) == []


def test_lifting_is_detected_and_putting_down_clears_it():
    """Lifting is detected and putting down clears it."""
    d = LiftDetector()
    out = feed(d, still(2) + lift() + [(LEVEL, (0.1, 0.0, 0.0), (0.0, 0.0, 0.2))] * 50 + still(3))
    assert [c for _, c in out] == ['lifted', 'down']
    assert not d.lifted


def test_a_short_bump_does_not_count():
    """A short bump does not count."""
    # A carpet edge: a jolt up and straight back down, no rise
    bump = ([(LEVEL, (0.5, 0.0, 6.0), (0.3, 0.0, 0.0))] * 3
            + [(LEVEL, (0.5, 0.0, -6.0), (0.3, 0.0, 0.0))] * 3)
    assert feed(LiftDetector(), still(2) + bump + still(2)) == []


def test_tilting_far_counts_a_small_tilt_does_not():
    """Tilting far counts a small tilt does not."""
    small = [(tilted(9), (0.3, 0.0, 0.0), (0.2, 0.0, 0.0))] * 50
    assert feed(LiftDetector(), still(2) + small) == []
    big = [(tilted(30), (0.3, 0.0, 0.0), (0.2, 0.0, 0.0))] * 50
    assert [c for _, c in feed(LiftDetector(), still(2) + big)] == ['lifted']


def test_driving_needs_a_bigger_rise():
    """Driving needs a bigger rise."""
    # ~6.5 cm: lifted at rest (5 cm here), not while commanded to move (8 cm)
    samples = still(2) + lift(up_ms2=1.2, accel_s=0.2, coast_s=0.1)

    def detector():
        return LiftDetector(rise_m=0.05, rise_moving_m=0.08)

    assert [c for _, c in feed(detector(), samples)] == ['lifted']
    assert feed(detector(), samples, moving=True) == []


def test_a_new_resting_surface_becomes_the_rest():
    """A new resting surface becomes the rest."""
    d = LiftDetector()
    feed(d, still(2) + lift() + still(3))          # lifted, put down
    # Resting tilted at 15 deg on a new surface: nothing new happens
    assert feed(d, still(5, q=tilted(15)), start=10.0) == []


def test_pick_phrase_never_repeats_the_last():
    from rainbow_brain.lift import pick_phrase
    phrases = ['Hey!', 'Wheee!', 'I can fly!']
    last = None
    for _ in range(50):
        phrase = pick_phrase(phrases, last)
        assert phrase in phrases and phrase != last
        last = phrase


def test_pick_phrase_single_and_empty():
    from rainbow_brain.lift import pick_phrase
    assert pick_phrase(['Hey!'], 'Hey!') == 'Hey!'
    assert pick_phrase([], 'Hey!') is None


def test_small_lifts_do_not_count_by_default():
    """A ~6.5 cm rise is under the default 10 cm."""
    samples = still(2) + lift(up_ms2=1.2, accel_s=0.2, coast_s=0.1)
    assert feed(LiftDetector(), samples) == []
