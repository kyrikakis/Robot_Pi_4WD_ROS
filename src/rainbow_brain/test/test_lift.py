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
    small = [(tilted(20), (0.3, 0.0, 0.0), (0.2, 0.0, 0.0))] * 50
    assert feed(LiftDetector(), still(2) + small) == []
    big = [(tilted(40), (0.3, 0.0, 0.0), (0.2, 0.0, 0.0))] * 50
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
    """A ~6.5 cm rise is under the default 30 cm."""
    samples = still(2) + lift(up_ms2=1.2, accel_s=0.2, coast_s=0.1)
    assert feed(LiftDetector(), samples) == []


def lower(down_ms2=2.0, accel_s=0.25, coast_s=0.2):
    """Return samples of lowering: accelerate down, coast, brake to a stop."""
    n, c = int(accel_s * RATE), int(coast_s * RATE)
    g = (0.0, 0.0, 0.2)
    return ([(LEVEL, (0.0, 0.0, -down_ms2), g)] * n + [(LEVEL, (0.0, 0.0, 0.0), g)] * c
            + [(LEVEL, (0.0, 0.0, down_ms2), g)] * n)


def quiet(seconds):
    """Return samples of a robot settling: quiet, but not strictly still."""
    return [(LEVEL, (0.3, 0.0, 0.0), (0.0, 0.0, 0.07))] * int(seconds * RATE)


def handled(seconds):
    """Return samples of a hand moving the robot sideways (no vertical motion)."""
    return [(LEVEL, (1.0, 0.0, 0.0), (0.0, 0.0, 0.5))] * int(seconds * RATE)


def lifted_detector():
    d = LiftDetector()
    assert [c for _, c in feed(d, still(2) + lift())] == ['lifted']
    return d


def test_a_landing_is_put_down_quickly():
    """Coming down and braking, then quiet for 0.25 s, is a put-down."""
    d = lifted_detector()
    landing = lower()
    start = 10.0
    out = feed(d, handled(0.5) + landing + quiet(1.0), start=start)
    braked = start + (int(0.5 * RATE) + len(landing)) / RATE
    assert [c for _, c in out] == ['down']
    assert d.why == 'landed'
    assert out[0][0] - braked <= 0.35


def test_a_stop_in_mid_air_is_not_put_down():
    """Lowered and stopped in the hand, which then moves on: still lifted."""
    d = lifted_detector()
    assert feed(d, handled(0.5) + lower() + quiet(0.15) + handled(2.0), start=10.0) == []
    assert d.lifted


def test_a_slow_put_down_falls_back_to_stillness():
    """Lowered too slowly to show a descent: put down after still_s still."""
    d = lifted_detector()
    # A moment held perfectly still first (resets the speed the lift left behind)
    slow = still(0.1) + lower(down_ms2=0.4, accel_s=0.25, coast_s=1.0)
    out = feed(d, slow + quiet(0.5) + still(1.5), start=10.0)
    assert [c for _, c in out] == ['down']
    assert d.why == 'still 1 s'


def test_desk_to_floor_is_picked_up_and_put_down():
    """Raised 3 cm off the desk, lowered ~45 cm to the floor: lifted, then landed."""
    d = LiftDetector()
    raise_a_little = lift(up_ms2=1.0, accel_s=0.15, coast_s=0.1)    # ~3.5 cm
    out = feed(d, still(2) + raise_a_little + lower(down_ms2=3.0, accel_s=0.3, coast_s=1.0)
               + quiet(1.0))
    assert [c for _, c in out] == ['lifted', 'down']
    assert d.why == 'landed'


def test_a_small_raise_alone_is_not_a_lift():
    """Raised ~3.5 cm and set back: nothing."""
    out = feed(LiftDetector(), still(2) + lift(up_ms2=1.0, accel_s=0.15, coast_s=0.1)
               + still(2))
    assert out == []
