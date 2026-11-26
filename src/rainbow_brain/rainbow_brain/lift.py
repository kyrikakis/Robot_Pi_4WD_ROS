"""Pick-up detection from the BNO055 (no ROS, so it can be tested on its own).

Recorded 2026-11-26 (robot in the hand, on the desk, on the floor, pushed over a
carpet edge): at rest the tilt stays within 0.5 deg and the acceleration under
0.1 m/s^2. A lift raises the robot: every pick-up rose >= 7 cm (desk to floor,
the smallest) and up to ~0.8 m; carrying tilts it up to 44 deg. Pushed over a
carpet edge it tilted 9 deg and jolted (6 m/s^2); the rise estimate below
peaked at 2.7 cm. Replayed, 5 cm flags the three lifts within ~1 s and not the push.

So: lifted when the robot rises more than rise_m (rise_moving_m while it is
commanded to move: bumps while driving are not measured yet), estimated by
integrating the vertical acceleration (world frame, gravity already removed by
the BNO055) with leaks against drift and a reset whenever it is still; or when
it tilts more than tilt_deg from its resting attitude. Put down once it has been
still for still_s. The resting attitude follows the robot while it rests, so a
new surface becomes the new rest.

Inputs per sample: the BNO055's orientation quaternion (body -> world, x y z w),
linear acceleration (body frame, m/s^2, without gravity), angular velocity
(rad/s), and whether the robot is commanded to move.
"""

import math
import random

STILL_GYRO = 0.05       # rad/s
STILL_ACCEL = 0.2       # m/s^2
VELOCITY_LEAK_S = 4.0   # time constant of the leak on the vertical speed
REST_FOLLOW = 0.02      # per still sample: how fast the resting attitude follows


def gravity_in_body(q):
    """World up (0, 0, 1) in the body frame, from a body -> world quaternion (x, y, z, w)."""
    x, y, z, w = q
    return (2 * (x * z - w * y), 2 * (w * x + y * z), w * w - x * x - y * y + z * z)


def vertical_accel(q, a):
    """World-frame vertical component of a body-frame acceleration."""
    x, y, z, w = q
    return (2 * (x * z - w * y) * a[0] + 2 * (y * z + w * x) * a[1]
            + (1 - 2 * (x * x + y * y)) * a[2])


def angle_deg(u, v):
    """Angle between two vectors, degrees."""
    nu = math.sqrt(sum(c * c for c in u)) or 1.0
    nv = math.sqrt(sum(c * c for c in v)) or 1.0
    cos = max(-1.0, min(1.0, sum(a * b for a, b in zip(u, v)) / (nu * nv)))
    return math.degrees(math.acos(cos))


class LiftDetector:
    """Lifted / put down from BNO055 samples (see the module docstring)."""

    def __init__(self, rise_m=0.10, rise_moving_m=0.12, tilt_deg=20.0, still_s=1.0):
        self.rise_m, self.rise_moving_m = rise_m, rise_moving_m
        self.tilt_deg, self.still_s = tilt_deg, still_s
        self.lifted = False
        self.rest = None          # resting "up" in the body frame
        self.last_t = None
        self.speed = 0.0          # estimated vertical speed (m/s)
        self.rise = 0.0           # estimated rise (m) since the robot was last still
        self.still_since = None
        self.why = ''

    def update(self, t, q, accel, gyro, moving=False):
        """Feed one sample; returns 'lifted' or 'down' when the state changes, else None."""
        dt = 0.0 if self.last_t is None else max(0.0, min(0.2, t - self.last_t))
        self.last_t = t
        up = gravity_in_body(q)
        still = (math.sqrt(sum(c * c for c in gyro)) < STILL_GYRO
                 and math.sqrt(sum(c * c for c in accel)) < STILL_ACCEL)
        if self.rest is None:
            self.rest = up

        if still:
            self.speed = self.rise = 0.0
            self.still_since = t if self.still_since is None else self.still_since
            if not self.lifted:
                self.rest = tuple(r + REST_FOLLOW * (u - r) for r, u in zip(self.rest, up))
        else:
            self.still_since = None
            self.speed += vertical_accel(q, accel) * dt
            self.speed *= math.exp(-dt / VELOCITY_LEAK_S)
            self.rise += self.speed * dt

        if self.lifted:
            if self.still_since is not None and t - self.still_since >= self.still_s:
                self.lifted = False
                self.rest = up
                self.why = ''
                return 'down'
            return None

        tilt = angle_deg(up, self.rest)
        limit = self.rise_moving_m if moving else self.rise_m
        if self.rise > limit:
            self.why = f'rose {self.rise * 100:.0f} cm'
        elif tilt > self.tilt_deg:
            self.why = f'tilted {tilt:.0f} deg'
        else:
            return None
        self.lifted = True
        return 'lifted'


# What the robot says when picked up and put down (the brain's lift_phrases /
# put_down_phrases parameters; spoken locally, no agent needed)
LIFT_PHRASES = ['Hey!', 'Wheee!', 'Whoa, where are we going?', 'Hey, put me down!',
                'I can fly!', 'Careful, I am delicate!']
PUT_DOWN_PHRASES = ['Phew, solid ground!', 'Thanks for the ride!', 'Home sweet floor.',
                    'That was fun!', 'Again, again!']


def pick_phrase(phrases, last=None, choice=random.choice):
    """Return a random phrase, not the one said last time; None for an empty list."""
    fresh = [p for p in phrases if p != last] or list(phrases)
    return choice(fresh) if fresh else None
