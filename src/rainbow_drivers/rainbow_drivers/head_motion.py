"""Smooth pan/tilt motion for the head (no ROS, so it can be tested on its own).

The servos have no position feedback and move at full speed to whatever angle
they are sent, so the smoothing happens here: the commanded angle moves towards
the target with limited speed and acceleration (a trapezoid, braking so it stops
on the target). A new target blends in from the current motion, so a stream of
targets (following a face) gives one continuous movement.

Both axes are scaled to arrive together, so a diagonal move is a straight line.

All angles are in degrees, times in seconds.
"""

import math


class Axis:

    def __init__(self, low, high):
        self.low = low
        self.high = high
        self.pos = 0.0
        self.vel = 0.0
        self.target = 0.0

    def step(self, dt, max_speed, accel):
        err = self.target - self.pos
        if err == 0.0 and self.vel == 0.0:
            return
        # The fastest speed from which it can still brake to a stop on the target,
        # braking accel * dt per step: v^2 / (2 a) + v dt / 2 = distance left
        brake = accel * (math.sqrt(0.25 * dt * dt + 2.0 * abs(err) / accel) - 0.5 * dt)
        wanted = math.copysign(min(max_speed, brake), err)
        dv = max(-accel * dt, min(accel * dt, wanted - self.vel))
        self.vel += dv
        new = self.pos + self.vel * dt
        # Arrived (or would pass it) at a speed it can stop from in one step: stop there
        passed = (self.target - new) * err <= 0.0
        if passed and abs(self.vel) <= accel * dt * (1.0 + 1e-9):
            self.pos = self.target
            self.vel = 0.0
        else:
            self.pos = max(self.low, min(self.high, new))

    def stopping_point(self, accel):
        if accel <= 0.0:
            return self.pos
        return self.pos + math.copysign(self.vel * self.vel / (2.0 * accel), self.vel)

    @property
    def still(self):
        return self.pos == self.target and self.vel == 0.0


class HeadMotion:
    """Pan/tilt motion with a settle time after every move.

    settle time = settle_base + settle_per_deg * degrees travelled since the
    head last settled (the larger the move, the more the head wobbles at the end).
    """

    def __init__(self, pan_limits, tilt_limits, speed, accel,
                 settle_base=0.2, settle_per_deg=0.005):
        self.pan = Axis(*pan_limits)
        self.tilt = Axis(*tilt_limits)
        self.speed = speed
        self.accel = accel
        self.settle_base = settle_base
        self.settle_per_deg = settle_per_deg
        self.move_speed = speed
        self.scale = (1.0, 1.0)
        self.travelled = 0.0
        self.stopped_at = None   # time both axes stopped, None while moving
        self.settled = True

    @staticmethod
    def clamp(value, low, high):
        return max(low, min(high, value))

    def set_target(self, pan=None, tilt=None, speed=None):
        """Aim at (pan, tilt) (None keeps that axis' target); returns the clamped target."""
        if pan is not None:
            self.pan.target = self.clamp(pan, self.pan.low, self.pan.high)
        if tilt is not None:
            self.tilt.target = self.clamp(tilt, self.tilt.low, self.tilt.high)
        self.move_speed = speed if speed and speed > 0.0 else self.speed
        # Scale each axis by its share of the move so both arrive together
        d_pan = abs(self.pan.target - self.pan.pos)
        d_tilt = abs(self.tilt.target - self.tilt.pos)
        longest = max(d_pan, d_tilt)
        if longest > 0.0:
            # Never below 10%: an axis that is already moving must still be able to brake
            self.scale = (max(0.1, d_pan / longest), max(0.1, d_tilt / longest))
        if not (self.pan.still and self.tilt.still):
            self.settled = False
            self.stopped_at = None
        return self.pan.target, self.tilt.target

    def stop(self):
        """Brake to a stop as soon as possible; returns where it will stop."""
        self.scale = (1.0, 1.0)
        for axis in (self.pan, self.tilt):
            axis.target = self.clamp(axis.stopping_point(self.accel), axis.low, axis.high)
        return self.pan.target, self.tilt.target

    def step(self, now, dt):
        """Advance by dt; returns True while the head is moving or settling."""
        before = (self.pan.pos, self.tilt.pos)
        for axis, scale in ((self.pan, self.scale[0]), (self.tilt, self.scale[1])):
            axis.step(dt, self.move_speed * scale, self.accel * scale)
        moved = abs(self.pan.pos - before[0]) + abs(self.tilt.pos - before[1])
        self.travelled += moved
        if moved > 0.0 or not (self.pan.still and self.tilt.still):
            self.settled = False
            self.stopped_at = None
            return True
        if self.settled:
            return False
        if self.stopped_at is None:
            self.stopped_at = now
        if now - self.stopped_at >= self.settle_base + self.settle_per_deg * self.travelled:
            self.settled = True
            self.travelled = 0.0
            return False
        return True

    @property
    def position(self):
        return self.pan.pos, self.tilt.pos

    @property
    def target(self):
        return self.pan.target, self.tilt.target
