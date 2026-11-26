"""Low-battery mode logic (no ROS, so it can be tested on its own).

The battery reading dips while the robot drives, so one low reading means
little: the mode starts once every reading for low_for seconds was at or below
low_volts, and ends once every reading for ok_for seconds was at or above
ok_volts (a charged battery; 7.6 V is well above where the pack sags to).
Readings that stop coming (ESP off) change nothing.
"""


class BatteryMonitor:

    def __init__(self, low_volts=7.0, low_for=30.0, ok_volts=7.6, ok_for=60.0):
        self.low_volts, self.low_for = low_volts, low_for
        self.ok_volts, self.ok_for = ok_volts, ok_for
        self.low = False
        self.since = None      # when the readings started meeting the switch condition

    def update(self, now, volts):
        """Feed one reading; returns 'low' or 'ok' when the mode changes, else None."""
        if volts is None or volts <= 0.0:
            return None            # no reading (0 V: the ESP reports nothing yet)
        switching = volts >= self.ok_volts if self.low else volts <= self.low_volts
        if not switching:
            self.since = None
            return None
        if self.since is None:
            self.since = now
        if now - self.since < (self.ok_for if self.low else self.low_for):
            return None
        self.low = not self.low
        self.since = None
        return 'low' if self.low else 'ok'
