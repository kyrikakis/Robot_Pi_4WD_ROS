"""When to turn the head to someone who walks in (no ROS, so it can be tested).

A person (COCO's person box, e.g. legs) or a face in view for notice_after
seconds while the robot is idle starts one look: the brain has the tracker follow
"face" (which falls back to the person's body, aimed at its head), face
recognition then names them and the agent greets them. The look ends after
follow_for seconds. One look per presence: the person has to be out of view for
gone_after seconds before they are noticed again (else a person standing there
would be looked at again as soon as the head let go).
"""


class Attention:

    def __init__(self, notice_after=1.0, follow_for=20.0, gone_after=10.0):
        self.notice_after = notice_after
        self.follow_for = follow_for
        self.gone_after = gone_after
        self.since = None          # start of the current presence
        self.last_seen = None
        self.looked = False        # this presence already had its look
        self.attending = False     # a look is running (the brain started the tracker)
        self.started = None

    def seen(self, now):
        """Someone (person or face) is in view in this frame."""
        if self.last_seen is None or now - self.last_seen > self.gone_after:
            self.since = now       # a new presence
            self.looked = False
        elif self.since is None:
            self.since = now
        self.last_seen = now

    def release(self):
        """Someone else took the head (a look, track or drive): the look is over."""
        self.attending = False

    def update(self, now, idle, enabled, busy_head):
        """'start' a look, 'end' it, or None. idle: not driving and the battery is fine;
        busy_head: the head follows something it was asked to."""
        if self.attending:
            if now - self.started >= self.follow_for or not enabled or not idle:
                self.attending = False
                return 'end'
            return None
        if self.last_seen is not None and now - self.last_seen > self.gone_after:
            self.since, self.looked = None, False
        in_view = self.last_seen is not None and now - self.last_seen < 1.0
        if (enabled and idle and not busy_head and not self.looked and in_view
                and self.since is not None and now - self.since >= self.notice_after):
            self.attending = self.looked = True
            self.started = now
            return 'start'
        return None
