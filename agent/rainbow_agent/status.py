"""What the agent is doing, for Foxglove.

The fields are rainbow_interfaces/AgentStatus's. They go to the brain over the
link (LinkRobot.report), which publishes them as the agent part of /brain/status;
log lines go to /agent/log. Only changed fields are sent, plus the uptime, and
every few seconds the clocks (uptime, session age) are refreshed. Without a
link (SimRobot) nothing is sent, but the fields are still kept (for tests).

Phases: idle, transcribing, thinking, calling_skill, localizing.
"""

import asyncio
import time

REFRESH = 5.0


class Status:

    def __init__(self, robot, model=''):
        self.robot = robot
        self.started = time.monotonic()
        self.session_started = None     # time.monotonic() of the session's first turn
        self.fields = {'phase': 'idle', 'skill': '', 'model': str(model)}

    def set(self, **fields):
        self.fields.update(fields)
        self._report(fields)

    def log(self, level, text):
        log_line = getattr(self.robot, 'log_line', None)
        if log_line is not None:
            log_line(level, text)

    def session_seconds(self):
        if self.session_started is None:
            return 0.0
        return round(time.monotonic() - self.session_started, 1)

    def _report(self, fields):
        report = getattr(self.robot, 'report', None)
        if report is not None:
            report({**fields, 'uptime': round(time.monotonic() - self.started, 1),
                    'session_seconds': self.session_seconds()})

    async def refresh(self):
        """Send everything now, then refresh the clocks every few seconds (a task)."""
        self._report(self.fields)
        while True:
            await asyncio.sleep(REFRESH)
            self._report({})
