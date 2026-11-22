r"""The listening state machine (no ROS, no audio device).

Fed one 80 ms chunk (1280 samples) at a time, it returns events:
  ('wake',)                        the wake word: recording starts
  ('stop',)                        the stop word (any time, even while the robot talks)
  ('utterance', samples, follow_up)  a finished utterance (int16); follow_up: no wake word
  ('nothing',)                     woke, but nobody spoke

    LISTENING --wake word--> RECORDING --end_silence of quiet, or max_seconds--> utterance
       |  \\--speech within follow_up s after the robot spoke--> RECORDING (no wake word)
       \\--stop word (always)--> stop
While the robot is speaking (speaking(True)) only the stop word is listened for.
"""

from collections import deque
import time

import numpy as np

CHUNK = 1280
SECONDS = CHUNK / 16000.0


class Vad:
    """webrtcvad over a chunk: voiced if at least 2 of its four 20 ms frames are."""

    def __init__(self, aggressiveness=2):
        import webrtcvad
        self.vad = webrtcvad.Vad(aggressiveness)

    def __call__(self, chunk):
        frames = np.asarray(chunk, dtype=np.int16).reshape(4, 320)
        return sum(self.vad.is_speech(f.tobytes(), 16000) for f in frames) >= 2


class Listener:

    def __init__(self, scores, vad, wake_word, stop_word=None, wake_threshold=0.5,
                 stop_threshold=0.5, end_silence=0.8, max_seconds=10.0, no_speech=4.0,
                 follow_up=8.0, preroll=0.4, cooldown=2.0, clock=time.monotonic):
        """Set up. scores: chunk -> {word: score} (WakeWords.process); vad: chunk -> bool."""
        self.scores, self.vad, self.clock = scores, vad, clock
        self.wake_word, self.stop_word = wake_word, stop_word
        self.wake_threshold, self.stop_threshold = wake_threshold, stop_threshold
        # Durations counted in whole chunks (summing 0.08 s drifts below the limit)
        self.end_silence, self.max_chunks, self.no_speech = (
            round(end_silence / SECONDS), round(max_seconds / SECONDS), round(no_speech / SECONDS))
        self.follow_up, self.cooldown = follow_up, cooldown
        self.preroll = deque(maxlen=max(1, int(preroll / SECONDS)))
        self.state = 'listening'
        self.muted = False
        self.follow_up_until = 0.0
        self.quiet_until = 0.0        # no new wake/stop trigger before this
        self.voiced_run = 0
        self._recording = None

    def speaking(self, on):
        """Tell the listener the robot started (True) or stopped (False) talking."""
        self.muted = on
        if on:
            self.voiced_run = 0
        else:
            self.follow_up_until = self.clock() + self.follow_up

    def feed(self, chunk):
        chunk = np.asarray(chunk, dtype=np.int16)
        now = self.clock()
        scores = self.scores(chunk)
        events = []
        if (self.stop_word and scores.get(self.stop_word, 0.0) >= self.stop_threshold
                and now >= self.quiet_until):
            self.quiet_until = now + self.cooldown
            self.state, self._recording = 'listening', None
            return [('stop',)]
        if self.muted:
            return events
        if self.state == 'listening':
            voiced = self.vad(chunk)
            if scores.get(self.wake_word, 0.0) >= self.wake_threshold and now >= self.quiet_until:
                self.quiet_until = now + self.cooldown
                self._start(follow_up=False, preroll=[])
                events.append(('wake',))
            elif now < self.follow_up_until:
                self.voiced_run = self.voiced_run + 1 if voiced else 0
                if self.voiced_run >= 2:  # 160 ms of speech
                    self._start(follow_up=True, preroll=list(self.preroll) + [chunk])
                    self._recording['spoke'] = True
                    events.append(('wake',))
                    return events
            self.preroll.append(chunk)
            return events
        return self._record(chunk)

    def _start(self, follow_up, preroll):
        self.state = 'recording'
        self.voiced_run = 0
        self.follow_up_until = 0.0
        self._recording = {'chunks': list(preroll), 'follow_up': follow_up, 'spoke': False,
                           'silence': 0, 'time': 0}

    def _record(self, chunk):
        r = self._recording
        r['chunks'].append(chunk)
        r['time'] += 1
        if self.vad(chunk):
            r['spoke'], r['silence'] = True, 0
        else:
            r['silence'] += 1
        done = (r['spoke'] and r['silence'] >= self.end_silence) or r['time'] >= self.max_chunks
        if not r['spoke'] and r['time'] >= self.no_speech:
            self.state, self._recording = 'listening', None
            return [('nothing',)]
        if done:
            self.state, self._recording = 'listening', None
            return [('utterance', np.concatenate(r['chunks']), r['follow_up'])]
        return []
