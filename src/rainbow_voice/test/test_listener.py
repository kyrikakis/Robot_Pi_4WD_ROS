"""Tests for the listening state machine, with scripted wake scores and VAD."""

import numpy as np
from rainbow_voice.listener import CHUNK, Listener, SECONDS

QUIET = np.zeros(CHUNK, dtype=np.int16)
LOUD = np.full(CHUNK, 1000, dtype=np.int16)


class Script:
    """Feeds a listener and fakes time: one chunk = 80 ms."""

    def __init__(self, **kwargs):
        self.t = 0.0
        self.wake = 0.0
        self.stop = 0.0
        self.listener = Listener(self.scores, lambda c: bool(c.any()), 'wake',
                                 kwargs.pop('stop_word', 'stop'), clock=lambda: self.t,
                                 **kwargs)

    def scores(self, chunk):
        return {'wake': self.wake, 'stop': self.stop}

    def feed(self, chunk, n=1, wake=0.0, stop=0.0):
        events = []
        for _ in range(n):
            self.wake, self.stop = wake, stop
            events += self.listener.feed(chunk)
            self.t += SECONDS
        return events


def kinds(events):
    return [e[0] for e in events]


def test_wake_record_until_silence():
    s = Script(end_silence=0.8)
    assert kinds(s.feed(QUIET, 5)) == []
    assert kinds(s.feed(QUIET, wake=0.9)) == ['wake']
    assert kinds(s.feed(LOUD, 10)) == []
    events = s.feed(QUIET, 12)
    assert kinds(events) == ['utterance']
    samples, follow_up = events[0][1], events[0][2]
    assert follow_up is False
    assert len(samples) == CHUNK * (10 + 10)  # speech plus 0.8 s of silence
    assert s.listener.state == 'listening'


def test_woke_but_nobody_spoke():
    s = Script(no_speech=1.0)
    s.feed(QUIET, wake=0.9)
    assert kinds(s.feed(QUIET, 13)) == ['nothing']


def test_max_length():
    s = Script(max_seconds=2.0)
    s.feed(QUIET, wake=0.9)
    assert kinds(s.feed(LOUD, 30)) == ['utterance']


def test_stop_word_any_time_even_while_speaking():
    s = Script()
    s.listener.speaking(True)
    assert kinds(s.feed(QUIET, wake=0.9)) == []          # muted: no wake word
    assert kinds(s.feed(QUIET, stop=0.9)) == ['stop']
    assert kinds(s.feed(QUIET, stop=0.9)) == []          # cooldown
    s.t += 3
    s.listener.speaking(False)
    s.feed(QUIET, wake=0.9)
    s.t += 3
    assert kinds(s.feed(LOUD, stop=0.9)) == ['stop']     # stops a recording too
    assert s.listener.state == 'listening'


def test_follow_up_without_wake_word():
    s = Script(follow_up=8.0, preroll=0.4)
    s.listener.speaking(True)
    s.feed(LOUD, 5)                                      # the robot's own voice: ignored
    s.listener.speaking(False)
    assert kinds(s.feed(QUIET, 3)) == []
    assert kinds(s.feed(LOUD, 2)) == ['wake']            # 160 ms of speech starts it
    events = s.feed(LOUD, 5) + s.feed(QUIET, 12)
    assert kinds(events) == ['utterance']
    assert events[0][2] is True
    # After the window: speech alone does nothing
    s.t += 20
    assert kinds(s.feed(LOUD, 5)) == []


def test_no_stop_word_configured():
    s = Script(stop_word=None)
    assert kinds(s.feed(QUIET, stop=0.9)) == []


def test_no_follow_up_window_without_speech_first():
    s = Script(follow_up=8.0)
    s.listener.speaking(False)      # the speech node starts: "not speaking"
    assert kinds(s.feed(LOUD, 5)) == []


def test_wake_beep_is_skipped_but_follow_ups_are_not():
    s = Script(skip_after_wake=0.16)
    assert s.feed(QUIET, wake=0.9) == [('wake', False)]
    s.feed(LOUD, 2)                      # the beep: dropped, not "speech"
    s.feed(LOUD, 3)
    events = s.feed(QUIET, 12)
    assert len(events[0][1]) == CHUNK * (3 + 10)
