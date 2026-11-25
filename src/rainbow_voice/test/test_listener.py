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
    # the chunk the wake word fired on, speech, 0.8 s of silence
    assert len(samples) == CHUNK * (1 + 10 + 10)
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
    assert kinds(s.feed(LOUD, 5)) == []                  # 400 ms: not enough
    assert kinds(s.feed(LOUD, 1)) == ['wake']            # 480 ms of speech starts it
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
    assert len(events[0][1]) == CHUNK * (1 + 3 + 10)   # the wake chunk, then after the beep


def test_follow_ups_stop_after_two_and_endless_talk_is_dropped():
    s = Script(follow_up=8.0, max_seconds=2.0)
    s.feed(QUIET, wake=0.9)
    s.feed(LOUD, 3)
    s.feed(QUIET, 12)                                    # the question
    for n in range(2):                                   # two follow-ups are fine
        s.listener.speaking(True)
        s.listener.speaking(False)
        events = s.feed(LOUD, 6) + s.feed(QUIET, 12)
        assert kinds(events) == ['wake', 'utterance'], n
    s.listener.speaking(True)
    s.listener.speaking(False)
    assert kinds(s.feed(LOUD, 10)) == []                 # a third: wake word needed
    s.t += 3
    s.feed(QUIET, wake=0.9)                              # the wake word resets it
    s.feed(LOUD, 3)
    s.feed(QUIET, 12)
    s.listener.speaking(True)
    s.listener.speaking(False)
    events = s.feed(LOUD, 40)                            # talk that never stops
    assert kinds(events) == ['wake', 'nothing']


def test_wake_word_during_a_follow_up_restarts_as_a_wake():
    s = Script(follow_up=8.0)
    s.listener.speaking(True)
    s.listener.speaking(False)
    assert kinds(s.feed(LOUD, 6)) == ['wake']            # a follow-up starts ...
    assert s.feed(LOUD, wake=0.9) == [('wake', False)]   # ... "hey Jarvis": a real wake
    s.feed(LOUD, 4)
    events = s.feed(QUIET, 12)
    assert kinds(events) == ['utterance'] and events[0][2] is False
    assert len(events[0][1]) == CHUNK * (1 + 4 + 10)     # only from the wake on


def test_wake_tail_or_hum_does_not_end_the_recording_before_the_command():
    s = Script(end_silence=0.8, min_speech=0.24, no_speech=4.0)
    s.feed(QUIET, wake=0.9)
    s.feed(LOUD, 2)                                      # the hum's tail: 160 ms
    assert kinds(s.feed(QUIET, 15)) == []                # 1.2 s pause: still recording
    s.feed(LOUD, 6)                                      # "look around"
    assert kinds(s.feed(QUIET, 12)) == ['utterance']


def test_no_follow_up_while_moving():
    s = Script(follow_up=8.0, motion_settle=0.5)
    s.listener.speaking(True)
    s.listener.speaking(False)
    for _ in range(20):                                  # turning: motor noise
        s.listener.moved()
        assert kinds(s.feed(LOUD)) == []
    s.feed(QUIET, 7)                                     # stopped and settled
    assert kinds(s.feed(LOUD, 6)) == ['wake']            # the window still works after
    assert kinds(s.feed(QUIET, 12)) == ['utterance']
    s.listener.speaking(True)
    s.listener.speaking(False)
    s.listener.moved()
    assert s.feed(QUIET, wake=0.9)[-1] == ('wake', False)  # the wake word always works


def test_several_wake_words_each_with_its_threshold():
    scores = {}
    listener = Listener(lambda c: scores, lambda c: bool(c.any()),
                        {'rainbow': 0.3, 'robot': 0.6}, clock=lambda: 0.0)
    scores.update(rainbow=0.2, robot=0.5)
    assert kinds(listener.feed(QUIET)) == []
    scores.update(rainbow=0.0, robot=0.7)
    assert kinds(listener.feed(QUIET)) == ['wake']
    listener = Listener(lambda c: scores, lambda c: bool(c.any()),
                        {'rainbow': 0.3, 'robot': 0.6}, clock=lambda: 0.0)
    scores.update(rainbow=0.35, robot=0.0)
    assert kinds(listener.feed(QUIET)) == ['wake']


def test_stop_word_right_after_the_wake_word():
    """Check "Rainbow, stop!": the wake word's cooldown must not silence the stop word."""
    s = Script()
    assert kinds(s.feed(LOUD, wake=0.9)) == ['wake']
    s.feed(LOUD, 3)
    assert kinds(s.feed(LOUD, stop=0.9)) == ['stop']
    assert kinds(s.feed(LOUD, stop=0.9)) == []          # but a stop has its own cooldown


def test_a_question_is_always_listened_for():
    s = Script(follow_up=8.0, max_seconds=2.0)
    s.feed(QUIET, wake=0.9)
    s.feed(LOUD, 3)
    s.feed(QUIET, 12)
    for _ in range(2):                                   # use up the follow-ups
        s.listener.speaking(True)
        s.listener.speaking(False)
        s.feed(LOUD, 6)
        s.feed(QUIET, 12)
    # The robot asks something on its own: "What is your name?"
    s.listener.speaking(True)
    s.listener.speaking(False)
    s.listener.asked()
    events = s.feed(LOUD, 6) + s.feed(QUIET, 12)
    assert kinds(events) == ['wake', 'utterance']
    assert events[1][2] is True                          # a follow-up: the answer


def test_question_before_the_end_of_speech_too():
    s = Script(follow_up=8.0, max_seconds=2.0)
    s.listener.speaking(True)
    s.listener.asked()                                   # arrives while still muted
    s.listener.speaking(False)
    assert kinds(s.feed(LOUD, 6) + s.feed(QUIET, 12)) == ['wake', 'utterance']
