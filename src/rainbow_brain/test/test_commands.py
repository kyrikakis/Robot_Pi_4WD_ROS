"""Tests for the brain's local voice commands: python3 -m pytest src/rainbow_brain/test."""

from rainbow_brain.commands import parse, phrases

PLACES = ['kitchen', 'home']


def test_head_commands_work_without_the_agent():
    assert parse('stop watching', PLACES) == {'action': 'stop_watching'}
    assert parse('Stop  following', PLACES) == {'action': 'stop_watching'}
    assert parse('watch for people', PLACES) == {'action': 'greet_on'}
    assert parse('greet people', PLACES) == {'action': 'greet_on'}
    assert parse('watch me', PLACES) == {'action': 'watch_me'}


def test_plain_stop_is_still_stop():
    assert parse('stop', PLACES) == {'action': 'stop'}
    assert parse('go to the kitchen', PLACES) == {'action': 'go_to', 'place': 'kitchen'}


def test_the_recogniser_hears_them():
    said = phrases(PLACES)
    for p in ('stop watching', 'watch for people', 'greet people', 'watch me'):
        assert p in said
