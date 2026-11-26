"""Local voice commands: the phrases Vosk listens for, and what they mean.

The brain publishes the phrases on /voice/grammar (they depend on the places);
the voice node recognises them and the brain acts on a confident match
without the Mac. Everything else goes to the agent.
"""

STOP = ('stop', 'wait')
HOME = ('come back', 'go home', 'come home')
LOCALIZE = ('look around', 'find yourself', 'localize')
# The head: handled here so they work without the Mac agent
STOP_WATCHING = ('stop watching', 'stop following', 'stop watching me', 'stop following me')
GREET_ON = ('watch for people', 'greet people')
WATCH_ME = ('watch me', 'follow me with your eyes')


def phrases(place_names):
    """Return the command phrases for these places."""
    out = (list(STOP) + list(HOME) + list(LOCALIZE) + list(STOP_WATCHING) + list(GREET_ON)
           + list(WATCH_ME))
    for name in place_names:
        out += [f'go to the {name}', f'go to {name}']
    return out


def parse(text, place_names):
    """Return the command for a recognised phrase, or None.

    {'action': 'stop'}, {'action': 'localize'}, {'action': 'go_to', 'place':
    name} (going home goes to the place named home), {'action': 'stop_watching'}
    (ends any head tracking and switches greeting people off), {'action':
    'greet_on'} or {'action': 'watch_me'}.
    """
    text = ' '.join(text.lower().split())
    if text in STOP:
        return {'action': 'stop'}
    if text in STOP_WATCHING:
        return {'action': 'stop_watching'}
    if text in GREET_ON:
        return {'action': 'greet_on'}
    if text in WATCH_ME:
        return {'action': 'watch_me'}
    if text in LOCALIZE:
        return {'action': 'localize'}
    if text in HOME:
        return {'action': 'go_to', 'place': 'home'}
    for prefix in ('go to the ', 'go to '):
        if text.startswith(prefix) and text[len(prefix):] in place_names:
            return {'action': 'go_to', 'place': text[len(prefix):]}
    return None
