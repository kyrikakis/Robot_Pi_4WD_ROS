"""Local voice commands: the phrases Vosk listens for, and what they mean.

The brain publishes the phrases on /voice/grammar (they depend on the places);
the voice node recognises them and the brain acts on a confident match
without the Mac. Everything else goes to the agent.
"""

STOP = ('stop', 'wait')
HOME = ('come back', 'go home', 'come home')


def phrases(place_names):
    """Return the command phrases for these places."""
    out = list(STOP) + list(HOME)
    for name in place_names:
        out += [f'go to the {name}', f'go to {name}']
    return out


def parse(text, place_names):
    """Return the command for a recognised phrase, or None.

    {'action': 'stop'} or {'action': 'go_to', 'place': name}; going home goes
    to the place named home.
    """
    text = ' '.join(text.lower().split())
    if text in STOP:
        return {'action': 'stop'}
    if text in HOME:
        return {'action': 'go_to', 'place': 'home'}
    for prefix in ('go to the ', 'go to '):
        if text.startswith(prefix) and text[len(prefix):] in place_names:
            return {'action': 'go_to', 'place': text[len(prefix):]}
    return None
