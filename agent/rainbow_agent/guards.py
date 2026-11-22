"""Deterministic guards around the model's tool use (the prompt alone is not enough).

- wants_action: the person clearly asked for an action. If the model replies
  without calling a tool, the conversation retries once with a note.
- ground_args: a place's description or room is kept only if the person said
  it; the model otherwise makes them up ("a cozy spot with a soft chair"). A
  description that only repeats the name is dropped too.
- MOVES: at most one move per turn ("drive five meters" became six 0.5 m drives).
"""

import re

MOVES = {'go_to_place', 'turn', 'drive_forward'}

_ACTION = re.compile(r"""
    ^\W*(stop|halt|freeze|wait)\b                       # "stop!", "wait"
  | \bstop\W*$
  | \b(go|drive|come|head)\s+(to|home|back|over)\b      # go to the kitchen, go home
  | \bturn\s+(left|right|around|to)\b
  | \b(move|drive|go)\s+forward\b
  | \bremember\s+(this|that|it|here)\b                  # remember this spot as X
  | \bthis\s+is\s+(the|my|our)\b.*\bremember\b          # this is the bathroom, remember it
  | \b(save|call|name)\s+(this|that|it|here)\b
  | \bcall\s+the\s+\w+(\s+\w+)?\s+the\b                 # call the sofa the couch
  | \bforget\s+(the|about\s+the)\b
  | ^\W*yes\b.*\b(right|correct)\b                     # yes, the desk is right
  | \b(what|who)\s+(do|can)\s+you\s+see\b              # seeing needs a photo
  | \bcan\s+you\s+see\b | \blook\s+(at|around)\b | \bin\s+front\s+of\s+you\b
  | \bremember\s+my\s+face\b
""", re.IGNORECASE | re.VERBOSE)

_FILLER = {'the', 'and', 'with', 'near', 'next', 'from', 'that', 'this', 'there', 'its',
           "it's", 'for', 'are', 'is', 'room'}


def wants_action(text):
    return bool(_ACTION.search(text or ''))


def _words(text):
    return re.findall(r"[a-z0-9']+", str(text).lower())


def grounded(value, said):
    """True if every meaningful word of value appears in what the person said."""
    words = [w for w in _words(value) if len(w) > 2 and w not in _FILLER]
    heard = set(_words(said))
    return all(w in heard or w.rstrip('s') in heard or w + 's' in heard for w in words)


def ground_args(name, args, said):
    """Drop invented description/room from place calls. Returns (args, dropped keys)."""
    if name not in ('save_place', 'edit_place'):
        return args, []
    args = dict(args)
    name_words = set(_words(args.get('name', '')))
    dropped = [k for k in ('description', 'room') if args.get(k) and (
        not grounded(args[k], said)
        or (k == 'description' and set(_words(args[k])) <= name_words))]
    for k in dropped:
        del args[k]
    return args, dropped
