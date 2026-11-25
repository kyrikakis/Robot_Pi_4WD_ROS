"""Deterministic guards around the model's tool use (the prompt alone is not enough).

- wants_action: the person clearly asked for an action. If the model replies
  without calling a tool, the conversation retries once with a note.
- ground_args: a place's description or room is kept only if the person said
  it; the model otherwise makes them up ("a cozy spot with a soft chair"). A
  description that only repeats the name is dropped too.
- MOVES: at most one move per turn ("drive five meters" became six 0.5 m drives).
- wants_photo: asked what it sees, the model answers only after capture_photo
  (after "look down" it described a red toy car without a photo).
- check_remember_face: a face is saved only under a name the person said, after
  the robot said it back and the person said yes (the model saved names at once
  and invented "Alex" when none was said).
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
  | \blook\s+(left|right|up|down|straight)\b
  | \b(watch|track)\s+(me|the|him|her|it|them)\b | \bkeep\s+(looking|an\s+eye)\b
  | \bfollow\s+(me|the|him|her|it|them)\s+with\s+your\s+(eyes|head)\b
  | \bstop\s+(watching|following|tracking)\b
  | \bremember\s+my\s+face\b
  | \b(which|what)\s+places\b | \bwhere\s+are\s+you\b | \byour\s+battery\b  # ask the tools
  | \b(stop|don'?t|no\s+more)\s+greet | \bgreet\s+(people|everyone|us)\b | \bleave\s+me\s+alone\b
  | \bforget\s+(my|me|his|her|their)\b.*\bface\b | \bforget\s+\w+'s\s+face\b
""", re.IGNORECASE | re.VERBOSE)

_FILLER = {'the', 'and', 'with', 'near', 'next', 'from', 'that', 'this', 'there', 'its',
           "it's", 'for', 'are', 'is', 'room'}


_PHOTO = re.compile(r"""
    \b(what|who)\s+(do|can)\s+you\s+see\b | \bcan\s+you\s+see\b
  | \b(tell|show)\s+me\s+what\b | \bwhat('?s|\s+is)\s+(there|in\s+front)\b
  | \bwhat\s+you\s+see\b | \bdescribe\s+what\b
""", re.IGNORECASE | re.VERBOSE)


def wants_photo(text):
    """The person asked what the robot sees: answering needs a photo."""
    return bool(_PHOTO.search(text or ''))


def wants_action(text, last_reply=''):
    """The person asked for an action, or answered the robot's question about a name:
    a yes to "Stefanos, did I get that right?" is the go-ahead to save the face, and
    an answer to "What is your name?" starts saving it (the guard then has the
    name said back first)."""
    if _ACTION.search(text or ''):
        return True
    reply = (last_reply or '').rstrip()
    if not reply.endswith('?'):
        return False
    if _ASKED_NAME.search(reply) and _GIVES_NAME.search(text or ''):
        return True
    return bool(_YES.search(text or '')) and bool(_CHECKING.search(reply))


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


# The robot asking for a name, and someone giving one
_ASKED_NAME = re.compile(r'\byour\s+name\b', re.IGNORECASE)
_GIVES_NAME = re.compile(r"\b(i'?m|i\s+am|my\s+name'?s?|name\s+is|call\s+me|it'?s)\b",
                         re.IGNORECASE)
# The robot checking something it heard: "Stefanos, did I get that right?"
_CHECKING = re.compile(r'\b(right|correct|get\s+that|is\s+that|spell)\b', re.IGNORECASE)
_YES = re.compile(r"^\W*(yes|yeah|yep|yup|correct|right|exactly|that'?s\s+(it|right|correct))\b",
                  re.IGNORECASE)


def _heard_names(said):
    """The words said, plus spelled-out names joined up ("S T E F A N O S")."""
    words = _words(said)
    out = set(words)
    run = ''
    for w in words + ['']:
        if len(w) == 1 and w.isalpha():
            run += w
            continue
        if len(run) > 1:
            out.add(run)
        run = ''
    return out


def check_remember_face(name, said, last_reply, last_said, confirmed=()):
    """None if remember_face(name) may go ahead, else what the model should do instead.

    said: the person's recent words; last_reply: the robot's previous reply;
    last_said: the person's latest words; confirmed: names the person already
    said yes to in this conversation.
    """
    name_words = _words(name)
    if name_words and ' '.join(name_words) in {' '.join(_words(c)) for c in confirmed}:
        return None
    if not name_words or not all(w in _heard_names(said) for w in name_words):
        return ('not saved: the person did not say that name. Ask them their name; '
                'never guess one')
    if not all(w in _words(last_reply) for w in name_words) or not _YES.search(last_said or ''):
        return (f'not saved yet: say the name back ("{name}, did I get that right?") and '
                'call remember_face only after they say yes')
    return None
