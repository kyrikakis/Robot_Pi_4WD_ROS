"""The robot's skills as tools for the model (OpenAI function-calling format).

One to one with rainbow_brain's skills (docs/brain.md), worded for the model:
angles in degrees here, converted to radians when sent to the brain.
"""


def _tool(name, description, properties=None, required=()):
    return {
        'type': 'function',
        'function': {
            'name': name,
            'description': description,
            'parameters': {
                'type': 'object',
                'properties': properties or {},
                'required': list(required),
            },
        },
    }


_TEXT = {'type': 'string'}

TOOLS = [
    _tool('go_to_place',
          'Drive to a named place on the map (use list_places for the names; "home" is '
          'the place named home). Returns at once with "started"; arriving or failing '
          'comes later as a robot event. A new move replaces the one running.',
          {'place': {'type': 'string', 'description': 'name of a known place'}},
          ['place']),
    _tool('turn',
          'Turn in place. Positive turns left, negative right.',
          {'degrees': {'type': 'number', 'description': '-360 to 360'}},
          ['degrees']),
    _tool('drive_forward',
          'Drive straight ahead a short distance. The robot cannot drive backwards.',
          {'meters': {'type': 'number', 'description': '0.05 to 1.0'}},
          ['meters']),
    _tool('stop', 'Stop moving at once and cancel any move.'),
    _tool('where_am_i',
          'Where the robot is, in words: room, nearest known place, what it faces, and '
          'how sure it is.'),
    _tool('get_state',
          "The robot's state: what it is doing, battery voltage, whether it knows where "
          'it is.'),
    _tool('list_places', 'The named places on the map, with rooms and descriptions.'),
    _tool('save_place',
          "Remember the robot's current position as a named place. Use guess=true when "
          'it is your own guess (for example from what a photo shows), not something '
          'the user told you.',
          {'name': _TEXT, 'description': _TEXT, 'room': _TEXT, 'guess': {'type': 'boolean'}},
          ['name']),
    _tool('edit_place',
          'Rename a place, change its description or room, or confirm one of your '
          'guesses (confirm=true) after the user agreed.',
          {'name': _TEXT, 'new_name': _TEXT, 'description': _TEXT, 'room': _TEXT,
           'confirm': {'type': 'boolean'}},
          ['name']),
    _tool('delete_place', 'Forget a named place.', {'name': _TEXT}, ['name']),
    _tool('look',
          'Point your head (and camera). Angles from straight ahead: pan positive left, '
          'negative right; tilt positive up, negative down. 0, 0 looks straight ahead. '
          'The head turns about 40 degrees each way and tilts about 36 up and 19 down; '
          'bigger values stop at the limit. Moving the head shows you nothing: only '
          'capture_photo does, so never say what is there without one. Returns once '
          'the head is still, so a capture_photo right after sees where it points.',
          {'pan_degrees': {'type': 'number', 'description': '-40 to 38'},
           'tilt_degrees': {'type': 'number', 'description': '-19 to 36'}}),
    _tool('track',
          'Keep your head turned towards something while it moves (only the head; '
          'you do not drive after it). what: "face" for whoever is in front of you, a '
          "person's name (if you know their face), or a thing like \"ball\". Keeps "
          'going until stop_tracking, stop or look; if it goes out of view you get a '
          'robot event. Starting it shows you nothing: capture_photo does.',
          {'what': {'type': 'string', 'description': 'face, a name, or a thing'}},
          ['what']),
    _tool('stop_tracking', 'Stop following something with your head.'),
    _tool('capture_photo',
          'Take a photo with your camera and look at it, when someone asks what you see '
          'or you need to look. You get the photo right after.'),
    _tool('remember_face',
          'Remember the face of the person in front of the camera under their name. Only '
          'when they ask you to, and only one person may be in view.',
          {'name': _TEXT}, ['name']),
]

NAMES = [t['function']['name'] for t in TOOLS]
