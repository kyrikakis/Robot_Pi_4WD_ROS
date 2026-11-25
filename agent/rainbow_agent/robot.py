"""The robot as the agent sees it: skill calls in, events out.

SimRobot simulates the brain for developing and testing the agent without the
robot: an in-memory map of places, moves that take time, and a place it can
never reach (to exercise failures). LinkRobot (the WebSocket link to the Pi's
rainbow_brain) replaces it in build step 2, with the same interface.

Every call returns a dict, sent to the model as JSON: {'ok': bool, ...}.
After a turn or a short drive the result also says where the robot now is
('now': where_am_i's sentence), so the model reports facts instead of making
them up ("now I face the kitchen"). LinkRobot must do the same.
Events are dicts on robot.events: {'kind': 'arrived' | 'failed' | ..., ...}.
"""

import asyncio
import math

# In look's result: the head moving shows the model nothing (it made up what was there)
LOOK_SEEN = 'nothing yet: call capture_photo to see what is there'
# The same for track and stop_tracking (it made up where things were)
TRACK_MESSAGE = 'I will turn my head after the {} whenever I can see it'
TRACK_SEEN = ('not known: you do not know if it is in view now; you are told if it is '
              'lost; call capture_photo to see')
BATTERY_EMPTY = 'my battery is empty, so I am not driving; please charge me'
STOP_TRACKING = 'stopped following; my head is back in the middle'


def _norm(name):
    name = ' '.join(str(name).lower().split())
    return name[4:] if name.startswith('the ') else name


DEFAULT_PLACES = {
    'home': {'x': 0.0, 'y': 0.0, 'room': 'living room',
             'description': 'next to the TV stand, where Rainbow rests', 'confirmed': True},
    'sofa': {'x': 1.5, 'y': 1.0, 'room': 'living room',
             'description': 'in front of the grey sofa', 'confirmed': True},
    'kitchen': {'x': 4.0, 'y': -1.0, 'room': 'kitchen',
                'description': 'in front of the fridge', 'confirmed': True},
    'hallway': {'x': 3.0, 'y': 2.5, 'room': 'hallway',
                'description': 'by the front door', 'confirmed': True},
    'kids room': {'x': 5.5, 'y': 4.0, 'room': "kids' room",
                  'description': 'next to the toy box', 'confirmed': True},
    'desk': {'x': 6.0, 'y': 1.0, 'room': 'office',
             'description': 'looks like a desk with a computer', 'confirmed': False},
    'balcony': {'x': 8.0, 'y': -2.0, 'room': 'balcony',
                'description': 'outside, behind the glass door', 'confirmed': True,
                'unreachable': True},
}


class SimRobot:

    def __init__(self, places=None, speed=0.3, time_scale=1.0, battery=7.6):
        self.places = {k: dict(v) for k, v in (places or DEFAULT_PLACES).items()}
        self.x, self.y, self.yaw = 0.0, 0.0, 0.0
        self.speed = speed              # m/s, for how long a move takes
        self.time_scale = time_scale    # 0 = instant (tests)
        self.battery = battery
        self.battery_low = False        # as the brain's low-battery mode
        self.greet = True               # greeting people (the brain's greet_people)
        self.activity = 'idle'
        self.tracking = ''
        self.faces = {}                 # name -> True; the face in view is 'in_view'
        self.face_in_view = ''          # name the face in front is saved under, '' = unknown
        self.events = asyncio.Queue()
        self.calls = []                 # (name, args) of every call, for tests and eval
        self._move = None

    async def call(self, name, args):
        self.calls.append((name, dict(args)))
        handler = getattr(self, f'_{name}', None)
        if handler is None:
            return {'ok': False, 'message': f'unknown skill {name}'}
        try:
            return await handler(**args)
        except TypeError as e:
            return {'ok': False, 'message': f'bad arguments: {e}'}

    def snapshot(self):
        """A short state summary for the model's context."""
        low = (' Battery empty: everything is paused and you cannot drive until you are '
               'charged.' if self.battery_low else '')
        return (f'Activity: {self.activity}. Battery {self.battery:.1f} V.{low} '
                f'Known places: {", ".join(sorted(self.places))}.')

    # Moves

    def _start_move(self, label, seconds, finish):
        if self._move is not None and not self._move.done():
            self._move.cancel()
        self.activity = label

        async def run():
            try:
                await asyncio.sleep(seconds * self.time_scale)
            except asyncio.CancelledError:
                return
            event = finish()
            self.activity = 'idle'
            if event:
                await self.events.put(event)

        self._move = asyncio.ensure_future(run())

    async def _go_to_place(self, place):
        if self.battery_low:
            return {'ok': False, 'message': BATTERY_EMPTY}
        key = _norm(place)
        if key not in self.places:
            return {'ok': False, 'message': f"I don't know a place called {key}"}
        p = self.places[key]
        distance = math.hypot(p['x'] - self.x, p['y'] - self.y)

        def finish():
            if p.get('unreachable'):
                return {'kind': 'failed', 'place': key,
                        'message': f'could not get to {key}: no path (blocked)'}
            self.x, self.y = p['x'], p['y']
            return {'kind': 'arrived', 'place': key}

        self._start_move(f'going to {key}', distance / self.speed, finish)
        return {'ok': True, 'message': f'started going to {key} ({distance:.1f} m)'}

    async def _turn(self, degrees):
        degrees = float(degrees)
        if not 0 < abs(degrees) <= 360:
            return {'ok': False, 'message': 'degrees must be non-zero, at most 360'}
        self.yaw = (self.yaw + math.radians(degrees)) % (2 * math.pi)
        return {'ok': True, 'message': f'turned {degrees:.0f} degrees',
                'now': (await self._where_am_i())['description']}

    async def _drive_forward(self, meters):
        meters = float(meters)
        if not 0 < meters <= 1.0:
            return {'ok': False, 'message': 'I only drive forwards, at most 1 m'}
        self.x += meters * math.cos(self.yaw)
        self.y += meters * math.sin(self.yaw)
        return {'ok': True, 'message': f'drove {meters:.2f} m',
                'now': (await self._where_am_i())['description']}

    async def _stop(self):
        if self._move is not None and not self._move.done():
            self._move.cancel()
        self.activity = 'idle'
        return {'ok': True, 'message': 'stopped'}

    # Questions

    async def _where_am_i(self):
        near = min(self.places.items(),
                   key=lambda kv: math.hypot(kv[1]['x'] - self.x, kv[1]['y'] - self.y))
        name, p = near
        d = math.hypot(p['x'] - self.x, p['y'] - self.y)
        if d > 1.5:
            sentence = "I'm not near any place I know."
        else:
            sentence = f"I'm in the {p['room']}, near the {name}."
        return {'ok': True, 'description': sentence, 'certainty': 'sure',
                'x': round(self.x, 2), 'y': round(self.y, 2)}

    async def _get_state(self):
        return {'ok': True, 'mode': 'remote', 'activity': self.activity,
                'battery_volts': self.battery, 'localized': True, 'certainty': 'sure'}

    async def _list_places(self):
        return {'ok': True, 'places': [
            {'name': k, 'room': v['room'], 'description': v['description'],
             'confirmed': v['confirmed']} for k, v in sorted(self.places.items())]}

    # Places

    async def _save_place(self, name, description='', room='', guess=False):
        key = _norm(name)
        if not key:
            return {'ok': False, 'message': 'a place needs a name'}
        self.places[key] = {'x': self.x, 'y': self.y, 'room': room,
                            'description': description, 'confirmed': not guess}
        return {'ok': True, 'message': f'saved {key}'}

    async def _edit_place(self, name, new_name='', description='', room='', confirm=False):
        key = _norm(name)
        if key not in self.places:
            return {'ok': False, 'message': f"I don't know a place called {key}"}
        p = self.places.pop(key)
        if description:
            p['description'] = description
        if room:
            p['room'] = room
        if confirm:
            p['confirmed'] = True
        new_key = _norm(new_name) if new_name else key
        self.places[new_key] = p
        return {'ok': True, 'message': f'updated {new_key}'}

    async def _look(self, pan_degrees=0.0, tilt_degrees=0.0):
        pan = max(-40.0, min(38.0, float(pan_degrees)))
        tilt = max(-19.0, min(36.0, float(tilt_degrees)))
        return {'ok': True, 'message': f'looking {pan:.0f} deg left, {tilt:.0f} deg up',
                'pan_degrees': round(pan), 'tilt_degrees': round(tilt), 'seen': LOOK_SEEN}

    async def _track(self, what):
        what = str(what).strip().lower()
        if not what:
            return {'ok': False, 'message': 'tell me what to follow'}
        self.tracking = what
        return {'ok': True, 'message': TRACK_MESSAGE.format(what), 'seen': TRACK_SEEN}

    async def _stop_tracking(self):
        self.tracking = ''
        return {'ok': True, 'message': STOP_TRACKING}

    async def _capture_photo(self):
        return {'ok': False, 'message': 'my camera is off (simulated robot)'}

    async def _remember_face(self, name, same_person=False):
        # As the brain answers (rainbow_brain.faces): face_in_view is the name the face in
        # front matches, '' for a face it does not know
        name = str(name).strip()
        same = same_person in (True, 'true', 'True', 1)
        if self.face_in_view and self.face_in_view != name:
            return {'ok': False, 'message': f'I already know this face as {self.face_in_view}'}
        if self.face_in_view == name:
            if same:
                self.faces[name] = self.faces.get(name, 1) + 1
                return {'ok': True, 'message': f'I added this view of {name}'}
            return {'ok': True, 'message': f'I already know you, {name}'}
        if name in self.faces:
            if same:
                self.faces[name] = self.faces.get(name, 1) + 1
                self.face_in_view = name
                return {'ok': True, 'message': f'I added this view of {name}'}
            return {'ok': False, 'message': f'I know a face called {name} that looks different',
                    'next': (f'ask if they are the {name} you know (another angle or other '
                             f'light); if yes, call remember_face again with same_person true; '
                             f'if not, ask for another name, like with their last name')}
        self.faces[name] = 1
        self.face_in_view = name
        return {'ok': True,
                'message': (f'I will remember {name}. Tell them to turn their head slowly to '
                            f'one side and the other, so I learn their face from more angles'),
                'next': 'the extra views are added by themselves: call nothing'}

    async def _greet_people(self, enabled):
        self.greet = enabled in (True, 'true', 'True', 1)
        return {'ok': True, 'message': 'I will greet people I see' if self.greet
                else 'I will not turn to people or greet them until you say so'}

    async def _forget_face(self, name):
        name = str(name).strip()
        match = next((n for n in self.faces if n.lower() == name.lower()), None)
        if match is None and self.face_in_view.lower() != name.lower():
            return {'ok': False, 'message': f'I do not know a face called {name}'}
        self.faces.pop(match, None)
        if self.face_in_view.lower() == name.lower():
            self.face_in_view = ''
        return {'ok': True, 'message': f'I forgot the face of {match or name}'}

    async def _delete_place(self, name):
        key = _norm(name)
        if self.places.pop(key, None) is None:
            return {'ok': False, 'message': f"I don't know a place called {key}"}
        return {'ok': True, 'message': f'deleted {key}'}
