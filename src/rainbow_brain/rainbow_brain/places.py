"""Named places on a map, kept in a YAML file (no ROS in this module).

The file is <data_dir>/places/<map>.yaml; a place's coordinates only make sense
on its own map. Format (see docs/brain.md):

  map: home_live
  places:
    kitchen:
      pose: {x: 3.2, y: -1.1, yaw: 1.57}
      room: kitchen
      description: in front of the fridge, facing the window
      source: owner          # owner | foxglove | robot
      confirmed: true

A place named "home" is where "go home" goes. Every change is written at once,
atomically (a crash cannot leave half a file).
"""

from dataclasses import asdict, dataclass
import math
import os
import re
import time

import yaml

SOURCES = ('owner', 'foxglove', 'robot')
MAX_NAME = 40


@dataclass
class Place:
    name: str
    x: float
    y: float
    yaw: float
    room: str = ''
    description: str = ''
    source: str = 'owner'
    confirmed: bool = True


def normalize_name(name):
    """Lower case, single spaces, no leading "the"; raises ValueError if unusable."""
    name = ' '.join(str(name).lower().split())
    name = re.sub(r'^the ', '', name)
    if not name:
        raise ValueError('a place needs a name')
    if len(name) > MAX_NAME:
        raise ValueError(f'place names are at most {MAX_NAME} characters')
    if not re.fullmatch(r"[a-z0-9][a-z0-9 '\-]*", name):
        raise ValueError(f'"{name}" is not a usable place name (letters, digits, spaces)')
    return name


def angle_diff(a, b):
    """Smallest signed difference a - b, in (-pi, pi]."""
    d = (a - b) % (2 * math.pi)
    return d - 2 * math.pi if d > math.pi else d


class PlaceStore:

    def __init__(self, data_dir, map_name, log=print):
        self.map_name = map_name
        self.path = os.path.join(data_dir, 'places', f'{map_name}.yaml')
        self.log = log
        self.places = {}
        self.load()

    # File

    def load(self):
        self.places = {}
        if not os.path.exists(self.path):
            return
        try:
            with open(self.path) as f:
                data = yaml.safe_load(f) or {}
            for name, p in (data.get('places') or {}).items():
                pose = p.get('pose') or {}
                place = Place(
                    name=normalize_name(name),
                    x=float(pose['x']), y=float(pose['y']), yaw=float(pose.get('yaw', 0.0)),
                    room=str(p.get('room') or ''),
                    description=str(p.get('description') or ''),
                    source=p.get('source') if p.get('source') in SOURCES else 'owner',
                    confirmed=bool(p.get('confirmed', True)),
                )
                self.places[place.name] = place
        except (OSError, yaml.YAMLError, KeyError, TypeError, ValueError, AttributeError) as e:
            # Keep the broken file for the owner to look at; start empty.
            bad = f'{self.path}.bad-{int(time.time())}'
            os.replace(self.path, bad)
            self.places = {}
            self.log(f'places file unreadable ({e}); moved it to {bad}')

    def save_file(self):
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        places = {}
        for p in sorted(self.places.values(), key=lambda p: p.name):
            d = asdict(p)
            d.pop('name')
            places[p.name] = {
                'pose': {k: round(d.pop(k), 3) for k in ('x', 'y', 'yaw')},
                **d,
            }
        tmp = self.path + '.tmp'
        with open(tmp, 'w') as f:
            yaml.safe_dump({'map': self.map_name, 'places': places}, f,
                           sort_keys=False, allow_unicode=True)
        os.replace(tmp, self.path)

    # Changes (each raises ValueError with a speakable reason)

    def save(self, name, x, y, yaw, room='', description='', source='owner'):
        if source not in SOURCES:
            raise ValueError(f'unknown source {source}')
        place = Place(normalize_name(name), float(x), float(y), float(yaw),
                      room.strip(), description.strip(), source,
                      confirmed=(source != 'robot'))
        self.places[place.name] = place
        self.save_file()
        return place

    def edit(self, name, new_name='', room='', description='', confirm=False):
        place = self.get(name)
        if new_name:
            new_name = normalize_name(new_name)
            if new_name != place.name and new_name in self.places:
                raise ValueError(f'there is already a place called {new_name}')
            del self.places[place.name]
            place.name = new_name
            self.places[new_name] = place
        if room:
            place.room = room.strip()
        if description:
            place.description = description.strip()
        if confirm:
            place.confirmed = True
        self.save_file()
        return place

    def delete(self, name):
        place = self.get(name)
        del self.places[place.name]
        self.save_file()
        return place

    # Queries

    def get(self, name):
        key = normalize_name(name)
        if key not in self.places:
            raise ValueError(f"I don't know a place called {key}")
        return self.places[key]

    def sorted_places(self):
        return sorted(self.places.values(), key=lambda p: p.name)

    def nearest(self, x, y, max_distance):
        """(place, distance) of the nearest place within max_distance, or (None, inf)."""
        best, best_d = None, math.inf
        for p in self.places.values():
            d = math.hypot(p.x - x, p.y - y)
            if d < best_d and d <= max_distance:
                best, best_d = p, d
        return best, best_d

    def facing(self, x, y, yaw, max_angle=math.radians(30), min_distance=0.5):
        """The nearest place roughly ahead (within max_angle of the heading), or None."""
        best, best_d = None, math.inf
        for p in self.places.values():
            d = math.hypot(p.x - x, p.y - y)
            if d < min_distance:
                continue
            bearing = math.atan2(p.y - y, p.x - x)
            if abs(angle_diff(bearing, yaw)) <= max_angle and d < best_d:
                best, best_d = p, d
        return best

    def describe(self, x, y, yaw, near_distance, certainty='sure'):
        """A speakable sentence about where (x, y, yaw) is. Returns (sentence, near, facing)."""
        near, d = self.nearest(x, y, near_distance)
        ahead = self.facing(x, y, yaw)
        if ahead is not None and near is not None and ahead.name == near.name:
            ahead = None
        parts = []
        if near is not None:
            where = f'in the {near.room}, ' if near.room else ''
            place = near.name if near.confirmed else f'what I think is the {near.name}'
            parts.append(f"I'm {where}near the {place}" if near.confirmed
                         else f"I'm {where}near {place}")
        else:
            parts.append("I'm not near any place I know")
        if ahead is not None:
            parts.append(f'facing the {ahead.name}')
        sentence = ', '.join(parts) + '.'
        # Every sentence starts with "I'm", which stays capitalised
        if certainty == 'roughly':
            sentence = 'I think ' + sentence
        elif certainty == 'lost':
            sentence = "I'm not sure where I am yet. My best guess: " + sentence
        return sentence, near, ahead
