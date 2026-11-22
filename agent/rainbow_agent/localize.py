"""Place photos and startup localization from a photo (docs/brain.md).

Whenever a place is saved through the agent, the robot also takes a photo
there; it is kept on the Mac (<root>/data/place_photos/<map>/<place>.jpg) and
follows renames and deletes. The emergency option: when the robot turned
around ("look around", a local command on the Pi) and AMCL is still not sure,
the brain sends a 'lost' event; then the robot takes a photo and the vision
model compares it with the place photos; a confident match is sent to AMCL as
a pose estimate, which the lidar then refines. A photo only gives a hint, never
a precise pose.
"""

import asyncio
import base64
import json
from pathlib import Path
import re

MAX_REFERENCES = 6
INTRO = ('A small home robot needs to know where it is. Below is its CURRENT photo, then '
         'REFERENCE photos taken earlier at known places in the same home. Each label comes '
         'right BEFORE its photo.')
QUESTION = ('Which REFERENCE place shows the same spot as the CURRENT photo (same room, '
            'roughly the same direction)? Reply with JSON only: {"place": "<the reference '
            'place name>" or null, "confidence": <0 to 1>}. Use null if none clearly matches.')


def place_key(name):
    """Normalise a place name like the brain does (lower case, no leading "the")."""
    name = ' '.join(str(name).lower().split())
    return name[4:] if name.startswith('the ') else name


def _file_name(place):
    return re.sub(r'[^a-z0-9]+', '_', place_key(place)).strip('_') + '.jpg'


class PlacePhotos:

    def __init__(self, root):
        self.root = Path(root)

    def _path(self, map_name, place):
        return self.root / map_name / _file_name(place)

    def save(self, map_name, place, jpeg_b64):
        path = self._path(map_name, place)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(base64.b64decode(jpeg_b64))
        (path.parent / 'names.json').write_text(json.dumps(self._names(map_name) | {
            path.name: place_key(place)}))

    def delete(self, map_name, place):
        path = self._path(map_name, place)
        path.unlink(missing_ok=True)

    def rename(self, map_name, old, new):
        old_path = self._path(map_name, old)
        if old_path.exists():
            data = base64.b64encode(old_path.read_bytes()).decode('ascii')
            old_path.unlink()
            self.save(map_name, new, data)

    def _names(self, map_name):
        names = self.root / map_name / 'names.json'
        return json.loads(names.read_text()) if names.exists() else {}

    def all(self, map_name):
        """Return {place name: base64 JPEG} of the photos on this map."""
        names = self._names(map_name)
        out = {}
        for path in sorted((self.root / map_name).glob('*.jpg')):
            out[names.get(path.name, path.stem.replace('_', ' '))] = \
                base64.b64encode(path.read_bytes()).decode('ascii')
        return out


def _image(b64):
    return {'type': 'image_url', 'image_url': {'url': f'data:image/jpeg;base64,{b64}'}}


def parse_answer(text):
    start, end = text.find('{'), text.rfind('}')
    try:
        data = json.loads(text[start:end + 1]) if start >= 0 else {}
        return data.get('place'), float(data.get('confidence') or 0.0)
    except (ValueError, TypeError, AttributeError):
        return None, 0.0


async def localize(robot, llm, model, photos, min_confidence=0.6):
    """Localize from a photo if AMCL is not sure; returns what happened, in words."""
    state = await robot.call('get_state', {})
    if not state.get('ok'):
        return f'no state from the robot: {state.get("message")}'
    if state.get('certainty') in ('sure', 'roughly'):
        return f'already localized ({state["certainty"]})'
    references = dict(list(photos.all(state.get('map', '')).items())[:MAX_REFERENCES])
    if not references:
        return 'no place photos yet: save places to have some'
    shot = await robot.call('capture_photo', {})
    if not shot.get('ok'):
        return f'no photo: {shot.get("message")}'
    content = [{'type': 'text', 'text': INTRO},
               {'type': 'text', 'text': 'CURRENT photo (what the robot sees now):'},
               _image(shot['image'])]
    for i, (name, b64) in enumerate(references.items(), 1):
        content += [{'type': 'text', 'text': f'REFERENCE {i}, place "{name}":'}, _image(b64)]
    content.append({'type': 'text', 'text': QUESTION})
    response = await llm.chat.completions.create(
        model=model, temperature=0.0, max_tokens=60,
        messages=[{'role': 'user', 'content': content}])
    place, confidence = parse_answer(response.choices[0].message.content or '')
    if place not in references or confidence < min_confidence:
        return f'no confident match ({place}, {confidence:.2f})'
    result = await robot.call('set_pose_estimate', {'place': place})
    return f'pose estimate at {place} ({confidence:.2f}): {result.get("message")}'


def place_photo_hook(robot, photos, log):
    """Return an on_tool callback keeping the place photos in step with the places."""
    def on_tool(name, args, result):
        if not result.get('ok') or not isinstance(args, dict):
            return
        map_name = getattr(robot, 'welcome', {}).get('map')
        if not map_name:
            return
        if name == 'save_place':
            asyncio.ensure_future(_take(robot, photos, map_name, args.get('name', ''), log))
        elif name == 'delete_place':
            photos.delete(map_name, args.get('name', ''))
        elif name == 'edit_place' and args.get('new_name'):
            photos.rename(map_name, args.get('name', ''), args['new_name'])
    return on_tool


async def _take(robot, photos, map_name, place, log):
    shot = await robot.call('capture_photo', {})
    if shot.get('ok'):
        photos.save(map_name, place, shot['image'])
        log(f'photo of {place} kept for localization')
    else:
        log(f'no photo of {place}: {shot.get("message")}')
