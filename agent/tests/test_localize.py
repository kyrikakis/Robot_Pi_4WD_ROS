"""Place photos, startup localization and photos in the conversation (fakes only)."""

import asyncio
import base64
import json
from types import SimpleNamespace as NS

from rainbow_agent.config import Config
from rainbow_agent.conversation import Conversation
from rainbow_agent.localize import localize, parse_answer, place_photo_hook, PlacePhotos

JPEG = base64.b64encode(b'\xff\xd8 fake jpeg \xff\xd9').decode('ascii')


class FakeRobot:

    def __init__(self, certainty='lost', photo=True):
        self.certainty, self.photo = certainty, photo
        self.calls = []
        self.welcome = {'map': 'home_live'}

    async def call(self, name, args):
        self.calls.append((name, args))
        if name == 'get_state':
            return {'ok': True, 'map': 'home_live', 'certainty': self.certainty}
        if name == 'capture_photo':
            return {'ok': True, 'image': JPEG} if self.photo else \
                {'ok': False, 'message': 'no camera'}
        return {'ok': True, 'message': f'{name} done'}

    def snapshot(self):
        return 'Activity: idle.'


class FakeLLM:

    def __init__(self, *replies):
        self.replies, self.requests = list(replies), []
        self.chat = NS(completions=NS(create=self.create))

    async def create(self, **kwargs):
        self.requests.append(kwargs)
        reply = self.replies.pop(0)
        return NS(choices=[NS(message=reply if isinstance(reply, NS) else
                              NS(content=reply, tool_calls=None))])


def run(coro):
    return asyncio.run(coro)


def test_place_photos_follow_the_places(tmp_path):
    photos = PlacePhotos(tmp_path)
    photos.save('home_live', 'The Kitchen', JPEG)
    photos.save('home_live', 'kids room', JPEG)
    assert set(photos.all('home_live')) == {'kitchen', 'kids room'}
    photos.rename('home_live', 'kitchen', 'galley')
    photos.delete('home_live', 'kids room')
    assert list(photos.all('home_live')) == ['galley']
    assert photos.all('other_map') == {}


def test_parse_answer():
    assert parse_answer('Sure: {"place": "kitchen", "confidence": 0.8}') == ('kitchen', 0.8)
    assert parse_answer('no idea') == (None, 0.0)


def test_localize_only_when_needed_and_confident(tmp_path):
    photos = PlacePhotos(tmp_path)
    photos.save('home_live', 'kitchen', JPEG)
    photos.save('home_live', 'sofa', JPEG)

    robot = FakeRobot(certainty='sure')
    assert 'already localized' in run(localize(robot, FakeLLM(), 'm', photos))

    robot = FakeRobot(certainty='roughly')   # not good enough: a photo is tried
    llm = FakeLLM('{"place": "kitchen", "confidence": 0.9}')
    assert 'pose estimate at kitchen' in run(localize(robot, llm, 'm', photos))

    robot, llm = FakeRobot(), FakeLLM('{"place": "sofa", "confidence": 0.9}')
    assert 'pose estimate at sofa' in run(localize(robot, llm, 'm', photos))
    assert robot.calls[-1] == ('set_pose_estimate', {'place': 'sofa'})
    images = [c for c in llm.requests[0]['messages'][0]['content'] if c['type'] == 'image_url']
    assert len(images) == 3   # now + two references

    robot = FakeRobot()
    assert 'no confident match' in run(localize(
        robot, FakeLLM('{"place": "sofa", "confidence": 0.3}'), 'm', photos))
    assert all(name != 'set_pose_estimate' for name, _ in robot.calls)

    assert 'no photo' in run(localize(FakeRobot(photo=False), FakeLLM(), 'm', photos))
    assert 'no place photos' in run(localize(FakeRobot(), FakeLLM(), 'm',
                                             PlacePhotos(tmp_path / 'empty')))


def test_saving_a_place_takes_its_photo(tmp_path):
    async def scenario():
        robot, photos, logs = FakeRobot(), PlacePhotos(tmp_path), []
        hook = place_photo_hook(robot, photos, logs.append)
        hook('save_place', {'name': 'the hallway'}, {'ok': True})
        await asyncio.sleep(0.05)
        return photos, logs
    photos, logs = run(scenario())
    assert list(photos.all('home_live')) == ['hallway'] and logs


def test_a_photo_reaches_the_model_as_an_image():
    call = NS(id='c1', function=NS(name='capture_photo', arguments='{}'))
    llm = FakeLLM(NS(content='', tool_calls=[call]), 'I see a sofa!')
    conv = Conversation(llm, 'm', FakeRobot(), Config())
    assert run(conv.user_says('What do you see?')) == 'I see a sofa!'
    tool_msg = next(m for m in conv.messages if m['role'] == 'tool')
    assert 'image' not in json.loads(tool_msg['content'])
    photo_msg = conv.messages[-2]
    assert photo_msg['role'] == 'user'
    assert photo_msg['content'][1]['image_url']['url'].endswith(JPEG)
