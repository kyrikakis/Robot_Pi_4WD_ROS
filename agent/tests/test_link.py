"""Both ends of the link: the Pi's LinkServer (src/rainbow_brain, no ROS needed)
with a fake brain behind it, and the agent's LinkRobot."""

import asyncio
import json
import os
from pathlib import Path
import socket
import stat
import sys
import threading

import pytest
from rainbow_agent.link import LinkRobot
import websockets

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src' / 'rainbow_brain'))
from rainbow_brain.link import LinkServer, load_token  # noqa: E402, I100, I202

TOKEN = 'secret-token-1234'


class FakeBrain:

    def __init__(self):
        self.said = []
        self.connected = []
        self.calls = []
        self.changed = threading.Event()

    async def call(self, skill, args):
        self.calls.append((skill, args))
        if skill == 'boom':
            raise RuntimeError('broken skill')
        return {'ok': True, 'skill': skill, 'args': args}

    def say(self, text):
        self.said.append(text)
        self.changed.set()

    def on_connected(self, connected):
        self.connected.append(connected)
        self.changed.set()

    def state(self):
        return {'activity': 'idle', 'battery_volts': 7.5, 'places': ['kitchen', 'sofa']}


def free_port():
    with socket.socket() as s:
        s.bind(('127.0.0.1', 0))
        return s.getsockname()[1]


@pytest.fixture
def server():
    brain = FakeBrain()
    link = LinkServer(brain, TOKEN, host='127.0.0.1', port=free_port(),
                      info={'robot': 'rainbow', 'map': 'home_live'},
                      heartbeat_timeout=1.5, log=lambda m: None)
    link.start()
    yield link, brain
    link.stop()


def robot_for(link, token=TOKEN, logs=None):
    log = logs.append if logs is not None else (lambda m: None)
    return LinkRobot(['127.0.0.1'], link.port, token, log)


def wait_for(predicate, timeout=3.0):
    for _ in range(int(timeout / 0.02)):
        if predicate():
            return True
        threading.Event().wait(0.02)
    return False


async def until(predicate, timeout=3.0):
    for _ in range(int(timeout / 0.02)):
        if predicate():
            return True
        await asyncio.sleep(0.02)
    return False


def test_call_say_state_and_events(server):
    link, brain = server

    async def scenario():
        robot = robot_for(link)
        await robot.start()
        assert await robot.wait_connected(3.0)
        assert robot.welcome['map'] == 'home_live'
        assert await until(lambda: brain.connected == [True])
        result = await robot.call('go_to_place', {'place': 'kitchen'})
        assert result == {'ok': True, 'skill': 'go_to_place', 'args': {'place': 'kitchen'}}
        boom = await robot.call('boom', {})
        assert not boom['ok'] and 'broken skill' in boom['message']
        await robot.say('Hello!')
        assert await until(lambda: brain.said == ['Hello!'])
        assert await until(lambda: 'kitchen, sofa' in robot.snapshot())
        link.send_event({'kind': 'arrived', 'place': 'kitchen'})
        event = await asyncio.wait_for(robot.events.get(), 2.0)
        assert event == {'kind': 'arrived', 'place': 'kitchen'}
        await robot.close()

    asyncio.run(scenario())
    assert wait_for(lambda: brain.connected[-1] is False)


def test_heartbeat_keeps_the_link_and_silence_drops_it(server):
    link, brain = server

    async def scenario():
        robot = robot_for(link)
        await robot.start()
        assert await robot.wait_connected(3.0)
        await asyncio.sleep(2.5)  # > heartbeat_timeout: pings keep it alive
        assert link.connected and brain.connected == [True]
        await robot.close()
        # A client that never pings is dropped
        async with websockets.connect(f'ws://127.0.0.1:{link.port}') as ws:
            await ws.send(json.dumps({'type': 'hello', 'token': TOKEN}))
            assert json.loads(await ws.recv())['type'] == 'welcome'
            assert await until(lambda: not link.connected, 3.0)

    asyncio.run(scenario())
    assert wait_for(lambda: brain.connected[-1] is False)


def test_bad_token_is_refused(server):
    link, brain = server
    logs = []

    async def scenario():
        robot = robot_for(link, token='wrong', logs=logs)
        await robot.start()
        assert not await robot.wait_connected(1.0)
        result = await robot.call('stop', {})
        assert not result['ok']
        await robot.close()

    asyncio.run(scenario())
    assert brain.connected == [] and brain.calls == []
    assert any('refused' in m for m in logs)


def test_events_while_away_arrive_on_connect(server):
    link, _ = server
    link.send_event({'kind': 'failed', 'place': 'balcony', 'message': 'blocked'})
    link.send_event({'kind': 'arrived', 'place': 'home'})

    async def scenario():
        robot = robot_for(link)
        await robot.start()
        event = await asyncio.wait_for(robot.events.get(), 3.0)
        await robot.close()
        return event

    event = asyncio.run(scenario())
    assert event['kind'] == 'link_restored'
    assert [e['place'] for e in event['while_away']] == ['balcony', 'home']


def test_a_new_agent_replaces_the_old(server):
    link, brain = server

    async def scenario():
        # The first agent is a raw client (a LinkRobot would reconnect by itself)
        async with websockets.connect(f'ws://127.0.0.1:{link.port}') as first:
            await first.send(json.dumps({'type': 'hello', 'token': TOKEN}))
            assert json.loads(await first.recv())['type'] == 'welcome'
            second = robot_for(link)
            await second.start()
            assert await second.wait_connected(3.0)
            assert (await second.call('where_am_i', {}))['ok']
            with pytest.raises(websockets.ConnectionClosed):
                while True:  # the first one was closed by the server
                    await asyncio.wait_for(first.recv(), 2.0)
            await second.close()

    asyncio.run(scenario())
    assert brain.connected[:2] == [True, True]


def test_token_file_is_created_private(tmp_path):
    path = tmp_path / 'sub' / 'link_token'
    token = load_token(str(path))
    assert len(token) == 32 and load_token(str(path)) == token
    assert stat.S_IMODE(os.stat(path).st_mode) == 0o600


def test_utterances_reach_the_agent(server):
    link, _ = server

    async def scenario():
        robot = robot_for(link)
        await robot.start()
        assert await robot.wait_connected(3.0)
        link.send({'type': 'utterance', 'audio': 'AAA=', 'rate': 16000, 'follow_up': False})
        link.send({'type': 'utterance', 'handled': True, 'text': 'stop', 'action': 'stopped'})
        first = await asyncio.wait_for(robot.utterances.get(), 2.0)
        second = await asyncio.wait_for(robot.utterances.get(), 2.0)
        await robot.close()
        return first, second

    first, second = asyncio.run(scenario())
    assert first['audio'] == 'AAA=' and second['handled'] is True
