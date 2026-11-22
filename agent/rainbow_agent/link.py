"""LinkRobot: the real robot, through rainbow_brain's WebSocket link.

Same interface as SimRobot (call, snapshot, events) plus say(). Connects to
the first robot host that answers, says hello with the shared token, pings
every second (the brain drops a silent agent after 3 s and goes back to LOCAL
mode), and reconnects by itself. Protocol: src/rainbow_brain/rainbow_brain/link.py.

The token comes from the robot once: `rainbow-agent pair`.
"""

import asyncio
import contextlib
import itertools
import json

import websockets

# Moves can take a while (a full turn, a 1 m drive); questions should be quick
SLOW = {'turn': 40.0, 'drive_forward': 40.0}
TIMEOUT = 10.0


class LinkRobot:

    def __init__(self, hosts, port, token, log=print):
        self.urls = [f'ws://{h}:{port}' for h in hosts]
        self.token = token
        self.log = log
        self.events = asyncio.Queue()
        self.utterances = asyncio.Queue()   # what people said to the robot
        self.calls = []
        self.state = {}
        self.welcome = {}
        self.connected = asyncio.Event()
        self._ws = None
        self._pending = {}
        self._ids = itertools.count(1)
        self._task = None

    # Lifecycle

    async def start(self):
        self._task = asyncio.ensure_future(self._run())

    async def close(self):
        if self._task is not None:
            self._task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await self._task

    async def wait_connected(self, timeout):
        with contextlib.suppress(asyncio.TimeoutError):
            await asyncio.wait_for(self.connected.wait(), timeout)
        return self.connected.is_set()

    async def _run(self):
        delay = 1.0
        while True:
            for url in self.urls:
                try:
                    async with websockets.connect(url, open_timeout=3, max_size=2**24) as ws:
                        if await self._hello(ws, url):
                            delay = 1.0
                            await self._session(ws)
                except (OSError, asyncio.TimeoutError, websockets.WebSocketException) as e:
                    if self.connected.is_set():
                        self.log(f'link to {url} lost: {e}')
                finally:
                    self._disconnected()
            await asyncio.sleep(delay)
            delay = min(delay * 2, 5.0)

    async def _hello(self, ws, url):
        await ws.send(json.dumps({'type': 'hello', 'token': self.token}))
        reply = json.loads(await asyncio.wait_for(ws.recv(), 5.0))
        if reply.get('type') != 'welcome':
            self.log(f'robot at {url} refused the link: {reply.get("message")}')
            return False
        self.welcome = reply
        self._ws = ws
        self.connected.set()
        self.log(f'linked to {reply.get("robot", "robot")} at {url} (map {reply.get("map")})')
        return True

    async def _session(self, ws):
        pinger = asyncio.ensure_future(self._ping(ws))
        try:
            async for raw in ws:
                self._message(json.loads(raw))
        finally:
            pinger.cancel()

    async def _ping(self, ws):
        while True:
            await ws.send(json.dumps({'type': 'ping'}))
            await asyncio.sleep(1.0)

    def _message(self, msg):
        kind = msg.get('type')
        if kind == 'result':
            future = self._pending.pop(msg.get('id'), None)
            if future is not None and not future.done():
                future.set_result(msg.get('result') or {'ok': False})
        elif kind == 'event':
            self.events.put_nowait(msg.get('event') or {})
        elif kind == 'state':
            self.state = msg.get('state') or {}
        elif kind == 'utterance':
            self.utterances.put_nowait(msg)

    def _disconnected(self):
        self._ws = None
        self.connected.clear()
        for future in self._pending.values():
            if not future.done():
                future.set_result({'ok': False, 'message': 'lost the connection to my body'})
        self._pending.clear()

    # The robot interface

    async def call(self, name, args):
        self.calls.append((name, dict(args)))
        if self._ws is None:
            return {'ok': False, 'message': "I can't reach my body right now"}
        call_id = next(self._ids)
        future = asyncio.get_running_loop().create_future()
        self._pending[call_id] = future
        try:
            await self._ws.send(json.dumps({'type': 'call', 'id': call_id, 'skill': name,
                                            'args': args}))
            return await asyncio.wait_for(future, SLOW.get(name, TIMEOUT))
        except asyncio.TimeoutError:
            return {'ok': False, 'message': f'{name} took too long'}
        except websockets.ConnectionClosed:
            return {'ok': False, 'message': 'lost the connection to my body'}
        finally:
            self._pending.pop(call_id, None)

    async def say(self, text):
        if self._ws is not None and text:
            with contextlib.suppress(websockets.ConnectionClosed):
                await self._ws.send(json.dumps({'type': 'say', 'text': text}))

    def snapshot(self):
        if not self.connected.is_set():
            return 'Not connected to my body: I cannot move or check anything right now.'
        s = self.state
        parts = [f"Activity: {s.get('activity', 'unknown')}."]
        if s.get('battery_volts'):
            parts.append(f"Battery {s['battery_volts']:.1f} V.")
        parts.append(f"Known places: {', '.join(s.get('places') or []) or 'none yet'}.")
        return ' '.join(parts)
