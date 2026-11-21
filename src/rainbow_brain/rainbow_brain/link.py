"""The link to the Mac agent: a WebSocket server (no ROS in this module).

Runs its own asyncio loop in a thread. One agent at a time: a new connection
replaces the old one. Protocol, JSON text messages (docs/brain.md):

  agent -> brain                      brain -> agent
  hello {token}                       welcome {robot, map, protocol} | error {message}
  ping                                pong
  call {id, skill, args}              result {id, result}
  say {text}                          event {event}      state {state}

The agent must say hello with the right token first, then ping at least every
heartbeat_timeout seconds; otherwise it is dropped and the brain goes back to
LOCAL mode. Events that happen while no agent is connected are kept (the last
few) and sent as one link_restored event when an agent connects again.

The brain plugs in through a handler object with:
  async call(skill, args) -> dict    run a skill (on the link's loop)
  say(text)                          speak
  on_connected(connected: bool)      the mode changed
  state() -> dict                    the state to push
"""

import asyncio
import contextlib
import json
import os
import secrets
import threading
import time

import websockets

PROTOCOL = 1
OFFLINE_EVENTS = 10


def load_token(path):
    """Return the shared token, creating it (owner-readable only) on first use."""
    if not os.path.exists(path):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        with os.fdopen(fd, 'w') as f:
            f.write(secrets.token_hex(16) + '\n')
    with open(path) as f:
        return f.read().strip()


class LinkServer:

    def __init__(self, handler, token, host='0.0.0.0', port=8766, info=None,
                 heartbeat_timeout=3.0, state_period=5.0, log=print):
        self.handler = handler
        self.token = token
        self.host, self.port = host, port
        self.info = info or {}
        self.heartbeat_timeout = heartbeat_timeout
        self.state_period = state_period
        self.log = log
        self.loop = None
        self.ws = None              # the connected, authenticated agent
        self.last_seen = 0.0
        self.offline = []           # events while no agent was connected
        self._last_state = 0.0
        self._thread = None
        self._ready = threading.Event()
        self._stop = None

    # Lifecycle (from any thread)

    def start(self):
        self._thread = threading.Thread(target=self._run, name='link', daemon=True)
        self._thread.start()
        if not self._ready.wait(5.0):
            raise RuntimeError('link server did not start')

    def stop(self):
        if self.loop is not None and self._stop is not None:
            self.loop.call_soon_threadsafe(self._stop.set)
            self._thread.join(5.0)

    @property
    def connected(self):
        return self.ws is not None

    def send_event(self, event):
        """Send an event to the agent, or keep it for when one connects (thread-safe)."""
        if self.loop is not None:
            self.loop.call_soon_threadsafe(self._event, event)

    def send_state(self):
        if self.loop is not None:
            self.loop.call_soon_threadsafe(self._push_state)

    # The loop

    def _run(self):
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self._main())

    async def _main(self):
        self._stop = asyncio.Event()
        async with websockets.serve(self._serve, self.host, self.port):
            self._ready.set()
            watch = asyncio.ensure_future(self._watch())
            await self._stop.wait()
            watch.cancel()
            if self.ws is not None:
                await self.ws.close()

    async def _watch(self):
        """Drop a silent agent; push the state now and then."""
        while True:
            await asyncio.sleep(0.5)
            now = time.monotonic()
            if self.ws is not None and now - self.last_seen > self.heartbeat_timeout:
                self.log('agent silent: dropping it')
                await self.ws.close()
            if self.ws is not None and now - self._last_state > self.state_period:
                self._push_state()

    async def _serve(self, ws, path=None):
        try:
            hello = json.loads(await asyncio.wait_for(ws.recv(), 5.0))
        except (asyncio.TimeoutError, ValueError, websockets.ConnectionClosed):
            return
        if hello.get('type') != 'hello' or not secrets.compare_digest(
                str(hello.get('token', '')), self.token):
            await self._send(ws, {'type': 'error', 'message': 'bad token'})
            self.log('agent refused: bad token')
            return
        old, self.ws = self.ws, ws  # take over first: the mode stays remote
        if old is not None:
            self.log('a new agent replaces the connected one')
            await old.close()
        self.last_seen = time.monotonic()
        await self._send(ws, {'type': 'welcome', 'protocol': PROTOCOL, **self.info})
        self.log('agent connected')
        self.handler.on_connected(True)
        if self.offline:
            await self._send(ws, {'type': 'event', 'event': {
                'kind': 'link_restored', 'while_away': self.offline}})
            self.offline = []
        self._push_state()
        try:
            async for raw in ws:
                self.last_seen = time.monotonic()
                await self._message(ws, raw)
        except websockets.ConnectionClosed:
            pass
        finally:
            if self.ws is ws:
                self.ws = None
                self.log('agent disconnected')
                self.handler.on_connected(False)

    async def _message(self, ws, raw):
        try:
            msg = json.loads(raw)
        except ValueError:
            return
        kind = msg.get('type')
        if kind == 'ping':
            await self._send(ws, {'type': 'pong'})
        elif kind == 'say':
            self.handler.say(str(msg.get('text', '')))
        elif kind == 'call':
            asyncio.ensure_future(self._call(ws, msg))

    async def _call(self, ws, msg):
        try:
            result = await self.handler.call(str(msg.get('skill')), msg.get('args') or {})
        except Exception as e:  # a skill must never take the link down
            result = {'ok': False, 'message': f'{type(e).__name__}: {e}'}
        await self._send(ws, {'type': 'result', 'id': msg.get('id'), 'result': result})

    def _event(self, event):
        if self.ws is None:
            self.offline = (self.offline + [event])[-OFFLINE_EVENTS:]
        else:
            asyncio.ensure_future(self._send(self.ws, {'type': 'event', 'event': event}))

    def _push_state(self):
        self._last_state = time.monotonic()
        if self.ws is not None:
            asyncio.ensure_future(self._send(self.ws, {'type': 'state',
                                                       'state': self.handler.state()}))

    @staticmethod
    async def _send(ws, msg):
        with contextlib.suppress(websockets.ConnectionClosed):
            await ws.send(json.dumps(msg))
