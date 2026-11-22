"""A conversation with Rainbow: the session and the model's tool loop.

One turn: the user's text (or a robot event) goes to the model with the
system prompt (persona + the robot's current state), the session so far and
the tools. The model may call skills (at most max_tool_calls per turn); each
result goes back to it, and its final text is the reply, to be spoken.

Turns run one at a time. After session_idle seconds without a turn, a new
session starts. With a memory (rainbow_agent.memory), the finished session is
summarised and stored in the background, and a new session starts by
recalling what is relevant to its first words; memory failures are logged and
never break the conversation.

Robot events: a failure starts a turn (the robot should tell someone); other
events are noted and the model sees them with the next turn.

Guards (rainbow_agent.guards): a clear request for an action that got no tool
call is retried once, with a note that nothing happened (llama-server's
tool_choice "required" does not force a call with this model); invented place descriptions and
rooms are dropped; only one move runs per turn.
"""

import asyncio
import json
import re
import time

from rainbow_agent.guards import ground_args, MOVES, wants_action
from rainbow_agent.persona import system_prompt
from rainbow_agent.skills import TOOLS

EVENT_PREFIX = '[Robot event] '
NUDGE = (EVENT_PREFIX + 'You answered without calling a tool, so nothing happened. '
         'Call the matching tool now; do not reply with text.')
MAX_SENTENCES = 3
MAX_WORDS = 40

# Emoji and pictographs, and markdown marks: espeak would read them out or choke
_UNSPEAKABLE = re.compile(r'[\U0001F000-\U0001FAFF☀-➿️‍]|[*_#`]+')


def speakable(text):
    """The reply as one plain line for espeak: no emoji, markdown or line breaks."""
    text = _UNSPEAKABLE.sub('', text)
    text = re.sub(r'^\s*(?:[-•]|\d+\.)\s+', '', text, flags=re.MULTILINE)
    # At most a few sentences: a looping model must not make the robot ramble
    sentences = re.split(r'(?<=[.!?])\s+', ' '.join(text.split()))
    text = ' '.join(sentences[:MAX_SENTENCES])
    words = text.split()
    if len(words) > MAX_WORDS:
        text = ' '.join(words[:MAX_WORDS]).rstrip(',;:') + '.'
    return text


def describe_event(event):
    kind = event.get('kind')
    if kind == 'arrived':
        return f"arrived at the {event['place']}"
    if kind == 'failed':
        return event.get('message') or f"could not get to the {event.get('place')}"
    if kind == 'face_seen':
        return f"I can see {event.get('name')} (face recognition)"
    if kind == 'stopped_by_voice':
        return 'someone said stop, so I stopped'
    if kind == 'cancelled':
        return f"the trip to the {event.get('place')} was cancelled"
    if kind == 'link_restored':
        away = '; '.join(describe_event(e) for e in event.get('while_away') or [])
        return 'reconnected to my body' + (f'; while I was away: {away}' if away else '')
    return json.dumps(event)


class Conversation:

    def __init__(self, llm, model, robot, cfg, on_tool=None, memory=None, log=print):
        self.llm = llm                  # an openai.AsyncOpenAI (or a fake in tests)
        self.model = model
        self.robot = robot
        self.cfg = cfg
        self.on_tool = on_tool          # callback(name, args, result), for display
        self.lock = asyncio.Lock()
        self.messages = []
        self.pending_events = []
        self.last_turn = 0.0
        self.memory = memory
        self.log = log
        self.memories = []              # recalled for this session
        self.session_started = time.time()
        self._storing = set()

    def new_session(self):
        """Start afresh; the finished session goes to memory in the background."""
        if self.memory is not None and self.messages:
            task = asyncio.ensure_future(self._store(self.messages, self.session_started))
            self._storing.add(task)
            task.add_done_callback(self._storing.discard)
        self.messages = []
        self.pending_events = []
        self.memories = []
        self.session_started = time.time()

    async def _store(self, messages, started):
        try:
            stored = await self.memory.store_session(messages, started)
            if stored:
                self.log(f'remembered: {stored[0]} facts: {stored[1]}')
        except Exception as e:  # memory must never break the conversation
            self.log(f'could not store the session: {type(e).__name__}: {e}')

    async def close(self):
        """End the session now (shutdown) and wait until it is stored."""
        self.new_session()
        if self._storing:
            await asyncio.gather(*self._storing, return_exceptions=True)

    def close_if_idle(self):
        """End the session if it has been idle long enough (call now and then)."""
        if self.messages and time.monotonic() - self.last_turn > self.cfg.session_idle:
            self.new_session()

    async def _recall(self, text):
        if self.memory is None:
            return
        try:
            self.memories = await self.memory.recall(text)
        except Exception as e:
            self.log(f'could not recall: {type(e).__name__}: {e}')

    async def user_says(self, text):
        """The user's words (already transcribed). Returns the reply text."""
        async with self.lock:
            self._maybe_new_session()
            if not self.messages:
                await self._recall(text)
            for note in self.pending_events:
                self.messages.append({'role': 'user', 'content': EVENT_PREFIX + note})
            self.pending_events = []
            self.messages.append({'role': 'user', 'content': text})
            return await self._turn(text)

    async def handled_locally(self, text, action):
        """The robot understood a simple command itself; keep the session in step."""
        async with self.lock:
            self._maybe_new_session()
            self.messages.append({'role': 'user', 'content': text})
            self.messages.append({'role': 'user', 'content': EVENT_PREFIX +
                                  f'I understood that myself and {action}; already answered.'})
            self.last_turn = time.monotonic()

    async def robot_event(self, event):
        """An event from the robot. Returns a reply to speak, or None."""
        note = describe_event(event)
        if event.get('kind') != 'failed':
            self.pending_events.append(note)
            return None
        async with self.lock:
            self.messages.append({'role': 'user', 'content': EVENT_PREFIX + note})
            return await self._turn()

    def _maybe_new_session(self):
        if self.messages and time.monotonic() - self.last_turn > self.cfg.session_idle:
            self.new_session()

    def _recent_user_words(self, n=3):
        said = [m['content'] for m in self.messages
                if m['role'] == 'user' and isinstance(m['content'], str)
                and not m['content'].startswith(EVENT_PREFIX)]
        return ' '.join(said[-n:])

    async def _complete(self, tools=True, extra=()):
        system = {'role': 'system',
                  'content': system_prompt(self.robot.snapshot(), self.memories)}
        kwargs = {'tools': TOOLS} if tools else {}
        response = await self.llm.chat.completions.create(
            model=self.model, messages=[system, *self.messages, *extra],
            temperature=self.cfg.temperature, max_tokens=self.cfg.max_reply_tokens,
            **kwargs)
        return response.choices[0].message

    async def _turn(self, said=None):
        """said: the person's words that started this turn (None for robot events)."""
        calls = 0
        moved = False
        while True:
            message = await self._complete(tools=calls < self.cfg.max_tool_calls)
            tool_calls = message.tool_calls or []
            if not tool_calls and calls == 0 and said and wants_action(said):
                # Asked to act but only talked: retry once with a note (not kept)
                retry = await self._complete(extra=[
                    {'role': 'assistant', 'content': message.content or ''},
                    {'role': 'user', 'content': NUDGE}])
                if retry.tool_calls:
                    message, tool_calls = retry, retry.tool_calls
            entry = {'role': 'assistant', 'content': message.content or ''}
            if tool_calls:
                entry['tool_calls'] = [
                    {'id': c.id, 'type': 'function',
                     'function': {'name': c.function.name,
                                  'arguments': c.function.arguments}}
                    for c in tool_calls]
            self.messages.append(entry)
            if not tool_calls:
                self.last_turn = time.monotonic()
                return speakable(message.content or '')
            photos = []
            for c in tool_calls:
                calls += 1
                try:
                    args = json.loads(c.function.arguments or '{}')
                except json.JSONDecodeError:
                    args = None
                name = c.function.name
                if not isinstance(args, dict):
                    result = {'ok': False, 'message': 'arguments were not a JSON object'}
                elif name in MOVES and moved:
                    result = {'ok': False, 'message': 'only one move per request; '
                              'tell the person what you did and let them ask again'}
                else:
                    args, dropped = ground_args(name, args, self._recent_user_words())
                    result = await self.robot.call(name, args)
                    if dropped:
                        result['note'] = (f'{" and ".join(dropped)} left out: only save what '
                                          'the person said')
                    moved = moved or name in MOVES
                if self.on_tool:
                    self.on_tool(name, args, result)
                if result.get('image'):
                    # Tool results are text only: the photo follows as a message
                    photos.append(result.pop('image'))
                    result['photo'] = 'attached below'
                self.messages.append({'role': 'tool', 'tool_call_id': c.id,
                                      'content': json.dumps(result)})
            for photo in photos:
                self.messages.append({'role': 'user', 'content': [
                    {'type': 'text', 'text': EVENT_PREFIX + 'The photo from my camera:'},
                    {'type': 'image_url',
                     'image_url': {'url': f'data:image/jpeg;base64,{photo}'}}]})
