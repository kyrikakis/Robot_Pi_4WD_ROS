"""A conversation with Rainbow: the session and the model's tool loop.

One turn: the user's text (or a robot event) goes to the model with the
system prompt (persona + the robot's current state), the session so far and
the tools. The model may call skills (at most max_tool_calls per turn); each
result goes back to it, and its final text is the reply, to be spoken.

Turns run one at a time. After session_idle seconds without a turn, a new
session starts (the old one is dropped for now; summaries come with memory,
build step 5).

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
    if kind == 'cancelled':
        return f"the trip to the {event.get('place')} was cancelled"
    if kind == 'link_restored':
        away = '; '.join(describe_event(e) for e in event.get('while_away') or [])
        return 'reconnected to my body' + (f'; while I was away: {away}' if away else '')
    return json.dumps(event)


class Conversation:

    def __init__(self, llm, model, robot, cfg, on_tool=None):
        self.llm = llm                  # an openai.AsyncOpenAI (or a fake in tests)
        self.model = model
        self.robot = robot
        self.cfg = cfg
        self.on_tool = on_tool          # callback(name, args, result), for display
        self.lock = asyncio.Lock()
        self.messages = []
        self.pending_events = []
        self.last_turn = 0.0

    def new_session(self):
        self.messages = []
        self.pending_events = []

    async def user_says(self, text):
        """The user's words (already transcribed). Returns the reply text."""
        async with self.lock:
            self._maybe_new_session()
            for note in self.pending_events:
                self.messages.append({'role': 'user', 'content': EVENT_PREFIX + note})
            self.pending_events = []
            self.messages.append({'role': 'user', 'content': text})
            return await self._turn(text)

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
                if m['role'] == 'user' and not m['content'].startswith(EVENT_PREFIX)]
        return ' '.join(said[-n:])

    async def _complete(self, tools=True, extra=()):
        system = {'role': 'system', 'content': system_prompt(self.robot.snapshot())}
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
                self.messages.append({'role': 'tool', 'tool_call_id': c.id,
                                      'content': json.dumps(result)})
