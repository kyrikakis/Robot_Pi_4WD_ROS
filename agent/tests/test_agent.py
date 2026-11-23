"""Agent tests with a scripted fake model (no servers, no downloads)."""

import asyncio
import json
from types import SimpleNamespace as NS

from rainbow_agent.config import Config
from rainbow_agent.conversation import Conversation, EVENT_PREFIX
from rainbow_agent.evaluate import check, value_matches
from rainbow_agent.robot import SimRobot
from rainbow_agent.skills import NAMES
import yaml


def tool_call(skill, **args):
    return NS(id=f'call-{skill}', function=NS(name=skill, arguments=json.dumps(args)))


def message(content='', calls=None):
    return NS(choices=[NS(message=NS(content=content, tool_calls=calls))])


class FakeLLM:
    """Returns the scripted messages in order; records every request."""

    def __init__(self, *script):
        self.script = list(script)
        self.requests = []
        self.chat = NS(completions=NS(create=self.create))

    async def create(self, **kwargs):
        self.requests.append(kwargs)
        return self.script.pop(0)


def run(coro):
    return asyncio.run(coro)


def make(*script, robot=None, **cfg):
    robot = robot or SimRobot(time_scale=0.0)
    llm = FakeLLM(*script)
    return Conversation(llm, 'm', robot, Config(**cfg)), llm, robot


def test_tool_loop_runs_skill_and_returns_reply():
    conv, llm, robot = make(
        message(calls=[tool_call('go_to_place', place='Kitchen')]),
        message('On my way to the kitchen!'))
    reply = run(conv.user_says('go to the kitchen'))
    assert reply == 'On my way to the kitchen!'
    assert robot.calls == [('go_to_place', {'place': 'Kitchen'})]
    tool_msg = conv.messages[2]
    assert tool_msg['role'] == 'tool' and json.loads(tool_msg['content'])['ok']
    # The system prompt carries the robot's state, and tools are offered
    assert 'Known places' in llm.requests[0]['messages'][0]['content']
    assert llm.requests[0]['tools']


def test_tool_call_cap_forces_a_reply():
    calls = [message(calls=[tool_call('where_am_i')]) for _ in range(2)]
    conv, llm, robot = make(*calls, message('Done.'), max_tool_calls=2)
    assert run(conv.user_says('where?')) == 'Done.'
    assert 'tools' not in llm.requests[-1]
    assert len(robot.calls) == 2


def test_bad_arguments_are_reported_to_the_model():
    bad = NS(id='x', function=NS(name='turn', arguments='not json'))
    conv, _, robot = make(message(calls=[bad]), message('Oops.'))
    run(conv.user_says('turn'))
    assert robot.calls == []
    assert not json.loads(conv.messages[2]['content'])['ok']


def test_events_failure_starts_a_turn_others_wait():
    conv, llm, _ = make(message('Hi!'), message("I couldn't reach the balcony."))
    assert run(conv.robot_event({'kind': 'arrived', 'place': 'sofa'})) is None
    run(conv.user_says('hello'))
    contents = [m['content'] for m in llm.requests[0]['messages']]
    assert EVENT_PREFIX + 'arrived at the sofa' in contents
    reply = run(conv.robot_event({'kind': 'failed', 'place': 'balcony', 'message': 'blocked'}))
    assert reply == "I couldn't reach the balcony."


def test_idle_session_starts_fresh():
    conv, _, _ = make(message('One.'), message('Two.'), session_idle=0.0)
    run(conv.user_says('a'))
    run(conv.user_says('b'))
    assert [m['content'] for m in conv.messages if m['role'] == 'user'] == ['b']


def test_sim_robot_moves_and_fails():
    async def scenario():
        robot = SimRobot(time_scale=0.0)
        assert (await robot.call('go_to_place', {'place': 'the kitchen'}))['ok']
        assert (await robot.events.get()) == {'kind': 'arrived', 'place': 'kitchen'}
        where = await robot.call('where_am_i', {})
        assert 'kitchen' in where['description']
        await robot.call('go_to_place', {'place': 'balcony'})
        assert (await robot.events.get())['kind'] == 'failed'
        assert not (await robot.call('go_to_place', {'place': 'garage'}))['ok']
        assert not (await robot.call('drive_forward', {'meters': 3}))['ok']
        assert not (await robot.call('turn', {'wrong': 1}))['ok']
    run(scenario())


def test_every_tool_has_a_sim_handler():
    robot = SimRobot()
    assert all(hasattr(robot, f'_{n}') for n in NAMES)


def test_eval_matching():
    assert value_matches('kids', 'kids room')
    assert value_matches({'min': 45, 'max': 135}, 90)
    assert value_matches({'abs': 180}, -180)
    assert value_matches(0.3, 0.33) and not value_matches(0.3, 0.5)
    assert value_matches(True, True) and not value_matches(True, 'yes')
    case = {'expect': [{'name': 'go_to_place', 'args': {'place': 'hallway'}},
                       {'name': 'stop'}]}
    assert check(case, [('go_to_place', {'place': 'Hallway'}), ('stop', {})], ['Ok.']) == []
    assert check(case, [('stop', {}), ('go_to_place', {'place': 'hallway'})], ['Ok.'])
    assert check({'forbid': ['any']}, [('stop', {})], ['Ok.'])
    assert check({}, [], ['**Hi**']) and check({}, [], ['- one\n- two'])


def test_eval_cases_file_is_valid():
    path = __file__.rsplit('/tests/', 1)[0] + '/eval/cases.yaml'
    cases = yaml.safe_load(open(path))
    ids = [c['id'] for c in cases]
    assert len(ids) == len(set(ids))
    for case in cases:
        says = case['say'] if isinstance(case['say'], list) else [case['say']]
        assert all(isinstance(s, str) and s for s in says), case['id']
        for exp in case.get('expect') or []:
            assert exp['name'] in NAMES, case['id']
        for name in case.get('forbid') or []:
            assert name == 'any' or name in NAMES, case['id']


def test_replies_are_made_speakable():
    from rainbow_agent.conversation import speakable
    assert speakable('Why?\nBecause it had a **virus**! 😊') == 'Why? Because it had a virus!'
    assert speakable('- one\n- two\n1. three') == 'one two three'
    assert speakable("I'm fine, thanks.") == "I'm fine, thanks."
    assert speakable('One. Two. Three. Four. Five.') == 'One. Two. Three.'
    assert len(speakable('I am ready to go ' * 30).split()) == 40


def test_eval_catches_invented_content():
    case = {'expect': [{'name': 'save_place',
                        'args': {'name': 'nook', 'description': {'empty': True}}}],
            'reply_not': ['sun']}
    assert check(case, [('save_place', {'name': 'nook'})], ['Saved.']) == []
    assert check(case, [('save_place', {'name': 'nook', 'description': 'cozy'})], ['Saved.'])
    assert check(case, [('save_place', {'name': 'nook'})], ['Saved, the sun is out.'])


def test_guards_action_words():
    from rainbow_agent.guards import wants_action
    for text in ('Stop!', 'stop', 'Wait', 'Go to the kitchen.', 'go home now', 'Turn left',
                 'Move forward a bit', 'Remember this spot as the nook',
                 'This is the bathroom, remember it.', 'Forget the hallway',
                 "Let's call the sofa the couch from now on.",
                 'Yes, the desk is right where you think it is.',
                 'Rainbow, what do you see?', "What's in front of you?",
                 'Look at this!', 'Remember my face, I am Mia'):
        assert wants_action(text), text
    for text in ('What is your name?', 'Do you remember my name?', 'Tell me a joke',
                 "Don't stop believing is a song", 'I like turning pages'):
        assert not wants_action(text), text


def test_guards_ground_args():
    from rainbow_agent.guards import ground_args
    said = 'Remember this spot as the reading corner, by the window.'
    args, dropped = ground_args('save_place', {'name': 'reading corner',
                                               'description': 'by the window',
                                               'room': 'kids room'}, said)
    assert args == {'name': 'reading corner', 'description': 'by the window'}
    assert dropped == ['room']
    assert ground_args('turn', {'degrees': 90}, '') == ({'degrees': 90}, [])
    said = 'This is the bathroom, remember it.'
    args, _ = ground_args('save_place', {'name': 'bathroom', 'description': 'bathroom',
                                         'room': 'bathroom'}, said)
    assert args == {'name': 'bathroom', 'room': 'bathroom'}


def test_forced_retry_when_action_asked_but_only_talked():
    conv, llm, robot = make(message('I stopped.'), message(calls=[tool_call('stop')]),
                            message('Stopped.'))
    assert run(conv.user_says('Stop!')) == 'Stopped.'
    assert robot.calls == [('stop', {})]
    assert 'nothing happened' in llm.requests[1]['messages'][-1]['content']
    # The nudge is not kept in the session
    assert not any('nothing happened' in str(m.get('content')) for m in conv.messages)
    # No retry for plain talk
    conv, llm, robot = make(message('I am Rainbow.'))
    run(conv.user_says('What is your name?'))
    assert len(llm.requests) == 1


def test_one_move_per_turn_and_invented_args_dropped():
    conv, _, robot = make(
        message(calls=[tool_call('drive_forward', meters=0.5),
                       tool_call('drive_forward', meters=0.5)]),
        message(calls=[tool_call('save_place', name='nook', description='cozy chair')]),
        message('Done.'))
    run(conv.user_says('Drive forward five meters'))
    assert robot.calls == [('drive_forward', {'meters': 0.5}), ('save_place', {'name': 'nook'})]
    results = [json.loads(m['content']) for m in conv.messages if m['role'] == 'tool']
    assert not results[1]['ok'] and 'one move' in results[1]['message']
    assert 'note' in results[2]


def test_handled_locally_keeps_the_session_in_step():
    conv, llm, _ = make(message('Sure!'))
    run(conv.handled_locally('go to the kitchen', 'started going to kitchen'))
    run(conv.user_says('thanks'))
    contents = [m['content'] for m in llm.requests[0]['messages']]
    assert 'go to the kitchen' in contents
    assert any('understood that myself' in c for c in contents)


def test_whisper_silence_hallucinations_are_dropped():
    from rainbow_agent.stt import is_speech
    assert not is_speech(' Thank you. ') and not is_speech('') and not is_speech('you')
    assert is_speech('Go to the kitchen.')
    assert not is_speech('¢$!') and not is_speech("a ʤ0'-%$!")
    assert is_speech("What's 2 plus 2?")
    assert not is_speech('You can make it very special. You can make it very special. '
                         'You can make it very special.')


class FakeMemory:

    def __init__(self):
        self.stored = []
        self.recalled = []

    async def store_session(self, messages, started):
        self.stored.append([m['content'] for m in messages if m['role'] == 'user'])
        return 'summary', []

    async def recall(self, text):
        self.recalled.append(text)
        return ['Fact: Mia likes dinosaurs.']


def test_memory_recall_at_start_and_store_at_end():
    async def scenario():
        memory = FakeMemory()
        llm = FakeLLM(message('Hi Mia!'), message('T-Rex!'), message('Hello again!'))
        conv = Conversation(llm, 'm', SimRobot(time_scale=0.0), Config(), memory=memory,
                            log=lambda m: None)
        await conv.user_says('Hi, I am Mia')
        await conv.user_says('Favourite dinosaur?')
        assert memory.recalled == ['Hi, I am Mia']          # only at the session start
        assert 'Mia likes dinosaurs' in llm.requests[0]['messages'][0]['content']
        conv.last_turn -= 1000
        conv.close_if_idle()
        await conv.close()
        assert memory.stored == [['Hi, I am Mia', 'Favourite dinosaur?']]
        await conv.user_says('Hello')                         # a new session recalls again
        assert memory.recalled[-1] == 'Hello'
    run(scenario())


def test_memory_failures_do_not_break_the_conversation():
    class Broken(FakeMemory):
        async def recall(self, text):
            raise RuntimeError('embeddings server down')

        async def store_session(self, messages, started):
            raise RuntimeError('model down')

    async def scenario():
        logs = []
        conv = Conversation(FakeLLM(message('Hi!')), 'm', SimRobot(time_scale=0.0), Config(),
                            memory=Broken(), log=logs.append)
        assert await conv.user_says('Hello') == 'Hi!'
        await conv.close()
        return logs
    logs = run(scenario())
    assert any('could not recall' in m for m in logs)
    assert any('could not store' in m for m in logs)


class ReportingRobot(SimRobot):
    """A SimRobot that records what the agent reports for Foxglove."""

    def __init__(self):
        super().__init__(time_scale=0.0)
        self.reports = []
        self.logs = []

    def report(self, status):
        self.reports.append(dict(status))

    def log_line(self, level, text):
        self.logs.append((level, text))


def test_status_reports_a_turn():
    robot = ReportingRobot()
    first = message(calls=[tool_call('turn', degrees=90)])
    first.usage = NS(prompt_tokens=900, completion_tokens=20)
    second = message('Turned left.')
    second.usage = NS(prompt_tokens=950, completion_tokens=5)
    conv, _, _ = make(first, second, robot=robot)
    assert run(conv.user_says('turn left')) == 'Turned left.'
    phases = [r['phase'] for r in robot.reports if 'phase' in r]
    assert phases == ['thinking', 'calling_skill', 'thinking', 'idle']
    assert any(r.get('skill') == 'turn' for r in robot.reports)
    s = conv.status.fields
    assert s['last_reply'] == 'Turned left.' and s['last_skill'].startswith('turn(degrees=90)')
    assert s['tool_calls'] == 1 and s['session_turns'] == 1
    assert (s['prompt_tokens'], s['completion_tokens']) == (1850, 25)
    assert all('uptime' in r for r in robot.reports)
    conv.new_session()
    assert conv.status.fields['session_turns'] == 0
