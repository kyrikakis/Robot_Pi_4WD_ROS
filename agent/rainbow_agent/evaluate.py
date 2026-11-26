"""Score the model on agent/eval/cases.yaml: does it call the right skills?

Each case runs in a fresh conversation with a fresh SimRobot (moves are
instant). A case passes when:
  expect   these calls happen, in this order (others may happen in between);
           args match loosely: strings by substring, numbers within 20% (or a
           {min, max} range), {abs: n} compares absolute values
  forbid   none of these skills is called ("any" = no skill at all)
  reply_not  none of these words is in a reply (invented details)
  reply_has  each of these words is in some reply
A case can start with a robot event (event: {kind: face_unknown}) and set
SimRobot attributes first (robot: {face_in_view: Bambu}).
An expected arg of {empty: true} must be absent or empty (not invented).
Every reply is also checked for being speakable: short, no markdown or emoji.
"""

import asyncio
import re
import statistics
import time

from rainbow_agent.conversation import Conversation
from rainbow_agent.robot import SimRobot
import yaml

DIM, RED, GREEN, RESET = '\033[2m', '\033[31m', '\033[32m', '\033[0m'


def value_matches(expected, actual):
    if isinstance(expected, dict):
        try:
            a = float(actual)
        except (TypeError, ValueError):
            return False
        if 'abs' in expected:
            return value_matches(expected['abs'], abs(a))
        return expected.get('min', -1e9) <= a <= expected.get('max', 1e9)
    if isinstance(expected, bool):
        return actual is expected
    if isinstance(expected, (int, float)):
        try:
            a = float(actual)
        except (TypeError, ValueError):
            return False
        return abs(a - expected) <= max(0.2 * abs(expected), 1e-6)
    return str(expected).lower() in str(actual).lower()


def arg_matches(key, expected, args):
    if isinstance(expected, dict) and expected.get('empty'):
        return not args.get(key)
    return key in args and value_matches(expected, args[key])


def call_matches(expected, call):
    name, args = call
    if name != expected['name']:
        return False
    return all(arg_matches(k, v, args) for k, v in (expected.get('args') or {}).items())


def check(case, calls, replies):
    """Returns a list of problems (empty = pass)."""
    problems = []
    i = 0
    for exp in case.get('expect') or []:
        while i < len(calls) and not call_matches(exp, calls[i]):
            i += 1
        if i == len(calls):
            problems.append(f'missing {exp["name"]}({exp.get("args") or ""})')
            break
        i += 1
    forbid = case.get('forbid') or []
    for name, _ in calls:
        if 'any' in forbid or name in forbid:
            problems.append(f'forbidden call {name}')
    for word in case.get('reply_has') or []:
        if not any(word.lower() in r.lower() for r in replies):
            problems.append(f'no reply says "{word}"')
    for word in case.get('reply_not') or []:
        if any(word.lower() in r.lower() for r in replies):
            problems.append(f'reply says "{word}" (invented)')
    for reply in replies:
        if not reply:
            problems.append('empty reply')
        elif len(reply.split()) > 60:
            problems.append(f'reply too long ({len(reply.split())} words)')
        elif re.search(r'(^|\n)\s*([-*#]|\d+\.)\s|\*\*|[\U0001F300-\U0001FAFF]', reply):
            problems.append('reply not speakable (markdown or emoji)')
    return problems


async def run(cfg, llm, cases_path, only=None):
    cases = yaml.safe_load(open(cases_path))
    if only:
        cases = [c for c in cases if only in c['id']]
    results, times = [], []
    for case in cases:
        robot = SimRobot(time_scale=0.0)
        for key, value in (case.get('robot') or {}).items():
            setattr(robot, key, value)      # e.g. face_in_view: Bambu
        conversation = Conversation(llm, cfg.chat_alias, robot, cfg)
        says = case['say'] if isinstance(case['say'], list) else [case['say']]
        replies = []
        start = time.monotonic()
        if case.get('event'):               # a robot event first, e.g. {kind: face_unknown}
            replies.append(await conversation.robot_event(case['event']) or '')
        for text in says:
            t = time.monotonic()
            replies.append(await conversation.user_says(text))
            times.append(time.monotonic() - t)
            # Then the robot's events, as the CLI delivers them (a failed trip gets a turn)
            for _ in range(5):
                await asyncio.sleep(0)
            while not robot.events.empty():
                reply = await conversation.robot_event(robot.events.get_nowait())
                if reply:
                    replies.append(reply)
        elapsed = time.monotonic() - start
        problems = check(case, robot.calls, replies)
        results.append(not problems)
        mark = f'{GREEN}pass{RESET}' if not problems else f'{RED}FAIL{RESET}'
        calls = ', '.join(f'{n}({", ".join(f"{k}={v!r}" for k, v in a.items())})'
                          for n, a in robot.calls) or '-'
        print(f'{mark} {case["id"]:<24} {elapsed:5.1f} s  {calls}')
        print(f'{DIM}     "{says[-1]}" -> {replies[-1]}{RESET}')
        for p in problems:
            print(f'     {RED}{p}{RESET}')
    passed = sum(results)
    print(f'\n{passed}/{len(results)} passed; per turn: median '
          f'{statistics.median(times):.1f} s, max {max(times):.1f} s')
    return passed == len(results)
