"""rainbow-agent: talk to Rainbow from the Mac, and test the model.

  rainbow-agent chat            type to Rainbow (simulated robot)
  rainbow-agent chat --robot    type to the real robot (it speaks the replies)
  rainbow-agent pair            fetch the link token from the robot (once)
  rainbow-agent run             the agent for real: listen, think, answer (always on)
  rainbow-agent transcribe WAV  Whisper on a file, with timing
  rainbow-agent servers         run the model servers in the foreground
  rainbow-agent eval            score the model on agent/eval/cases.yaml

chat and eval start the model servers if they are not running, and stop them
again on exit. In chat, "/hear FILE.wav" transcribes a recording and says it.
"""

import argparse
import asyncio
import base64
import contextlib
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from rainbow_agent.config import Config
from rainbow_agent.conversation import Conversation
from rainbow_agent.robot import SimRobot
from rainbow_agent.servers import chat_server, embed_server

DIM, BOLD, RESET = '\033[2m', '\033[1m', '\033[0m'


@contextlib.contextmanager
def servers(cfg, embed=False):
    started = [chat_server(cfg)] + ([embed_server(cfg)] if embed else [])
    try:
        for s in started:
            print(f'{DIM}starting {s.name} model server (port {s.port})...{RESET}',
                  flush=True)
            s.start()
        yield
    finally:
        for s in started:
            s.stop()


def llm_client(cfg):
    from openai import AsyncOpenAI
    return AsyncOpenAI(base_url=cfg.chat_url, api_key='local')


def open_memory(cfg):
    """The long-term memory in <root>/data/memory.db (needs the embeddings server)."""
    from openai import AsyncOpenAI
    from rainbow_agent.memory import llama_embedder, Memory
    cfg.data.mkdir(parents=True, exist_ok=True)
    embedder = llama_embedder(AsyncOpenAI(base_url=cfg.embed_url, api_key='local'),
                              cfg.embed_alias)
    return Memory(cfg.data / 'memory.db', llm_client(cfg), cfg.chat_alias, embedder)


def log_line(text):
    print(f'{DIM}  [memory] {text}{RESET}')


async def idle_sessions(conversation):
    while True:
        await asyncio.sleep(30)
        conversation.close_if_idle()


def show_tool(name, args, result):
    args_text = ', '.join(f'{k}={v!r}' for k, v in (args or {}).items())
    status = result.get('message') or ('ok' if result.get('ok') else 'failed')
    print(f'{DIM}  [robot] {name}({args_text}) -> {status}{RESET}')


def make_robot(cfg, real):
    if not real:
        return SimRobot()
    from rainbow_agent.link import LinkRobot
    if not cfg.link_token.exists():
        raise RuntimeError(f'no link token in {cfg.link_token}: run `rainbow-agent pair`')
    log = (lambda m: print(f'{DIM}  [link] {m}{RESET}'))
    return LinkRobot(cfg.robot_hosts, cfg.robot_port, cfg.link_token.read_text().strip(), log)


async def chat(cfg, real=False):
    robot = make_robot(cfg, real)
    if real:
        await robot.start()
        if not await robot.wait_connected(10.0):
            print('could not reach the robot yet; still trying in the background')

    async def speak(reply):
        print(f'{BOLD}rainbow>{RESET} {reply}', end='')
        if real:
            await robot.say(reply)

    memory = open_memory(cfg)
    conversation = Conversation(llm_client(cfg), cfg.chat_alias, robot, cfg, show_tool,
                                memory=memory, log=log_line)
    whisper = None

    async def events():
        while True:
            event = await robot.events.get()
            print(f'\n{DIM}  [robot event] {event}{RESET}')
            reply = await conversation.robot_event(event)
            if reply:
                await speak(reply)
                print()
            print('you> ', end='', flush=True)

    event_task = asyncio.ensure_future(events())
    idle_task = asyncio.ensure_future(idle_sessions(conversation))
    print(f'Talk to Rainbow ({"the real robot" if real else "simulated robot"}). Ctrl-D to '
          'quit, /new for a new session, /hear FILE.wav to speak a recording.')
    try:
        while True:
            try:
                text = (await asyncio.to_thread(input, 'you> ')).strip()
            except EOFError:
                print()
                break
            if not text:
                continue
            if text == '/new':
                conversation.new_session()
                continue
            if text.startswith('/hear '):
                if whisper is None:
                    from rainbow_agent.stt import Whisper
                    whisper = Whisper(cfg.whisper_model)
                text, seconds = await asyncio.to_thread(
                    whisper.transcribe, text[6:].strip())
                print(f'{DIM}  [heard in {seconds:.2f} s] {text}{RESET}')
            start = time.monotonic()
            reply = await conversation.user_says(text)
            await speak(reply)
            print(f'  {DIM}({time.monotonic() - start:.1f} s){RESET}')
    finally:
        event_task.cancel()
        idle_task.cancel()
        await conversation.close()
        memory.close()
        if real:
            await robot.close()


async def run_agent(cfg):
    """Listen to the robot: utterances -> Whisper -> conversation -> spoken reply."""
    from rainbow_agent.stt import is_speech, pcm16_to_float, Whisper
    whisper = Whisper(cfg.whisper_model)
    await asyncio.to_thread(whisper.transcribe, pcm16_to_float(bytes(32000)))  # load it
    robot = make_robot(cfg, real=True)
    memory = open_memory(cfg)
    conversation = Conversation(llm_client(cfg), cfg.chat_alias, robot, cfg, show_tool,
                                memory=memory, log=log_line)
    await robot.start()
    print('agent running; Ctrl-C to stop')

    async def events():
        while True:
            event = await robot.events.get()
            print(f'{DIM}  [robot event] {event}{RESET}')
            reply = await conversation.robot_event(event)
            if reply:
                print(f'{BOLD}rainbow>{RESET} {reply}')
                await robot.say(reply)

    async def utterances():
        while True:
            u = await robot.utterances.get()
            if u.get('handled'):
                print(f'{DIM}  [handled on the robot] {u.get("text")!r}: {u.get("action")}{RESET}')
                await conversation.handled_locally(u.get('text', ''), u.get('action', ''))
                continue
            audio = pcm16_to_float(base64.b64decode(u.get('audio', '')))
            text, seconds = await asyncio.to_thread(whisper.transcribe, audio)
            if not is_speech(text):
                print(f'{DIM}  [ignored: {text!r}]{RESET}')
                continue
            print(f'you> {text}  {DIM}(heard in {seconds:.2f} s){RESET}')
            start = time.monotonic()
            reply = await conversation.user_says(text)
            print(f'{BOLD}rainbow>{RESET} {reply}  {DIM}({time.monotonic() - start:.1f} s){RESET}')
            await robot.say(reply)

    try:
        await asyncio.gather(events(), utterances(), idle_sessions(conversation))
    finally:
        await conversation.close()
        memory.close()
        await robot.close()


def pair(cfg):
    """Copy the brain's link token from the robot (over SSH) to the Mac."""
    cmd = ['ssh', cfg.robot_ssh, 'docker exec robot cat /data/rainbow/link_token']
    try:
        out = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
    except subprocess.TimeoutExpired:
        raise RuntimeError(f'no answer from {cfg.robot_ssh} over SSH')
    token = out.stdout.strip()
    if out.returncode != 0 or len(token) < 16:
        raise RuntimeError(f'could not read the token: {out.stderr.strip() or "empty"} '
                           '(is the robot running a brain with the link?)')
    cfg.link_token.parent.mkdir(parents=True, exist_ok=True)
    fd = os.open(cfg.link_token, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    with os.fdopen(fd, 'w') as f:
        f.write(token + '\n')
    print(f'paired: token saved in {cfg.link_token}')


def transcribe(cfg, path):
    from rainbow_agent.stt import Whisper
    whisper = Whisper(cfg.whisper_model)
    for i in range(2):  # the first run includes loading the model
        text, seconds = whisper.transcribe(str(path))
        print(f'{"first" if i == 0 else "again"}: {seconds:.2f} s  {text}')


def robot_options(parser):
    parser.add_argument('--robot-host', help='the robot (default: rainbow.local, then the IP)')
    parser.add_argument('--robot-port', type=int, help='its link port (default 8766)')
    parser.add_argument('--token-file', type=Path, help='the link token (default: paired one)')


def model_options(parser):
    parser.add_argument('--model', type=Path, help='another chat model GGUF (to compare)')
    parser.add_argument('--mmproj', type=Path, help='its vision projector GGUF')


def main(argv=None):
    parser = argparse.ArgumentParser(prog='rainbow-agent', description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='command', required=True)
    p = sub.add_parser('chat', help='type to Rainbow (simulated robot)')
    p.add_argument('--robot', action='store_true', help='the real robot, over the link')
    robot_options(p)
    model_options(p)
    sub.add_parser('pair', help='fetch the link token from the robot over SSH')
    p = sub.add_parser('run', help='the agent for real: listen to the robot and answer')
    robot_options(p)
    model_options(p)
    p = sub.add_parser('transcribe', help='Whisper on a WAV file')
    p.add_argument('wav', type=Path)
    sub.add_parser('servers', help='run the model servers until Ctrl-C')
    p = sub.add_parser('eval', help='score the model on the eval cases')
    p.add_argument('--cases', type=Path,
                   default=Path(__file__).resolve().parent.parent / 'eval' / 'cases.yaml')
    p.add_argument('--only', help='run only cases whose id contains this')
    model_options(p)
    args = parser.parse_args(argv)
    cfg = Config()
    # launchd (and kill) stop with SIGTERM: shut down like Ctrl-C, model servers included
    signal.signal(signal.SIGTERM, signal.default_int_handler)
    if getattr(args, 'robot_host', None):
        cfg.robot_hosts = [args.robot_host]
    if getattr(args, 'robot_port', None):
        cfg.robot_port = args.robot_port
    if getattr(args, 'token_file', None):
        cfg.link_token = args.token_file
    if getattr(args, 'model', None):
        cfg.chat_model = args.model
        cfg.vision_projector = args.mmproj or cfg.vision_projector

    try:
        if args.command == 'chat':
            with servers(cfg, embed=True):
                asyncio.run(chat(cfg, real=args.robot))
        elif args.command == 'pair':
            pair(cfg)
        elif args.command == 'run':
            with servers(cfg, embed=True):
                asyncio.run(run_agent(cfg))
        elif args.command == 'transcribe':
            transcribe(cfg, args.wav)
        elif args.command == 'servers':
            with servers(cfg, embed=True):
                print('model servers running; Ctrl-C to stop')
                while True:
                    time.sleep(3600)
        elif args.command == 'eval':
            from rainbow_agent.evaluate import run
            with servers(cfg):
                ok = asyncio.run(run(cfg, llm_client(cfg), args.cases, args.only))
            sys.exit(0 if ok else 1)
    except KeyboardInterrupt:
        print()
    except RuntimeError as e:
        sys.exit(f'error: {e}')
