"""rainbow-agent: talk to Rainbow from the Mac, and test the model.

  rainbow-agent chat            type to Rainbow (simulated robot for now)
  rainbow-agent transcribe WAV  Whisper on a file, with timing
  rainbow-agent servers         run the model servers in the foreground
  rainbow-agent eval            score the model on agent/eval/cases.yaml

chat and eval start the model servers if they are not running, and stop them
again on exit. In chat, "/hear FILE.wav" transcribes a recording and says it.
"""

import argparse
import asyncio
import contextlib
from pathlib import Path
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


def show_tool(name, args, result):
    args_text = ', '.join(f'{k}={v!r}' for k, v in (args or {}).items())
    status = result.get('message') or ('ok' if result.get('ok') else 'failed')
    print(f'{DIM}  [robot] {name}({args_text}) -> {status}{RESET}')


async def chat(cfg):
    robot = SimRobot()
    conversation = Conversation(llm_client(cfg), cfg.chat_alias, robot, cfg, show_tool)
    whisper = None

    async def events():
        while True:
            event = await robot.events.get()
            print(f'\n{DIM}  [robot event] {event}{RESET}')
            reply = await conversation.robot_event(event)
            if reply:
                print(f'{BOLD}rainbow>{RESET} {reply}')
            print('you> ', end='', flush=True)

    event_task = asyncio.ensure_future(events())
    print('Talk to Rainbow (simulated robot). Ctrl-D to quit, /new for a new session, '
          '/hear FILE.wav to speak a recording.')
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
            print(f'{BOLD}rainbow>{RESET} {reply}  {DIM}({time.monotonic() - start:.1f} s){RESET}')
    finally:
        event_task.cancel()


def transcribe(cfg, path):
    from rainbow_agent.stt import Whisper
    whisper = Whisper(cfg.whisper_model)
    for i in range(2):  # the first run includes loading the model
        text, seconds = whisper.transcribe(str(path))
        print(f'{"first" if i == 0 else "again"}: {seconds:.2f} s  {text}')


def model_options(parser):
    parser.add_argument('--model', type=Path, help='another chat model GGUF (to compare)')
    parser.add_argument('--mmproj', type=Path, help='its vision projector GGUF')


def main(argv=None):
    parser = argparse.ArgumentParser(prog='rainbow-agent', description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='command', required=True)
    p = sub.add_parser('chat', help='type to Rainbow (simulated robot)')
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
    if getattr(args, 'model', None):
        cfg.chat_model = args.model
        cfg.vision_projector = args.mmproj or cfg.vision_projector

    try:
        if args.command == 'chat':
            with servers(cfg):
                asyncio.run(chat(cfg))
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
