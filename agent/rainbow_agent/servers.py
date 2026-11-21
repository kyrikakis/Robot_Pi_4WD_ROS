"""The two llama-server processes: the chat model (with vision) and embeddings.

Both run on the GPU (Metal) and listen on localhost only. Commands that need
them start them if they are not already running, and stop what they started.
Logs: <root>/logs/<name>.log.
"""

import subprocess
import time
import urllib.error
import urllib.request


def is_up(port):
    try:
        with urllib.request.urlopen(f'http://127.0.0.1:{port}/health', timeout=2) as r:
            return r.status == 200
    except (urllib.error.URLError, OSError):
        return False


class LlamaServer:

    def __init__(self, name, binary, port, args, logs):
        self.name = name
        self.port = port
        self.cmd = [str(binary), '--host', '127.0.0.1', '--port', str(port), *map(str, args)]
        self.log_path = logs / f'{name}.log'
        self.process = None

    def start(self, timeout=300.0):
        """Start unless something already answers on the port. Returns once healthy."""
        if is_up(self.port):
            return
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        log = open(self.log_path, 'ab')
        self.process = subprocess.Popen(self.cmd, stdout=log, stderr=subprocess.STDOUT)
        deadline = time.monotonic() + timeout
        while not is_up(self.port):
            if self.process.poll() is not None:
                raise RuntimeError(f'{self.name} exited; see {self.log_path}')
            if time.monotonic() > deadline:
                self.stop()
                raise RuntimeError(f'{self.name} did not start in {timeout:.0f} s; '
                                   f'see {self.log_path}')
            time.sleep(0.5)

    def stop(self):
        if self.process is not None and self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.process.kill()
        self.process = None


def chat_server(cfg):
    return LlamaServer('chat', cfg.llama_server, cfg.chat_port, [
        '-m', cfg.chat_model, '--mmproj', cfg.vision_projector,
        '--alias', cfg.chat_alias, '--jinja', '-c', cfg.context, '-ngl', 999,
    ], cfg.logs)


def embed_server(cfg):
    return LlamaServer('embed', cfg.llama_server, cfg.embed_port, [
        '-m', cfg.embed_model, '--alias', cfg.embed_alias,
        '--embedding', '--pooling', 'last', '-ngl', 999,
    ], cfg.logs)
