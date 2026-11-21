"""Where everything lives and the agent's settings.

All runtime files are under RAINBOW_AGENT_HOME (default ~/.rainbow-agent),
created by agent/install.sh; the code stays in the repo.
"""

from dataclasses import dataclass, field
import os
from pathlib import Path

ROOT = Path(os.environ.get('RAINBOW_AGENT_HOME', Path.home() / '.rainbow-agent'))


@dataclass
class Config:
    root: Path = ROOT
    llama_server: Path = ROOT / 'bin' / 'llama' / 'llama-server'
    chat_model: Path = ROOT / 'models' / 'Qwen3VL-30B-A3B-Instruct-Q4_K_M.gguf'
    vision_projector: Path = ROOT / 'models' / 'mmproj-Qwen3VL-30B-A3B-Instruct-F16.gguf'
    embed_model: Path = ROOT / 'models' / 'Qwen3-Embedding-0.6B-Q8_0.gguf'
    whisper_model: Path = ROOT / 'models' / 'whisper-large-v3-turbo'
    logs: Path = ROOT / 'logs'
    data: Path = ROOT / 'data'

    # llama-server instances, on localhost only
    chat_port: int = 8180
    embed_port: int = 8181
    chat_alias: str = 'qwen3-vl'
    embed_alias: str = 'qwen3-embedding'
    context: int = 16384

    # Conversation
    max_tool_calls: int = 6           # per turn
    session_idle: float = 120.0       # s without a turn: a new session starts
    temperature: float = 0.2
    max_reply_tokens: int = 120

    # The robot: rainbow_brain's link (docs/brain.md)
    robot_hosts: list = field(default_factory=lambda: ['rainbow.local', '192.168.1.172'])
    robot_port: int = 8766
    robot_ssh: str = 'pi@192.168.1.172'      # for `rainbow-agent pair`
    link_token: Path = ROOT / 'data' / 'link_token'

    @property
    def chat_url(self):
        return f'http://127.0.0.1:{self.chat_port}/v1'

    @property
    def embed_url(self):
        return f'http://127.0.0.1:{self.embed_port}/v1'
