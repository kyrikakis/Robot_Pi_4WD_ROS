# rainbow-agent

Rainbow's agent on the Mac (design: [docs/brain.md](../docs/brain.md)): Whisper
for speech to text, Qwen3-VL with the robot's skills as tools, and later memory
and the link to the robot. For now it talks to a simulated robot.

Needs a Mac with Apple Silicon (built for an M4 Max with 64 GB) and
[uv](https://docs.astral.sh/uv/).

## Install and remove

```
agent/install.sh      # ~21 GB, all into ~/.rainbow-agent
agent/uninstall.sh    # removes ~/.rainbow-agent (asks first: it holds the memory)
```

Everything goes into `~/.rainbow-agent` (or `$RAINBOW_AGENT_HOME`): llama.cpp's
server, the models, Python, the locked dependencies, uv's cache, data and logs.
Every download is pinned and checked against its SHA-256; to update one,
change its pin in `install.sh` and rerun `rainbow-agent eval`.

## Use

```
~/.rainbow-agent/venv/bin/rainbow-agent chat              # type to Rainbow
~/.rainbow-agent/venv/bin/rainbow-agent transcribe a.wav  # Whisper, with timing
~/.rainbow-agent/venv/bin/rainbow-agent eval              # score the model (eval/cases.yaml)
```

`chat` and `eval` start the model servers (llama-server on 127.0.0.1:8180,
embeddings on :8181) and stop them on exit; `rainbow-agent servers` keeps them
running. Their logs are in `~/.rainbow-agent/logs/`.

## Develop

```
cd agent
~/.rainbow-agent/venv/bin/python -m pytest      # no models needed
~/.rainbow-agent/venv/bin/flake8
```

The code is installed editable: changes take effect without reinstalling. After
changing dependencies: `uv lock`, then rerun `install.sh`.
