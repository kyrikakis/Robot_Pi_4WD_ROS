#!/usr/bin/env bash
# Remove Rainbow's Mac agent completely: the launchd agent (if installed), the
# ~/.local/bin/rainbow-agent link (only if it points into the install) and
# $RAINBOW_AGENT_HOME (default ~/.rainbow-agent), which holds everything else:
# models, Python, uv's cache, memory and logs. The code in the repo stays.

set -euo pipefail

ROOT="${RAINBOW_AGENT_HOME:-$HOME/.rainbow-agent}"
PLIST="$HOME/Library/LaunchAgents/ai.rainbow.agent.plist"
LINK="$HOME/.local/bin/rainbow-agent"

if [[ -L "$LINK" && "$(readlink "$LINK")" == "$ROOT/"* ]]; then
    rm "$LINK"
    echo "removed $LINK"
fi

if [[ -f "$PLIST" ]]; then
    launchctl bootout "gui/$(id -u)" "$PLIST" 2>/dev/null || true
    rm "$PLIST"
    echo "removed $PLIST"
fi
if [[ -d "$ROOT" ]]; then
    echo "removing $ROOT ($(du -sh "$ROOT" | cut -f1)), including the agent's memory"
    read -r -p "continue? [y/N] " answer
    [[ "$answer" == y ]] || { echo "kept"; exit 0; }
    rm -rf "$ROOT"
    echo "removed $ROOT"
fi
rm -rf "$(cd "$(dirname "$0")" && pwd)/.venv"
