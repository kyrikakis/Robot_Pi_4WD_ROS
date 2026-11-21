#!/usr/bin/env bash
# Install Rainbow's Mac agent (docs/brain.md) into one folder, $RAINBOW_AGENT_HOME
# (default ~/.rainbow-agent). Nothing goes anywhere else: uv's cache, Python
# itself, the model files and the agent's data all live under that folder.
# agent/uninstall.sh removes it again. Safe to rerun: finished downloads are
# kept, interrupted ones resume, and every file is checked against its SHA-256.
#
# Needs: macOS on Apple Silicon, uv (https://docs.astral.sh/uv/), curl.

set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="${RAINBOW_AGENT_HOME:-$HOME/.rainbow-agent}"

# Pins. Change them together with a test run (rainbow-agent eval).
LLAMA_TAG=b12123
LLAMA_SHA256=4273bf18f29f51ca7849d0993bcf01245bbaeef5caeced1050e634475ba8cbca
QWEN_VL_REPO=Qwen/Qwen3-VL-30B-A3B-Instruct-GGUF
QWEN_VL_REV=f54435e6cc31258f04b0969105c3f6badb197931
EMBED_REPO=Qwen/Qwen3-Embedding-0.6B-GGUF
EMBED_REV=370f27d7550e0def9b39c1f16d3fbaa13aa67728
WHISPER_REPO=mlx-community/whisper-large-v3-turbo
WHISPER_REV=a4aaeec0636e6fef84abdcbe3544cb2bf7e9f6fb

# Keep uv's Python and cache inside ROOT too
export UV_CACHE_DIR="$ROOT/uv-cache"
export UV_PYTHON_INSTALL_DIR="$ROOT/python"
export UV_PROJECT_ENVIRONMENT="$ROOT/venv"

[[ "$(uname -s)/$(uname -m)" == Darwin/arm64 ]] || { echo "needs macOS on Apple Silicon"; exit 1; }
command -v uv >/dev/null || { echo "needs uv: https://docs.astral.sh/uv/"; exit 1; }
mkdir -p "$ROOT"/{bin,models,data,logs}

# fetch URL DEST SHA256: download (resuming) and verify
fetch() {
    local url="$1" dest="$2" sha="$3"
    if [[ -f "$dest" ]] && [[ "$(shasum -a 256 "$dest" | cut -d' ' -f1)" == "$sha" ]]; then
        echo "ok      $(basename "$dest")"
        return
    fi
    echo "fetch   $(basename "$dest")"
    curl -fL --retry 3 -C - -o "$dest.part" "$url"
    local got
    got="$(shasum -a 256 "$dest.part" | cut -d' ' -f1)"
    if [[ "$got" != "$sha" ]]; then
        rm -f "$dest.part"
        echo "checksum mismatch for $dest (got $got)"; exit 1
    fi
    mv "$dest.part" "$dest"
}

hf() { echo "https://huggingface.co/$1/resolve/$2/$3"; }

# llama.cpp server (prebuilt, Metal)
if [[ ! -x "$ROOT/bin/llama-$LLAMA_TAG/llama-server" ]]; then
    fetch "https://github.com/ggml-org/llama.cpp/releases/download/$LLAMA_TAG/llama-$LLAMA_TAG-bin-macos-arm64.tar.gz" \
        "$ROOT/bin/llama-$LLAMA_TAG.tar.gz" "$LLAMA_SHA256"
    mkdir -p "$ROOT/bin/llama-$LLAMA_TAG"
    tar -xzf "$ROOT/bin/llama-$LLAMA_TAG.tar.gz" -C "$ROOT/bin/llama-$LLAMA_TAG" --strip-components 1
    rm "$ROOT/bin/llama-$LLAMA_TAG.tar.gz"
fi
ln -sfn "llama-$LLAMA_TAG" "$ROOT/bin/llama"

# Models
fetch "$(hf $QWEN_VL_REPO $QWEN_VL_REV Qwen3VL-30B-A3B-Instruct-Q4_K_M.gguf)" \
    "$ROOT/models/Qwen3VL-30B-A3B-Instruct-Q4_K_M.gguf" \
    87bb374d849f80ebdfabb304189fac9e0bd35a0f74506e6a59c51b206cbe863b
fetch "$(hf $QWEN_VL_REPO $QWEN_VL_REV mmproj-Qwen3VL-30B-A3B-Instruct-F16.gguf)" \
    "$ROOT/models/mmproj-Qwen3VL-30B-A3B-Instruct-F16.gguf" \
    cae72cf123cc9e08d553cd5a5055d6d3cf0f82652aa41c3e4aa424cda9a26f7f
fetch "$(hf $EMBED_REPO $EMBED_REV Qwen3-Embedding-0.6B-Q8_0.gguf)" \
    "$ROOT/models/Qwen3-Embedding-0.6B-Q8_0.gguf" \
    06507c7b42688469c4e7298b0a1e16deff06caf291cf0a5b278c308249c3e439
mkdir -p "$ROOT/models/whisper-large-v3-turbo"
fetch "$(hf $WHISPER_REPO $WHISPER_REV weights.safetensors)" \
    "$ROOT/models/whisper-large-v3-turbo/weights.safetensors" \
    951ed3fc1203e6a62467abb2144a96ce7eafca8fa77e3704fdb8635ff3e7f8a6
curl -fsSL -o "$ROOT/models/whisper-large-v3-turbo/config.json" \
    "$(hf $WHISPER_REPO $WHISPER_REV config.json)"

# The agent: Python 3.12 and the locked dependencies, the code itself stays
# in the repo (editable install, so changes take effect without reinstalling)
uv sync --project "$HERE" --frozen --python 3.12 --managed-python

echo
echo "Installed in $ROOT ($(du -sh "$ROOT" | cut -f1))."
echo "Run: $ROOT/venv/bin/rainbow-agent --help"
