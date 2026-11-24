"""Keep only the accented clips Whisper hears as the phrase (runs on the Mac, on MLX).

Uses the agent's Whisper (large-v3-turbo) from its venv:

  ~/.rainbow-agent/venv/bin/python whisper_filter.py <accent_pool folder>

For every <model>/<split>/ directory it writes accepted.txt and rejected.txt (file name,
tab, transcript); files already listed are skipped, so it can be stopped and rerun.
"""

import collections
import os
import re
import sys
import wave
from pathlib import Path

import mlx_whisper
import numpy as np

MODEL = os.path.expanduser('~/.rainbow-agent/models/whisper-large-v3-turbo')
RULES = {
    # something before "rainbow" ("hey", or how an accent or Whisper renders it), short
    'hey_rainbow': re.compile(r"^\W*\w+[\s,!.'-]+rain ?bow\W*$"),
    'stop': re.compile(r'^\W*stop\W*$'),
    'hey_robot': re.compile(r"^\W*\w+[\s,!.'-]+robot\W*$"),
}


def transcribe(path):
    with wave.open(str(path)) as w:
        audio = np.frombuffer(w.readframes(w.getnframes()), np.int16).astype(np.float32) / 32768
    return mlx_whisper.transcribe(audio, path_or_hf_repo=MODEL, language='en',
                                  condition_on_previous_text=False)['text'].strip()


def main(root):
    for model_dir in sorted(Path(root).iterdir()):
        rule = RULES.get(model_dir.name)
        if rule is None:
            continue
        for split_dir in sorted(p for p in model_dir.iterdir() if p.is_dir()):
            done = set()
            for name in ('accepted.txt', 'rejected.txt'):
                f = split_dir / name
                if f.exists():
                    done |= {line.split('\t')[0] for line in f.read_text().splitlines()}
            todo = sorted(p for p in split_dir.glob('*.wav') if p.name not in done)
            stats = collections.Counter()
            with open(split_dir / 'accepted.txt', 'a') as acc, \
                    open(split_dir / 'rejected.txt', 'a') as rej:
                for i, path in enumerate(todo):
                    text = transcribe(path)
                    ok = bool(rule.match(text.lower()))
                    (acc if ok else rej).write(f'{path.name}\t{text}\n')
                    stats[(path.name.split('__')[0], ok)] += 1
                    if i % 200 == 199:
                        acc.flush()
                        rej.flush()
                        print(f'{model_dir.name}/{split_dir.name}: {i + 1}/{len(todo)}', flush=True)
            voices = sorted({v for v, _ in stats})
            print(f'== {model_dir.name}/{split_dir.name}: accepted by voice: ' + ', '.join(
                f'{v} {stats[(v, True)]}/{stats[(v, True)] + stats[(v, False)]}' for v in voices),
                flush=True)


if __name__ == '__main__':
    main(sys.argv[1])
