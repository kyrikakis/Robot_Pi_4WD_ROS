"""Generate accented positive clips into /pool/<model>/<split>/, to be checked by Whisper.

Piper's non-US voices often mangle the phrase (the Spanish, Italian, Greek and Hindi
voices reading English especially), so accented positives are not used straight from
the synthesizer: whisper_filter.py (on the Mac) keeps only the clips Whisper hears as
the phrase, and generate_samples.py draws accented positives from those.

  make_accent_pool.py "hey rainbow" --train 6000 --test 1200
  make_accent_pool.py "hey rainbow" --greek --train 4000 --test 800   (Greek accent)
"""

import argparse
import random
import sys
from pathlib import Path

sys.path.insert(0, '/src')
import generate_samples as g  # noqa: E402

if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('phrase')
    ap.add_argument('--train', type=int, default=6000)
    ap.add_argument('--test', type=int, default=1200)
    ap.add_argument('--greek', action='store_true', help='Greek-accented LibriTTS clips')
    a = ap.parse_args()
    model = a.phrase.replace(' ', '_')
    for split, n in (('train', a.train), ('test', a.test)):
        out = Path('/pool') / model / split
        out.mkdir(parents=True, exist_ok=True)
        have = len([p for p in out.glob('*.wav') if p.name.startswith('greek__') == a.greek])
        random.seed(hash((model, split, have)))
        while have < n:
            if a.greek:
                items = g.greek_batch(a.phrase, min(16, n - have), split == 'test')
            else:
                items = [g._voice_clip(a.phrase, split == 'test', (0.8, 1.0, 1.2))]
            for item in items:
                if item is None or len(item[1]) < 1600:
                    continue
                g._write(out / f'{item[0]}__{random.getrandbits(64):016x}.wav', item[1])
                have += 1
                if have % 500 == 0:
                    print(f'{model}/{split}: {have}/{n}', flush=True)
