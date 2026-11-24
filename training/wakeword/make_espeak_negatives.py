"""Negative features of the robot's own voice (espeak-ng), for every model.

Mixing espeak into the background of the training clips alone taught "hey rainbow" r2 that
espeak means the wake word (462 false accepts per hour on espeak): the positives carry
background speech more often than the negatives. This makes espeak a negative class of its
own: random sentences in varied espeak voices, cut into 2 s windows, augmented (room
impulse responses, AudioSet noise), as openWakeWord features in
/data/features/espeak_negative_features.npy for the configs' feature_data_files.

The text differs from the eval set's (prepare_data.ESPEAK_SENTENCES), so the eval stays fair.
"""

import random
import subprocess
from pathlib import Path

import numpy as np
import pronouncing
import soundfile as sf
from openwakeword.data import augment_clips
from openwakeword.utils import compute_features_from_generator
from scipy.signal import resample_poly

OUT = Path('/data/espeak_negative')
FEATURES = Path('/data/features/espeak_negative_features.npy')
N_SENTENCES = 1500
WINDOW = 32000   # 2 s, the clip length of the models
VOICES = ['en', 'en-us', 'en-gb', 'en-gb-scotland', 'en-029', 'en+f2', 'en+f3', 'en+f4',
          'en+m1', 'en+m3', 'en+m5', 'en-us+f2', 'en-us+m2', 'en+klatt', 'en+whisper']
TEMPLATES = [
    "I am going to the {w}.", "Did you say {w}?", "The {w} is next to the {w}.",
    "I can see a {w} and a {w}.", "Let me check the {w} for you.", "Okay, {w}.",
    "I don't know what {w} means.", "Turning towards the {w} now.", "Hello {w}, how are you?",
    "There is a {w} in front of me, I will go around it.", "My battery is at {n} percent.",
    "It is {n} degrees.", "I found {n} {w}s in the {w}.", "Shall I {w} the {w}?",
]


def sentence(rng, words):
    if rng.random() < 0.5:
        return ' '.join(rng.choice(words) for _ in range(rng.randint(5, 14))) + '.'
    t = rng.choice(TEMPLATES)
    while '{w}' in t or '{n}' in t:
        t = t.replace('{w}', rng.choice(words), 1).replace('{n}', str(rng.randint(1, 99)), 1)
    return t


def main():
    rng = random.Random(42)
    pronouncing.init_cmu()
    words = [w for w, _ in pronouncing.pronunciations if w.isalpha() and len(w) > 2
             and not w.startswith(('rain', 'stop', 'robot'))]
    OUT.mkdir(parents=True, exist_ok=True)
    if len(list(OUT.glob('*.wav'))) < 3000:
        for i in range(N_SENTENCES):
            text = ' '.join(sentence(rng, words) for _ in range(rng.randint(1, 3)))
            tmp = OUT / 'tmp.wav'
            subprocess.run(['espeak-ng', '-v', rng.choice(VOICES), '-s', str(rng.randint(130, 200)),
                            '-p', str(rng.randint(30, 70)), '-w', str(tmp), text], check=True)
            audio, sr = sf.read(tmp, dtype='float32')
            audio = resample_poly(audio, 16000 // np.gcd(sr, 16000), sr // np.gcd(sr, 16000))
            audio = (np.clip(audio, -1, 1) * 32767).astype(np.int16)
            for j, start in enumerate(range(0, len(audio) - WINDOW // 2, WINDOW)):
                chunk = audio[start:start + WINDOW]
                if np.abs(chunk).max() > 1000:   # skip near silence
                    sf.write(OUT / f'{i:04d}_{j}.wav', chunk, 16000, subtype='PCM_16')
        (OUT / 'tmp.wav').unlink(missing_ok=True)
    clips = sorted(str(p) for p in OUT.glob('*.wav')) * 2   # two augmentation rounds
    rng.shuffle(clips)
    rirs = [str(p) for p in Path('/data/rirs').glob('*.wav')]
    background = [str(p) for p in Path('/data/background').glob('*.wav')]
    generator = augment_clips(clips, total_length=WINDOW, batch_size=16,
                              background_clip_paths=background, RIR_paths=rirs)
    compute_features_from_generator(generator, n_total=len(clips), clip_duration=WINDOW,
                                    output_file=str(FEATURES), device='cpu', ncpu=8)
    print('features', np.load(FEATURES, mmap_mode='r').shape)


if __name__ == '__main__':
    main()
