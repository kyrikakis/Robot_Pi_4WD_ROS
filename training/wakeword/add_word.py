"""Teach a model the name alone: "rainbow" as well as "hey rainbow".

Generates clips of the word (LibriTTS speakers, Greek pronunciations, the English Piper
voices; test clips from unseen speakers) into /data/runs/<model>/positive_train_word and
positive_test_word, computes their features in ROUNDS augmentation rounds (the first like
train.py's, the others like extra_positives.py's: more room and noise) and appends them to
positive_features_train.npy. Run it after the augmentation (and extra_positives.py), before
training; it does nothing when the word was already added. The negatives must not contain
the word (WW_NEGATIVE_EXCLUDE when generating them, see generate_samples.py).

  add_word.py hey_rainbow rainbow [--n 15000] [--test 1500] [--rounds 3]
"""

import argparse
import os
import random
from pathlib import Path

import numpy as np
import yaml
from openwakeword.data import augment_clips
from openwakeword.utils import compute_features_from_generator

import generate_samples as gs
from extra_positives import PROBABILITIES

LIBRITTS, GREEK = 0.75, 0.15   # the rest: English Piper voices (British, Scottish, non-native)


def english_voice_clip(word, test):
    pool = [(n, s) for n, s in gs._voice_pool(test) if n.startswith('en_')]
    name = random.choice(sorted({n for n, _ in pool}))
    speaker = random.choice([s for n, s in pool if n == name])
    syn = gs.SynthesisConfig(speaker_id=speaker, length_scale=random.uniform(0.75, 1.3),
                             noise_scale=random.uniform(0.5, 0.9),
                             noise_w_scale=random.uniform(0.6, 1.0))
    chunks = list(gs._voices[name].synthesize(word, syn_config=syn))
    audio = np.concatenate([c.audio_float_array for c in chunks])
    return name, gs._to_16k(audio, chunks[0].sample_rate)


def generate(word, n, folder, test):
    folder.mkdir(parents=True, exist_ok=True)
    written = len(list(folder.glob('*.wav')))
    while written < n:
        r = random.random()
        if r < LIBRITTS:
            clips = gs._generator_batch([word] * 32, test, (0.75, 1.0, 1.25), (0.98,), (0.98,))
        elif r < LIBRITTS + GREEK:
            clips = gs.greek_batch(word, 32, test)
        else:
            clips = [english_voice_clip(word, test) for _ in range(8)]
        for source, audio in clips:
            if len(audio) < 1600 or written >= n:
                continue
            gs._write(folder / f'{source}__{os.urandom(8).hex()}.wav', audio)
            written += 1
        print(f'{folder.name}: {written}/{n}', flush=True)


def features(model, clips_dir, rounds, out):
    config = yaml.safe_load(open(f'/src/configs/{model}.yml'))
    background = []
    for path, dup in zip(config['background_paths'], config['background_paths_duplication_rate']):
        background += [str(p) for p in Path(path).glob('*.wav')] * dup
    rirs = [str(p) for d in config['rir_paths'] for p in Path(d).glob('*.wav')]
    clips = [str(p) for p in clips_dir.glob('*.wav')]
    parts = []
    for i in range(rounds):
        part = out.with_suffix(f'.round{i}.npy')
        if not part.exists():
            random.shuffle(clips)
            gen = augment_clips(clips, total_length=32000, batch_size=16,
                                background_clip_paths=background, RIR_paths=rirs,
                                **({} if i == 0 else {'augmentation_probabilities': PROBABILITIES}))
            tmp = part.with_suffix('.partial.npy')
            compute_features_from_generator(gen, n_total=len(clips), clip_duration=32000,
                                            output_file=str(tmp), device='cpu',
                                            ncpu=max(1, (os.cpu_count() or 8) // 2))
            os.replace(tmp, part)
        parts.append(np.load(part, mmap_mode='r'))
    return parts


def append(feats, parts):
    old = np.load(feats, mmap_mode='r')
    n = len(old) + sum(len(p) for p in parts)
    tmp = feats.with_suffix('.tmp.npy')
    out = np.lib.format.open_memmap(tmp, mode='w+', dtype=np.float32, shape=(n,) + old.shape[1:])
    out[:len(old)] = old
    i = len(old)
    for p in parts:
        out[i:i + len(p)] = p
        i += len(p)
    out.flush()
    del out
    os.replace(tmp, feats)
    print(f'{feats.name}: {old.shape} -> {n}', flush=True)


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('model')
    ap.add_argument('word')
    ap.add_argument('--n', type=int, default=15000)
    ap.add_argument('--test', type=int, default=1500)
    ap.add_argument('--rounds', type=int, default=3)
    a = ap.parse_args()
    run = Path('/data/runs') / a.model
    done = run / f'word_{a.word}.done'
    if done.exists():
        print(f'"{a.word}" already added to {a.model}')
    else:
        generate(a.word, a.n, run / 'positive_train_word', test=False)
        generate(a.word, a.test, run / 'positive_test_word', test=True)
        parts = features(a.model, run / 'positive_train_word', a.rounds,
                         run / 'positive_features_train_word.npy')
        append(run / 'positive_features_train.npy', parts)
        done.write_text(f'{sum(len(p) for p in parts)} rows\n')
