"""Add more augmented copies of the positive training clips to the training features.

train.py computes the features of each clip once, with one random augmentation (room
impulse response on about half the batches), so the model sees each positive in a single
acoustic condition. "hey rainbow" r3 kept up with "hey Jarvis" on clean clips but fell far
behind in noise and in a reverberant room (33% vs 82%). This appends ROUNDS more copies of
every positive with fresh augmentation (room on 80% of batches, background noise on 90%) to
/data/runs/<model>/positive_features_train.npy; the original is kept as *.base.npy.

Room impulse responses and background come from the model's config (/src/configs), so the
far-field rooms (make_far_rirs.py) are used. --negative-rounds adds the same for the
adversarial negatives, so that a reverberant room is no cue for the wake word.

  extra_positives.py hey_rainbow [--rounds 2] [--negative-rounds 1]   (defaults)
  extra_positives.py stop --rounds 0 --rebuild-negatives   (after regenerating negative_train)
"""

import argparse
import os
import random
import shutil
from pathlib import Path

import numpy as np
import yaml
from openwakeword.data import augment_clips
from openwakeword.utils import compute_features_from_generator

PROBABILITIES = {"SevenBandParametricEQ": 0.25, "TanhDistortion": 0.15, "PitchShift": 0.25,
                 "BandStopFilter": 0.25, "AddColoredNoise": 0.3, "AddBackgroundNoise": 0.9,
                 "Gain": 1.0, "RIR": 0.8}

def rebuild_base(model, kind):
    """Features of freshly generated clips, one round with train.py's own augmentation
    (train.py only computes features when the positive ones are missing too)."""
    run = Path('/data/runs') / model
    base = run / f'{kind}_features_train.base.npy'
    config = yaml.safe_load(open(f'/src/configs/{model}.yml'))
    background = []
    for path, dup in zip(config['background_paths'], config['background_paths_duplication_rate']):
        background += [str(p) for p in Path(path).glob('*.wav')] * dup
    rirs = [str(p) for d in config['rir_paths'] for p in Path(d).glob('*.wav')]
    # the test features too when they were removed with the regenerated test clips
    test = run / f'{kind}_features_test.npy'
    for split, out in (('train', base), ('test', test)):
        if split == 'test' and test.exists():
            continue
        clips = [str(p) for p in (run / f'{kind}_{split}').glob('*.wav')]
        gen = augment_clips(clips, total_length=32000, batch_size=16,
                            background_clip_paths=background, RIR_paths=rirs)
        partial = out.with_suffix('.partial.npy')
        compute_features_from_generator(gen, n_total=len(clips), clip_duration=32000,
                                        output_file=str(partial), device='cpu',
                                        ncpu=max(1, (os.cpu_count() or 8) // 2))
        os.replace(partial, out)
        print(f'{out.name}:', np.load(out, mmap_mode='r').shape, flush=True)
    (run / f'{kind}_features_train.extra.npy').unlink(missing_ok=True)


def extend(model, kind, rounds):
    run = Path('/data/runs') / model
    feats = run / f'{kind}_features_train.npy'
    base = run / f'{kind}_features_train.base.npy'
    if not base.exists():
        shutil.copy(feats, base)
    base_arr = np.load(base, mmap_mode='r')
    # train.py makes 2 s clips (16 feature frames) for short phrases like ours
    assert base_arr.shape[1] == 16, base_arr.shape
    total_length = 32000
    config = yaml.safe_load(open(f'/src/configs/{model}.yml'))
    background = []
    for path, dup in zip(config['background_paths'], config['background_paths_duplication_rate']):
        background += [str(p) for p in Path(path).glob('*.wav')] * dup
    rirs = [str(p) for d in config['rir_paths'] for p in Path(d).glob('*.wav')]
    clips = [str(p) for p in (run / f'{kind}_train').glob('*.wav')] * rounds
    random.shuffle(clips)
    extra = run / f'{kind}_features_train.extra.npy'
    if extra.exists() and len(np.load(extra, mmap_mode='r')) >= 0.95 * len(clips):
        print(f'{kind}: reusing {extra}')   # computed by an earlier run
    else:
        compute(clips, background, rirs, extra, total_length)
    merge(feats, base_arr, extra)


def compute(clips, background, rirs, extra, total_length):
    gen = augment_clips(clips, total_length=total_length,
                        batch_size=16, augmentation_probabilities=PROBABILITIES,
                        background_clip_paths=background, RIR_paths=rirs)
    partial = extra.with_suffix('.partial.npy')   # renamed when complete, so never reused half-done
    compute_features_from_generator(gen, n_total=len(clips),
                                    clip_duration=total_length,
                                    output_file=str(partial), device='cpu',
                                    ncpu=max(1, (os.cpu_count() or 8) // 2))
    os.replace(partial, extra)


def merge(feats, base_arr, extra):
    extra_arr = np.load(extra, mmap_mode='r')
    assert extra_arr.shape[1:] == base_arr.shape[1:], (extra_arr.shape, base_arr.shape)
    tmp = feats.with_suffix('.tmp.npy')   # swapped in at the end: a running training keeps its file
    combined = np.lib.format.open_memmap(tmp, mode='w+', dtype=np.float32,
                                         shape=(len(base_arr) + len(extra_arr),) + base_arr.shape[1:])
    combined[:len(base_arr)] = base_arr
    combined[len(base_arr):] = extra_arr
    combined.flush()
    shape = combined.shape
    del combined
    os.replace(tmp, feats)
    print(f'{feats.name}:', base_arr.shape, '+', extra_arr.shape, '->', shape)


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('model')
    ap.add_argument('--rounds', type=int, default=2)
    ap.add_argument('--negative-rounds', type=int, default=1)
    ap.add_argument('--rebuild-negatives', action='store_true',
                    help='first recompute the base features of regenerated negative clips')
    a = ap.parse_args()
    if a.rebuild_negatives:
        rebuild_base(a.model, 'negative')
    for kind, rounds in (('positive', a.rounds), ('negative', a.negative_rounds)):
        if rounds:
            extend(a.model, kind, rounds)

