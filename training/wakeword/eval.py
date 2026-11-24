"""Evaluate wake word models the way the robot runs them.

Streams audio in 80 ms chunks through the robot's own rainbow_voice/wakeword.py (mounted at
/robot), with a 2 s cooldown after each detection as in listener.py, and reports:

  recall       held-out positive clips (speakers never used in training), each placed in
               4 s of real background (AudioSet clips never used in training) at several
               signal-to-noise ratios, broken down by voice (accent)
  near misses  held-out adversarial clips ("hey robin", "top", ...) the same way: share that fire
  FA/hour      false accepts per hour on ~5 h of AudioSet (speech, music, noise) and on
               espeak speech (the robot's own voice)

Usage: eval.py --model NAME=/path/model.onnx --positives DIR --negatives DIR [--json OUT]
"""

import argparse
import collections
import json
import os
import random
import sys
from multiprocessing import Pool
from pathlib import Path

import numpy as np
import soundfile as sf

sys.path.insert(0, '/robot')
from wakeword import CHUNK, WakeWords  # noqa: E402  (the robot's code)

FEATURES = Path('/opt/openWakeWord/openwakeword/resources/models')
# None: digital silence around the clip (no real microphone hears that); 30: a quiet room;
# 'room': far, reverberant, 5 dB noise
SNRS = [None, 30, 10, 5, 0, 'room']
COOLDOWN_CHUNKS = 25      # 2 s


def _read(path):
    data, sr = sf.read(path, dtype='int16')
    assert sr == 16000, path
    return data if data.ndim == 1 else data[:, 0]


def _detector(model):
    return WakeWords({'w': model}, FEATURES / 'melspectrogram.onnx',
                     FEATURES / 'embedding_model.onnx')


def _scores(ww, samples):
    ww.reset()
    n = len(samples) // CHUNK
    return np.array([ww.process(samples[i * CHUNK:(i + 1) * CHUNK])['w'] for i in range(n)])


def _count(scores, threshold):
    hits, quiet = 0, -1
    for i, s in enumerate(scores):
        if s >= threshold and i > quiet:
            hits += 1
            quiet = i + COOLDOWN_CHUNKS
    return hits


def _room(clip, rng):
    """The clip as heard across a room: a synthetic impulse response (direct sound plus
    exponentially decaying reverb, RT60 0.4-1.0 s, direct-to-reverb -8..0 dB: far away). Synthetic,
    so never seen in training (which uses the MIT recordings)."""
    rt60 = rng.uniform(0.4, 1.0)
    n = int(16000 * rt60)
    t = np.arange(n) / 16000
    nprng = np.random.default_rng(rng.randrange(1 << 30))
    tail = nprng.standard_normal(n) * np.exp(-6.9 * t / rt60)
    tail[:int(0.003 * 16000)] = 0   # reflections start a few ms after the direct sound
    drr = 10 ** (rng.uniform(-8, 0) / 10)
    tail *= np.sqrt(1 / (drr * np.sum(tail ** 2)))
    ir = tail
    ir[0] = 1.0
    out = np.convolve(clip.astype(np.float32), ir)[:len(clip) + n // 2]
    return out * (np.abs(clip).max() / (np.abs(out).max() + 1e-9))


def _mix(clip, background, snr, rng):
    """4 s: background, the clip starting at 1.5-2 s at the given SNR (None: clean;
    'room': far away in a reverberant room, then 5 dB of background)."""
    total = 16000 * 4
    out = np.zeros(total, dtype=np.float32)
    start = rng.randint(24000, 32000)
    if snr == 'room':
        clip, snr = _room(clip, rng), 5
    clip = clip[:total - start].astype(np.float32)
    out[start:start + len(clip)] = clip
    if snr is not None:
        bg = background.astype(np.float32)
        if len(bg) < total:
            bg = np.tile(bg, total // len(bg) + 1)
        off = rng.randint(0, len(bg) - total)
        bg = bg[off:off + total]
        p_clip = np.mean(clip ** 2) + 1e-9
        p_bg = np.mean(bg ** 2) + 1e-9
        out += bg * np.sqrt(p_clip / (p_bg * 10 ** (snr / 10)))
    peak = np.abs(out).max()
    if peak > 32767:
        out *= 32767 / peak
    return out.astype(np.int16)


def _clip_job(args):
    model, clips, backgrounds, seed, thresholds = args
    ww = _detector(model)
    rng = random.Random(seed)
    out = []
    for clip_path in clips:
        clip = _read(clip_path)
        source = Path(clip_path).name.split('__')[0] if '__' in Path(clip_path).name else '?'
        for snr in SNRS:
            bg = _read(rng.choice(backgrounds))
            s = _scores(ww, _mix(clip, bg, snr, rng))
            out.append((source, snr, float(s.max())))
    return out


def _stream_job(args):
    model, files, thresholds = args
    ww = _detector(model)
    audio = np.concatenate([_read(f) for f in files])
    s = _scores(ww, audio)
    return len(audio) / 16000, {t: _count(s, t) for t in thresholds}


def evaluate(model, positives, negatives, thresholds=(0.1, 0.2, 0.3, 0.4, 0.5, 0.7), n_clips=400,
             workers=None, seed=0):
    workers = workers or max(1, (os.cpu_count() or 4) - 2)
    rng = random.Random(seed)
    eval_bg = sorted(str(p) for p in Path('/data/eval_background').glob('*.wav'))
    espeak = sorted(str(p) for p in Path('/data/espeak').glob('*.wav'))
    result = {'model': str(model), 'thresholds': list(thresholds)}

    with Pool(workers) as pool:
        for kind, folder in (('positive', positives), ('near_miss', negatives)):
            clips = sorted(str(p) for p in Path(folder).glob('*.wav'))
            rng.shuffle(clips)
            clips = clips[:n_clips]
            jobs = [(model, clips[i::workers], eval_bg, seed + i, thresholds) for i in range(workers)]
            rows = [r for part in pool.map(_clip_job, jobs) for r in part]
            by_snr, by_source = {}, collections.defaultdict(list)
            for t in thresholds:
                by_snr[t] = {str(snr): round(float(np.mean([m >= t for _, s, m in rows if s == snr])), 3)
                             for snr in SNRS}
            for source, snr, m in rows:
                by_source[source].append(m)
            result[kind] = {
                'clips': len(clips), 'by_snr': by_snr,
                # the plain LibriTTS voices alone: comparable with "hey Jarvis" (no accented clips)
                'libritts_by_snr@0.5': {str(snr): round(float(np.mean(
                    [m >= 0.5 for src, s, m in rows if s == snr and src == 'libritts'] or [0])), 3)
                    for snr in SNRS},
                'by_voice@0.5': {k: [round(float(np.mean(np.array(v) >= 0.5)), 2), len(v) // len(SNRS)]
                                 for k, v in sorted(by_source.items())},
            }

        for kind, files in (('audioset', eval_bg), ('espeak', espeak)):
            jobs = [(model, files[i::workers], thresholds) for i in range(workers)]
            parts = pool.map(_stream_job, jobs)
            hours = sum(p[0] for p in parts) / 3600
            result[f'fa_per_hour_{kind}'] = {
                str(t): round(sum(p[1][t] for p in parts) / hours, 2) for t in thresholds}
            result[f'hours_{kind}'] = round(hours, 2)
    return result


def summary(name, r):
    lines = [f'== {name}  ({Path(r["model"]).name})']
    lines.append('  threshold   recall silent/30/10/5/0dB/room near-miss fires   FA/h audioset  FA/h espeak')
    for t in r['thresholds']:
        rec = '/'.join(f'{r["positive"]["by_snr"][t][str(s)]:.2f}' for s in SNRS)
        nm = np.mean(list(r['near_miss']['by_snr'][t].values()))
        lines.append(f'  {t:<10}  {rec:<31} {nm:<17.3f} {r["fa_per_hour_audioset"][str(t)]:<14} '
                     f'{r["fa_per_hour_espeak"][str(t)]}')
    lines.append(f'  hours: audioset {r["hours_audioset"]}, espeak {r["hours_espeak"]}')
    lines.append('  recall@0.5 LibriTTS voices only: ' + '/'.join(
        f'{v:.2f}' for v in r['positive'].get('libritts_by_snr@0.5', {}).values()))
    lines.append('  recall@0.5 by voice (all SNRs): ' + ', '.join(
        f'{k} {v[0]:.2f}' for k, v in r['positive']['by_voice@0.5'].items()))
    return '\n'.join(lines)


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', action='append', required=True, help='NAME=path.onnx')
    ap.add_argument('--positives', required=True)
    ap.add_argument('--negatives', required=True)
    ap.add_argument('--clips', type=int, default=400)
    ap.add_argument('--json')
    ap.add_argument('--thresholds', default='0.1,0.2,0.3,0.4,0.5,0.7')
    a = ap.parse_args()
    thresholds = tuple(float(t) for t in a.thresholds.split(','))
    results = {}
    for spec in a.model:
        name, path = spec.split('=', 1)
        results[name] = evaluate(path, a.positives, a.negatives, thresholds, n_clips=a.clips)
        print(summary(name, results[name]), flush=True)
    if a.json:
        Path(a.json).parent.mkdir(parents=True, exist_ok=True)
        json.dump(results, open(a.json, 'w'), indent=1)
