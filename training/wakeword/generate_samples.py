"""generate_samples() as openWakeWord's train.py imports it, on the current piper-sample-generator.

train.py expects dscripka's old fork (16 kHz output, Python 3.9 packages). This module
gives it the same function on top of rhasspy/piper-sample-generator 3.x and piper-tts 1.3:

- Most clips (1 - ACCENT_FRACTION) come from the LibriTTS-R generator (904 speakers),
  each clip a random blend of two random speakers (the upstream loop pairs only the first
  few speakers with all the others), with random speed and variability.
- The rest come from the Piper voices in /data/voices (British, Scottish, non-native
  English, and Greek/German/Spanish/French/Italian/Hindi/Portuguese voices reading the
  English text with their own phonemizer, which gives an accent), a random speaker each.
- Accented positives (positive_* directories) are not synthesized here but drawn from
  /pool/<model>/<split>/accepted.txt: clips from those voices that Whisper heard as the
  phrase (make_accent_pool.py, whisper_filter.py). Accented negatives are synthesized
  directly (a mangled negative does no harm).
- Speakers are split: clips written to a *_test directory only use speakers never used
  for training (LibriTTS speakers 800-899, every 4th speaker of the multi-speaker voices,
  and two whole voices), so the validation recall is on unseen voices.
- Everything is resampled to 16 kHz (train.py's augmentation requires it).
"""

import logging
import os
import random
import re
import sys
import wave
from pathlib import Path

import numpy as np
import torch
from scipy.signal import resample_poly

sys.path.insert(0, '/opt/piper-sample-generator')
from piper import PiperVoice, SynthesisConfig  # noqa: E402
from piper_sample_generator.__main__ import generate_audio, get_phonemes  # noqa: E402

GENERATOR = Path(os.environ.get('WW_GENERATOR', '/data/models/en_US-libritts_r-medium.pt'))
GENERATOR_CONFIG = Path('/opt/piper-sample-generator/models/en_US-libritts_r-medium.pt.json')
VOICES_DIR = Path(os.environ.get('WW_VOICES', '/data/voices'))
POOL_DIR = Path(os.environ.get('WW_POOL', '/pool'))
ACCENT_FRACTION = float(os.environ.get('WW_ACCENT_FRACTION', '0.3'))
# Greek-accented pronunciations (IPA) for the LibriTTS generator: Greek vowels (pure e/i/o,
# open o), χ/ç for "h", a trilled r, m before b. Piper's Greek voices produce gibberish even
# for Greek text, so the accent comes from these phonemes in many English voices instead.
# Whisper decides which clips are kept (an American model turns the Greek tap ɾ into "d").
GREEK_PHONEMES = {
    'hey rainbow': ['çˈei rˈeinbou', 'xˈei rˈeinbou', 'hˈei rˈeinbo', 'hˈei ɹˈeinbo',
                    'xˈei ɹˈeimbo', 'çˈei rˈeimbo', 'hˈeɪ rˈeɪnbo', 'xˈei ɹˈeinbou',
                    'hˈei ɹˈeimbou'],
    'stop': ['stˈɔp', 'stˈɔːp', 'stˈop'],
    'hey robot': ['çˈei rˈobot', 'xˈei rˈobot', 'hˈei ɹˈobot', 'hˈei ɹˈobɔt', 'xˈei ɹˈobɔt',
                  'hˈeɪ rˈobɔt', 'çˈei ɹˈoubot', 'hˈei rˈobɔt'],
    # the names alone (add_word.py), the same pronunciations without "hey"
    'rainbow': ['rˈeinbou', 'rˈeinbo', 'ɹˈeinbo', 'ɹˈeimbo', 'rˈeimbo', 'rˈeɪnbo',
                'ɹˈeinbou', 'ɹˈeimbou'],
    'robot': ['rˈobot', 'ɹˈobot', 'ɹˈobɔt', 'rˈobɔt', 'ɹˈoubot'],
}
GREEK_SHARE = float(os.environ.get('WW_GREEK_SHARE', '0.5'))   # of the accented positives
TEST_ONLY_VOICES = {'en_GB-alba-medium', 'el_GR-joy-medium'}
TRAIN_SPEAKERS = range(0, 800)
TEST_SPEAKERS = range(800, 900)   # the last few LibriTTS speakers are noisy; not used

_LOG = logging.getLogger('generate_samples')
_generator = None
_voices = {}
_pools = {}


def _load_generator():
    global _generator
    if _generator is None:
        import json
        _generator = torch.load(GENERATOR, weights_only=False, map_location='cpu')
        _generator.eval()
        _generator.config = json.load(open(GENERATOR_CONFIG))
    return _generator


def _voice_pool(test):
    """(voice, speaker) choices: training and test use different speakers."""
    if not _voices:
        for onnx in sorted(VOICES_DIR.glob('*.onnx')):
            _voices[onnx.stem] = PiperVoice.load(str(onnx))
        _pools[False], _pools[True] = [], []
        for name, voice in _voices.items():
            n = voice.config.num_speakers
            if name in TEST_ONLY_VOICES:
                _pools[True] += [(name, s if n > 1 else None) for s in range(n)]
            elif n > 1:
                for s in range(n):
                    _pools[s % 4 == 3].append((name, s))
            else:
                _pools[False].append((name, None))
    return _pools[test]


def _to_16k(audio, sr):
    g = np.gcd(sr, 16000)
    out = resample_poly(audio.astype(np.float32), 16000 // g, sr // g)
    out = np.trim_zeros(np.clip(out, -1, 1))
    return (out * 32767).astype(np.int16)


def _write(path, samples):
    with wave.open(str(path), 'wb') as w:
        w.setframerate(16000)
        w.setsampwidth(2)
        w.setnchannels(1)
        w.writeframes(samples.tobytes())


def _generator_batch(texts, test, length_scales, noise_scales, noise_scale_ws):
    model = _load_generator()
    config = model.config
    speakers = TEST_SPEAKERS if test else TRAIN_SPEAKERS
    ids = [get_phonemes(config['espeak']['voice'], config, t) for t in texts]
    longest = max(len(i) for i in ids)
    ids = [i + [1] * (longest - len(i)) for i in ids]
    s1 = torch.LongTensor([random.choice(speakers) for _ in texts])
    s2 = torch.LongTensor([random.choice(speakers) for _ in texts])
    with torch.no_grad():
        audio, phoneme_samples = generate_audio(
            model, s1, s2, ids, slerp_weight=random.uniform(0.0, 1.0),
            noise_scale=random.choice(noise_scales), noise_scale_w=random.choice(noise_scale_ws),
            length_scale=random.choice(length_scales) * random.uniform(0.9, 1.1), max_len=None)
    out = []
    for i in range(audio.shape[0]):
        last = int(phoneme_samples[i].flatten().sum().item())
        out.append(('libritts', _to_16k(audio[i, 0, :last + 1].numpy(),
                                        config['audio']['sample_rate'])))
    return out


def greek_batch(text, n, test):
    """n clips of text with a Greek accent (random variant per batch) from LibriTTS speakers."""
    model = _load_generator()
    config = model.config
    speakers = TEST_SPEAKERS if test else TRAIN_SPEAKERS
    phonemes = random.choice(GREEK_PHONEMES[text])
    ids = [get_phonemes('en-us', config, phonemes, phoneme_input=True)] * n
    s1 = torch.LongTensor([random.choice(speakers) for _ in range(n)])
    s2 = torch.LongTensor([random.choice(speakers) for _ in range(n)])
    with torch.no_grad():
        audio, phoneme_samples = generate_audio(
            model, s1, s2, ids, slerp_weight=random.uniform(0.0, 1.0),
            noise_scale=random.uniform(0.6, 0.8), noise_scale_w=random.uniform(0.7, 0.9),
            length_scale=random.uniform(0.85, 1.25), max_len=None)
    out = []
    for i in range(n):
        last = int(phoneme_samples[i].flatten().sum().item())
        out.append(('greek', _to_16k(audio[i, 0, :last + 1].numpy(),
                                     config['audio']['sample_rate'])))
    return out


def _voice_clip(text, test, length_scales):
    pool = _voice_pool(test)
    if not pool:
        return None
    name = random.choice(sorted({n for n, _ in pool}))   # each voice equally often
    speaker = random.choice([s for n, s in pool if n == name])
    syn = SynthesisConfig(speaker_id=speaker,
                          length_scale=random.choice(length_scales) * random.uniform(0.85, 1.15),
                          noise_scale=random.uniform(0.5, 0.9), noise_w_scale=random.uniform(0.6, 1.0))
    chunks = list(_voices[name].synthesize(text, syn_config=syn))
    if not chunks:
        return None
    audio = np.concatenate([c.audio_float_array for c in chunks])
    return name, _to_16k(audio, chunks[0].sample_rate)


def _accepted_pool(text, test):
    folder = POOL_DIR / text.replace(' ', '_') / ('test' if test else 'train')
    listing = folder / 'accepted.txt'
    if not listing.exists():
        raise FileNotFoundError(f'{listing}: run make_accent_pool.py and whisper_filter.py')
    files = [folder / line.split('\t')[0] for line in listing.read_text().splitlines() if line]
    random.shuffle(files)
    return files


def _read_wav(path):
    with wave.open(str(path)) as w:
        return np.frombuffer(w.readframes(w.getnframes()), np.int16)


def generate_samples(text, max_samples, output_dir, batch_size=50, file_names=None,
                     length_scales=(0.75, 1.0, 1.25), noise_scales=(0.667,),
                     noise_scale_ws=(0.8,), auto_reduce_batch_size=False, **kwargs):
    """Write max_samples 16 kHz wav clips of the given text(s) to output_dir."""
    texts = [text] if isinstance(text, str) else list(text)
    output_dir = Path(output_dir)
    exclude = os.environ.get('WW_NEGATIVE_EXCLUDE')
    if exclude and not output_dir.name.startswith('positive'):
        # Adversarial texts that contain the wake word ("bus stop", "stopper") contradict the
        # positives: the model fires at the end of the word, before it hears what follows.
        kept = [t for t in texts if not re.search(exclude, t, re.IGNORECASE)]
        _LOG.warning('%s: dropped %d of %d texts matching %r', output_dir.name,
                     len(texts) - len(kept), len(texts), exclude)
        texts = kept
    output_dir.mkdir(parents=True, exist_ok=True)
    test = output_dir.name.endswith('_test')
    names = iter(file_names) if file_names else None
    batch_size = max(1, min(batch_size, 32))
    torch.set_num_threads(os.cpu_count() or 4)

    def next_path(source):
        if names is not None:
            try:
                return output_dir / next(names)
            except StopIteration:
                pass
        return output_dir / f'{source}__{os.urandom(8).hex()}.wav'   # source: for the eval

    positive = output_dir.name.startswith('positive')
    pools = {True: [], False: []}   # accepted accented clips, Greek or not; reused when used up

    def accent_positive(t):
        greek = random.random() < GREEK_SHARE
        if not pools[greek]:
            pools[greek] = [p for p in _accepted_pool(t, test)
                            if p.name.startswith('greek__') == greek]
        path = pools[greek].pop()
        return path.name.split('__')[0], _read_wav(path)

    written = 0
    pbar_every = max(1, max_samples // 50)
    while written < max_samples:
        n = min(batch_size, max_samples - written)
        if random.random() < ACCENT_FRACTION:
            make = accent_positive if positive else \
                (lambda t: _voice_clip(t, test, length_scales))
            clips = [make(random.choice(texts)) for _ in range(n)]
        else:
            clips = _generator_batch([random.choice(texts) for _ in range(n)], test,
                                     length_scales, noise_scales, noise_scale_ws)
        for item in clips:
            if item is None or len(item[1]) < 1600:   # nothing, or under 0.1 s
                continue
            _write(next_path(item[0]), item[1])
            written += 1
            if written % pbar_every == 0:
                _LOG.warning('%s: %d/%d clips', output_dir.name, written, max_samples)
            if written >= max_samples:
                break
