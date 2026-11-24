"""Download and prepare everything the training needs, into /data (a Docker volume).

Idempotent: finished steps are skipped. Hugging Face files come from pinned revisions.

  /data/rirs/             MIT room impulse responses (16 kHz wav)
  /data/background/       AudioSet clips (noise, music, speech; 16 kHz wav) for augmentation
  /data/eval_background/  AudioSet clips never used in training, for the false-accept eval
  /data/espeak/           espeak-ng speech (the robot's own voice), background for "stop"
  /data/voices/           Piper voices for accents (onnx + json)
  /data/features/         openWakeWord's precomputed negative features (downloaded separately)
"""

import io
import json
import os
import random
import subprocess
from pathlib import Path

import numpy as np
import pyarrow.parquet as pq
import requests
import soundfile as sf
from scipy.signal import resample_poly
from tqdm import tqdm

DATA = Path('/data')
HF = 'https://huggingface.co'
RIR_REPO = 'davidscripka/MIT_environmental_impulse_responses'
RIR_REV = 'b824a1ef2821f112fda0b9cb26e4278c62b425bb'
AUDIOSET_REPO = 'agkphysics/AudioSet'
AUDIOSET_REV = '0c609e8302cf139307f639c57652032af0a88041'
AUDIOSET_TRAIN_PARTS = ['00', '01', '02', '03', '04', '05']   # ~500 clips (~1.4 h) each
AUDIOSET_EVAL_PARTS = ['06', '07', '08', '09']
VOICES_REV = 'e9c8a39444359388adaa94255becb37a5a2b8e51'

# Accent voices: (name, path in rhasspy/piper-voices)
VOICES = [
    'en/en_GB/vctk/medium/en_GB-vctk-medium',
    'en/en_US/l2arctic/medium/en_US-l2arctic-medium',
    'en/en_US/arctic/medium/en_US-arctic-medium',
    'en/en_GB/aru/medium/en_GB-aru-medium',
    'en/en_GB/semaine/medium/en_GB-semaine-medium',
    'en/en_GB/alba/medium/en_GB-alba-medium',
    'en/en_GB/northern_english_male/medium/en_GB-northern_english_male-medium',
    'en/en_GB/southern_english_female/low/en_GB-southern_english_female-low',
    'en/en_GB/cori/medium/en_GB-cori-medium',
    'en/en_US/kusal/medium/en_US-kusal-medium',
    'en/en_US/reza_ibrahim/medium/en_US-reza_ibrahim-medium',
    'el/el_GR/rapunzelina/medium/el_GR-rapunzelina-medium',
    'el/el_GR/joy/medium/el_GR-joy-medium',
    'de/de_DE/thorsten/medium/de_DE-thorsten-medium',
    'es/es_ES/sharvard/medium/es_ES-sharvard-medium',
    'es/es_MX/ald/medium/es_MX-ald-medium',
    'fr/fr_FR/siwis/medium/fr_FR-siwis-medium',
    'it/it_IT/paola/medium/it_IT-paola-medium',
    'hi/hi_IN/pratham/medium/hi_IN-pratham-medium',
    'hi/hi_IN/priyamvada/medium/hi_IN-priyamvada-medium',
    'pt/pt_BR/faber/medium/pt_BR-faber-medium',
]

ESPEAK_SENTENCES = [
    "I'm offline, I can only do simple things.", "Okay, going to the kitchen.",
    "I can see you.", "Hello, nice to meet you.", "I don't know where that is.",
    "I'm looking around.", "The battery is getting low.", "I'm not sure, can you say that again?",
    "Turning left.", "I found the bedroom.", "Sorry, I can't go there.", "Here I am!",
    "It is twenty degrees in the living room.", "Let me think about that.",
    "I remember you said you like coffee.", "Okay, I will wait here.",
    "There is something in front of me.", "Good morning, did you sleep well?",
    "I took a photo.", "That was fun, what next?", "I am a small robot called Rainbow.",
    "Navigation started.", "I arrived.", "I'm stuck, can you help me?",
    "The weather today is sunny with some clouds.", "Shall I follow you?",
]


def download(url, dest, chunk=1 << 20):
    dest = Path(dest)
    if dest.exists() and dest.stat().st_size > 0:
        return dest
    dest.parent.mkdir(parents=True, exist_ok=True)
    tmp = dest.with_suffix(dest.suffix + '.part')
    with requests.get(url, stream=True, timeout=60) as r:
        r.raise_for_status()
        with open(tmp, 'wb') as f:
            for block in r.iter_content(chunk):
                f.write(block)
    tmp.rename(dest)
    return dest


def to_16k_int16(audio, sr):
    if audio.ndim > 1:
        audio = audio.mean(axis=1)
    if sr != 16000:
        g = np.gcd(sr, 16000)
        audio = resample_poly(audio, 16000 // g, sr // g)
    return (np.clip(audio, -1, 1) * 32767).astype(np.int16)


def rirs():
    out = DATA / 'rirs'
    if out.exists() and len(list(out.glob('*.wav'))) > 250:
        return
    files = requests.get(f'{HF}/api/datasets/{RIR_REPO}/revision/{RIR_REV}', timeout=60).json()
    names = [s['rfilename'] for s in files['siblings'] if s['rfilename'].startswith('16khz/')]
    for name in tqdm(names, desc='RIRs'):
        download(f'{HF}/datasets/{RIR_REPO}/resolve/{RIR_REV}/{name}', out / Path(name).name)


def audioset(parts, out):
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    for part in parts:
        done = out / f'.done_{part}'
        if done.exists():
            continue
        pfile = DATA / 'downloads' / f'audioset_bal_train_{part}.parquet'
        download(f'{HF}/datasets/{AUDIOSET_REPO}/resolve/{AUDIOSET_REV}/data/bal_train/{part}.parquet',
                 pfile)
        table = pq.ParquetFile(pfile)
        for batch in tqdm(table.iter_batches(batch_size=64), desc=f'AudioSet {part}'):
            rows = batch.to_pylist()
            for row in rows:
                audio = row['audio']
                name = Path(audio.get('path') or f"{row.get('video_id', random.random())}").stem
                try:
                    data, sr = sf.read(io.BytesIO(audio['bytes']), dtype='float32')
                except Exception:   # noqa: B902 - a few clips are broken; skip them
                    continue
                if len(data) < sr:  # under 1 s
                    continue
                sf.write(out / f'{part}_{name}.wav', to_16k_int16(data, sr), 16000,
                         subtype='PCM_16')
        pfile.unlink()  # the wavs are what is kept
        done.touch()


def voices():
    out = DATA / 'voices'
    for path in tqdm(VOICES, desc='voices'):
        name = Path(path).name
        for ext in ('.onnx', '.onnx.json'):
            download(f'{HF}/rhasspy/piper-voices/resolve/{VOICES_REV}/{path}{ext}', out / (name + ext))


def espeak():
    out = DATA / 'espeak'
    if out.exists() and len(list(out.glob('*.wav'))) >= 300:
        return
    out.mkdir(parents=True, exist_ok=True)
    rng = random.Random(0)
    n = 0
    for i in range(300):
        text = ' '.join(rng.sample(ESPEAK_SENTENCES, 3))
        voice = rng.choice(['en', 'en-us', 'en-gb', 'en+f3', 'en+m3', 'en-us+f2'])
        speed = rng.randint(140, 190)
        wav = out / f'espeak_{i:03d}.wav'
        subprocess.run(['espeak-ng', '-v', voice, '-s', str(speed), '-w', str(wav) + '.tmp', text],
                       check=True)
        data, sr = sf.read(str(wav) + '.tmp', dtype='float32')
        sf.write(wav, to_16k_int16(data, sr), 16000, subtype='PCM_16')
        os.remove(str(wav) + '.tmp')
        n += 1
    print(f'espeak: {n} clips')


def check_features():
    sizes = {'openwakeword_features_ACAV100M_2000_hrs_16bit.npy': 17280000000,
             'validation_set_features.npy': 185000000}
    for name, size in sizes.items():
        p = DATA / 'features' / name
        ok = p.exists() and p.stat().st_size >= size * 0.99
        print(f'{name}: {"ok" if ok else "MISSING/incomplete"}')


if __name__ == '__main__':
    rirs()
    voices()
    espeak()
    audioset(AUDIOSET_TRAIN_PARTS, DATA / 'background')
    audioset(AUDIOSET_EVAL_PARTS, DATA / 'eval_background')
    check_features()
    print(json.dumps({d: len(list((DATA / d).glob('*'))) for d in
                      ['rirs', 'background', 'eval_background', 'espeak', 'voices']}))
