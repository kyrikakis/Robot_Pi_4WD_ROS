"""Far-field room impulse responses for training: a person speaking to the robot across a room.

The MIT recordings in /data/rirs are mostly measured close to the source (strong direct
sound), and models trained with them alone caught "hey rainbow" said from across a
reverberant room only ~33-54% of the time, against 82% for "hey Jarvis". These are
simulated with pyroomacoustics (image source method): shoebox rooms 3-8 x 2.5-6 m, RT60
0.3-1.0 s, the microphone 15-30 cm above the floor (the robot's ReSpeaker), the talker
standing or sitting 1-5 m away. The eval's far-room condition uses a different, simpler
model (exponential noise tail), so it stays unseen.

  make_far_rirs.py [--n 600]   ->  /data/rirs_far/*.wav (16 kHz)
"""

import argparse
import random
from pathlib import Path

import numpy as np
import pyroomacoustics as pra
import soundfile as sf

OUT = Path('/data/rirs_far')
SR = 16000


def one(rng):
    dims = [rng.uniform(3, 8), rng.uniform(2.5, 6), rng.uniform(2.4, 3.2)]
    rt60 = rng.uniform(0.3, 1.0)
    try:
        absorption, max_order = pra.inverse_sabine(rt60, dims)
    except ValueError:   # RT60 not reachable in a small room: take a shorter one
        absorption, max_order = pra.inverse_sabine(0.3, dims)
    room = pra.ShoeBox(dims, fs=SR, materials=pra.Material(absorption),
                       max_order=min(max_order, 60))
    for _ in range(100):
        mic = [rng.uniform(0.5, dims[0] - 0.5), rng.uniform(0.5, dims[1] - 0.5),
               rng.uniform(0.15, 0.30)]
        dist = rng.uniform(1.0, 5.0)
        angle = rng.uniform(0, 2 * np.pi)
        src = [mic[0] + dist * np.cos(angle), mic[1] + dist * np.sin(angle),
               rng.uniform(1.0, 1.8)]
        if 0.3 < src[0] < dims[0] - 0.3 and 0.3 < src[1] < dims[1] - 0.3:
            break
    else:
        return None
    room.add_source(src)
    room.add_microphone(mic)
    room.compute_rir()
    rir = np.asarray(room.rir[0][0], dtype=np.float32)
    rir = rir[np.argmax(np.abs(rir)) - 16:] if np.argmax(np.abs(rir)) > 16 else rir
    return rir / np.abs(rir).max(), dist, rt60


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--n', type=int, default=600)
    a = ap.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    rng = random.Random(7)
    made = len(list(OUT.glob('*.wav')))
    while made < a.n:
        r = one(rng)
        if r is None:
            continue
        rir, dist, rt60 = r
        sf.write(OUT / f'far_{made:04d}_d{dist:.1f}_rt{rt60:.2f}.wav', rir, SR, subtype='FLOAT')
        made += 1
        if made % 100 == 0:
            print(made, flush=True)
