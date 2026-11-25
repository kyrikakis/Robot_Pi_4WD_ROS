"""Speech to text: Whisper large-v3-turbo with mlx-whisper (Metal), English only.

Audio is passed as samples, never as a file: mlx-whisper decodes files with
ffmpeg, which we do not install. The robot sends 16 kHz mono PCM anyway;
read_wav reads such files for testing.
"""

import re
import time
import wave

import numpy as np

RATE = 16000

# What Whisper says on silence or noise (it was trained on subtitles)
HALLUCINATIONS = {'', 'you', 'thank you', 'thank you.', 'thanks for watching!',
                  'thanks for watching.', 'bye.', 'bye', '.', 'okay.', 'so', 'uh'}


def answer_text(text):
    """An answer to the robot's question: only empty text is dropped. A short "yes"
    alone comes out of Whisper as "you" or "thank you", which would otherwise be
    dropped as silence (the owner's "yes" to "Stefanos, did I get that right?" was).
    Returns the text to use, or '' to drop it."""
    t = ' '.join(text.lower().split()).strip(' .!?')
    if not t:
        return ''
    if t in ('you', 'yes you', 'yeah', 'yep'):
        return 'Yes.'
    return text.strip()


def is_speech(text):
    """Return False for empty text, Whisper's silence hallucinations and its loops.

    On background noise Whisper repeats itself ("You can make it very special.
    You can make it very special. ..."): a sentence three times means noise.
    """
    t = ' '.join(text.lower().split())
    if len(t) <= 1 or t in HALLUCINATIONS:
        return False
    # Noise also comes out as symbols ("¢$!", "a ʤ0'-%$!"): need mostly a-z letters
    letters = sum(c.isascii() and c.isalpha() for c in t)
    if letters < 2 or letters < 0.6 * len(t.replace(' ', '')):
        return False
    sentences = [x.strip() for x in re.split(r'[.!?]+', t) if x.strip()]
    return not any(sentences.count(x) >= 3 for x in set(sentences))


def pcm16_to_float(data):
    """16-bit little-endian PCM bytes -> float32 samples in [-1, 1]."""
    return np.frombuffer(data, dtype='<i2').astype(np.float32) / 32768.0


def read_wav(path):
    """A 16 kHz, 16-bit WAV (mono, or the first channel) as float32 samples."""
    with wave.open(str(path)) as w:
        if w.getframerate() != RATE or w.getsampwidth() != 2:
            raise ValueError(f'{path}: need 16 kHz 16-bit PCM, got '
                             f'{w.getframerate()} Hz {8 * w.getsampwidth()}-bit '
                             f'(convert with: afconvert -f WAVE -d LEI16@16000 -c 1 in out)')
        samples = pcm16_to_float(w.readframes(w.getnframes()))
        return samples[::w.getnchannels()]


class Whisper:

    def __init__(self, model_dir):
        import mlx_whisper  # slow import (MLX); only when speech is used
        self._transcribe = mlx_whisper.transcribe
        self.model_dir = str(model_dir)

    def transcribe(self, audio):
        """audio: a WAV path or 16 kHz float32 samples. Returns (text, seconds)."""
        if not isinstance(audio, np.ndarray):
            audio = read_wav(audio)
        start = time.monotonic()
        result = self._transcribe(audio, path_or_hf_repo=self.model_dir, language='en',
                                  condition_on_previous_text=False)
        return result['text'].strip(), time.monotonic() - start
