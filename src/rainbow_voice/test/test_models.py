"""The real models on two clips; skipped where the models are not installed.

Models: /opt/rainbow/voice in the image (Dockerfile), or $RAINBOW_VOICE_MODELS.
"""

import os
from pathlib import Path
import wave

import numpy as np
import pytest
from rainbow_voice.listener import CHUNK

MODELS = Path(os.environ.get('RAINBOW_VOICE_MODELS', '/opt/rainbow/voice'))
DATA = Path(__file__).parent / 'data'
needs_models = pytest.mark.skipif(not (MODELS / 'melspectrogram.onnx').exists(),
                                  reason=f'no voice models in {MODELS}')


def clip(name):
    with wave.open(str(DATA / name)) as w:
        a = np.frombuffer(w.readframes(w.getnframes()), dtype='<i2')
    pad = np.zeros(16000, dtype=np.int16)
    return np.concatenate([pad, a, pad])


def max_score(samples):
    from rainbow_voice.wakeword import WakeWords
    ww = WakeWords({'jarvis': MODELS / 'hey_jarvis_v0.1.onnx'},
                   MODELS / 'melspectrogram.onnx', MODELS / 'embedding_model.onnx')
    return max(ww.process(samples[i:i + CHUNK])['jarvis']
               for i in range(0, len(samples) - CHUNK + 1, CHUNK))


@needs_models
def test_wake_word_fires_only_on_the_word():
    assert max_score(clip('hey_jarvis_go_to_the_kitchen.wav')) > 0.9
    assert max_score(clip('dinosaurs_and_trains.wav')) < 0.1


@needs_models
def test_whole_pipeline_wake_word_then_command():
    """Wake word, recording until silence, then Vosk: as on the robot."""
    from rainbow_voice.commands import CommandRecognizer
    from rainbow_voice.listener import Listener, Vad
    from rainbow_voice.wakeword import WakeWords
    ww = WakeWords({'wake': MODELS / 'hey_jarvis_v0.1.onnx'},
                   MODELS / 'melspectrogram.onnx', MODELS / 'embedding_model.onnx')
    listener = Listener(ww.process, Vad(2), 'wake')
    samples = clip('hey_jarvis_go_to_the_kitchen.wav')
    events = []
    for i in range(0, len(samples) - CHUNK + 1, CHUNK):
        events += listener.feed(samples[i:i + CHUNK])
    assert [e[0] for e in events] == ['wake', 'utterance']
    r = CommandRecognizer(MODELS / 'vosk-model-small-en-us-0.15')
    r.set_phrases(['go to the kitchen', 'go to the sofa', 'come back', 'wait', 'stop'])
    text, conf = r.recognize(events[1][1])
    assert text == 'go to the kitchen' and conf > 0.8
    assert r.recognize(clip('dinosaurs_and_trains.wav')) == ('', 0.0)
