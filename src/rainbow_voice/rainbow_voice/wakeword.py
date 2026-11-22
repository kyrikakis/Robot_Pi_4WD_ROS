"""Wake word detection with openWakeWord's models, on onnxruntime alone.

openWakeWord (https://github.com/dscripka/openWakeWord) is not installed: it
pulls in scipy and scikit-learn, and its Linux install wants tflite-runtime,
which has no Python 3.12 build. Its streaming inference is small, so it is
reimplemented here with the same models and the same numbers (checked against
the package's scores, see test/test_wakeword.py):

  every 80 ms (1280 samples at 16 kHz):
    melspectrogram.onnx on the newest 1280 + 480 samples -> 8 mel frames (x/10 + 2)
    embedding_model.onnx on the newest 76 mel frames     -> one 96-value embedding
    each wake word model on its newest N embeddings      -> a score 0..1
"""

import numpy as np
import onnxruntime as ort

CHUNK = 1280            # samples per step (80 ms)
_MEL_CONTEXT = 480      # extra samples the mel model needs (3 frames of 160)
_MEL_WINDOW = 76        # mel frames per embedding
_MAX_MEL = 970          # ~10 s
_MAX_FEATURES = 120     # ~10 s
_WARMUP = 5             # first scores after a reset are forced to 0 (as openWakeWord)


def _session(path):
    options = ort.SessionOptions()
    options.inter_op_num_threads = 1
    options.intra_op_num_threads = 1
    return ort.InferenceSession(str(path), sess_options=options,
                                providers=['CPUExecutionProvider'])


class WakeWords:

    def __init__(self, models, melspectrogram, embedding):
        """Load the models. models: {name: path to a wake word .onnx}."""
        self.mel = _session(melspectrogram)
        self.embed = _session(embedding)
        self.models = {name: _session(path) for name, path in models.items()}
        self.inputs = {name: (s.get_inputs()[0].name, s.get_inputs()[0].shape[1])
                       for name, s in self.models.items()}
        # openWakeWord starts its feature buffer from the embeddings of 4 s of noise
        noise = np.random.default_rng(0).integers(-1000, 1000, 16000 * 4).astype(np.int16)
        self._noise_features = self._embeddings_of_clip(noise)
        self.reset()

    def reset(self):
        self.raw = np.zeros(0, dtype=np.int16)
        self.mel_buffer = np.ones((_MEL_WINDOW, 32), dtype=np.float32)
        self.features = self._noise_features.copy()
        self.steps = 0

    def _melspectrogram(self, samples):
        out = self.mel.run(None, {'input': samples.astype(np.float32)[None, :]})[0]
        return np.squeeze(out) / 10.0 + 2.0

    def _embed(self, windows):
        return self.embed.run(None, {'input_1': windows.astype(np.float32)})[0].reshape(-1, 96)

    def _embeddings_of_clip(self, samples):
        mel = self._melspectrogram(samples)
        windows = [mel[i:i + _MEL_WINDOW] for i in range(0, mel.shape[0], 8)
                   if i + _MEL_WINDOW <= mel.shape[0]]
        return self._embed(np.array(windows)[..., None])

    def process(self, chunk):
        """Feed 1280 int16 samples; return {wake word name: score}."""
        chunk = np.asarray(chunk, dtype=np.int16)
        if chunk.shape != (CHUNK,):
            raise ValueError(f'need {CHUNK} samples, got {chunk.shape}')
        self.raw = np.concatenate((self.raw, chunk))[-(CHUNK + _MEL_CONTEXT):]
        self.mel_buffer = np.vstack((self.mel_buffer, self._melspectrogram(self.raw)))[-_MAX_MEL:]
        window = self.mel_buffer[-_MEL_WINDOW:][None, :, :, None]
        self.features = np.vstack((self.features, self._embed(window)))[-_MAX_FEATURES:]
        self.steps += 1
        scores = {}
        for name, session in self.models.items():
            input_name, n = self.inputs[name]
            x = self.features[-n:][None].astype(np.float32)
            score = float(np.squeeze(session.run(None, {input_name: x})[0]))
            scores[name] = 0.0 if self.steps < _WARMUP else score
        return scores
