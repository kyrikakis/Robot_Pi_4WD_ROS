"""Local commands recognised on the Pi with Vosk, from a fixed phrase list.

The brain decides which phrases exist (it knows the places) and publishes them
on /voice/grammar; the voice node only recognises. Vosk with a grammar maps
any speech to the closest phrase, so "[unk]" is in the grammar and a match
also needs a high confidence (checked by the brain).
"""

import json


class CommandRecognizer:

    def __init__(self, model_path):
        import vosk
        vosk.SetLogLevel(-1)
        self._vosk = vosk
        self.model = vosk.Model(str(model_path))
        self.phrases = []
        self.heard = ''   # the last recognition as Vosk heard it, for the log

    def set_phrases(self, phrases):
        self.phrases = [p for p in phrases if p]

    def recognize(self, samples):
        """Return (phrase, confidence) for 16 kHz int16 samples; ('', 0.0) if none."""
        self.heard = ''
        if not self.phrases:
            return '', 0.0
        grammar = json.dumps(self.phrases + ['[unk]'])
        rec = self._vosk.KaldiRecognizer(self.model, 16000, grammar)
        rec.SetWords(True)
        rec.AcceptWaveform(samples.astype('<i2').tobytes())
        result = json.loads(rec.FinalResult())
        words = [w for w in result.get('result') or [] if w.get('word')]
        self.heard = ' '.join(f'{w["word"]} {w.get("conf", 0.0):.2f}' for w in words)
        # One trailing unknown word is fine ("go to the sofa please"); more means a
        # longer sentence, which is for the agent. One leading one too: the tail of the
        # wake word ("...bow, stop"), as the recording starts where the wake word fired.
        if words and words[-1]['word'] == '[unk]':
            words = words[:-1]
        if words and words[0]['word'] == '[unk]':
            words = words[1:]
        text = ' '.join(w['word'] for w in words)
        if not words or text not in self.phrases:
            return '', 0.0
        return text, min(w.get('conf', 0.0) for w in words)
