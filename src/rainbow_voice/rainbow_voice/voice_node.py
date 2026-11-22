"""Rainbow's ears: wake word, stop word, utterances and local commands (docs/brain.md).

Reads the ReSpeaker Lite with arecord (16 kHz, 16-bit; no Python audio
library needed) and runs rainbow_voice.listener on every 80 ms chunk:
openWakeWord models (rainbow_voice.wakeword) for the wake and stop words,
webrtcvad for the end of speech. A finished utterance goes through Vosk with
the brain's phrase list (rainbow_voice.commands) and is published for the
brain, which handles local commands itself and sends the rest to the Mac.

The wake word is a placeholder ("hey Jarvis", openWakeWord's pretrained model)
until a "Rainbow" model is trained; there is no stop model yet, so "stop" only
works after the wake word (as a local command). The capture device can only be
opened by one process: nothing else may record from it.

On the wake word it plays a short quiet beep on the same card (wake_beep,
beep_volume); the recording skips the beep's 0.16 s, since the mic hears it.

Publishes:  /voice/wake (std_msgs/Empty), /voice/stop (std_msgs/Empty),
            /voice/utterance (rainbow_interfaces/Utterance), /eyes (std_msgs/String)
Subscribes: /speech/speaking (std_msgs/Bool, latched): no listening while talking
            /voice/grammar (std_msgs/String, latched): JSON list of command phrases
"""

import array
import json
from pathlib import Path
import subprocess
import threading
import time

import numpy as np
from rainbow_interfaces.msg import Utterance
from rainbow_voice.commands import CommandRecognizer
from rainbow_voice.listener import CHUNK, Listener, Vad
from rainbow_voice.wakeword import WakeWords
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Empty, String

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
EYES = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST,
                  depth=1)


def beep_samples(volume, seconds=0.12, hz=880):
    """Return a short sine beep (16 kHz mono PCM bytes) with soft 10 ms edges."""
    t = np.arange(int(16000 * seconds)) / 16000.0
    wave = np.sin(2 * np.pi * hz * t)
    edge = np.minimum(1.0, np.minimum(t, seconds - t) / 0.01)
    return (wave * edge * volume * 32767).astype('<i2').tobytes()


class VoiceNode(Node):

    def __init__(self):
        super().__init__('voice')
        p = self.declare_parameter
        self.device = p('device', 'plughw:CARD=Lite,DEV=0').value
        self.channels = p('channels', 2).value
        self.channel = p('channel', 0).value
        models = Path(p('models_dir', '/opt/rainbow/voice').value)
        wake_model = p('wake_model', 'hey_jarvis_v0.1.onnx').value
        stop_model = p('stop_model', '').value

        self.wake_beep = p('wake_beep', True).value
        self.beep = beep_samples(p('beep_volume', 0.15).value)

        words = {'wake': models / wake_model}
        if stop_model:
            words['stop'] = models / stop_model
        self.wake = WakeWords(words, models / 'melspectrogram.onnx',
                              models / 'embedding_model.onnx')
        self.listener = Listener(
            self.wake.process, Vad(p('vad_aggressiveness', 2).value), 'wake',
            'stop' if stop_model else None,
            wake_threshold=p('wake_threshold', 0.5).value,
            stop_threshold=p('stop_threshold', 0.5).value,
            end_silence=p('end_silence', 0.8).value,
            max_seconds=p('max_seconds', 10.0).value,
            follow_up=p('follow_up', 8.0).value,
            skip_after_wake=0.16 if self.wake_beep else 0.0)
        self.commands = CommandRecognizer(
            models / p('vosk_model', 'vosk-model-small-en-us-0.15').value)
        self.lock = threading.Lock()
        self.engaged = False   # woke and not yet back to happy eyes
        self.engaged_since = 0.0
        self.eyes_timeout = p('eyes_timeout', 15.0).value

        self.wake_pub = self.create_publisher(Empty, '/voice/wake', 10)
        self.stop_pub = self.create_publisher(Empty, '/voice/stop', 10)
        self.utterance_pub = self.create_publisher(Utterance, '/voice/utterance', 10)
        self.eyes_pub = self.create_publisher(String, '/eyes', EYES)
        self.create_subscription(Bool, '/speech/speaking', self.on_speaking, LATCHED)
        self.create_subscription(String, '/voice/grammar', self.on_grammar, LATCHED)
        # Back to happy eyes if no answer is ever spoken (agent away, no speech node)
        self.create_timer(1.0, self.check_eyes)

        self.arecord = None
        self.running = True
        self.thread = threading.Thread(target=self.audio_loop, name='audio', daemon=True)
        self.thread.start()
        self.get_logger().info(
            f'listening on {self.device} (wake: {wake_model}, stop: {stop_model or "none"})')

    def on_speaking(self, msg):
        with self.lock:
            self.listener.speaking(msg.data)
        if not msg.data and self.engaged:
            self.engaged = False
            self.eyes('happy')

    def check_eyes(self):
        if (self.engaged and not self.listener.muted
                and time.monotonic() - self.engaged_since > self.eyes_timeout):
            self.engaged = False
            self.eyes('happy')

    def on_grammar(self, msg):
        try:
            phrases = json.loads(msg.data)
        except ValueError:
            self.get_logger().warning('bad /voice/grammar message')
            return
        with self.lock:
            self.commands.set_phrases(phrases)
        self.get_logger().info(f'{len(phrases)} command phrases')

    def eyes(self, expression):
        self.eyes_pub.publish(String(data=expression))

    def play_beep(self):
        """Play the wake beep without waiting; skipped if the card is busy (espeak)."""
        def play():
            try:
                subprocess.run(['aplay', '-q', '-D', self.device, '-f', 'S16_LE', '-r', '16000',
                                '-c', '1', '-t', 'raw'], input=self.beep, timeout=2,
                               capture_output=True)
            except (OSError, subprocess.TimeoutExpired):
                pass
        threading.Thread(target=play, daemon=True).start()

    # Audio

    def audio_loop(self):
        frame_bytes = CHUNK * self.channels * 2
        while self.running and rclpy.ok():
            cmd = ['arecord', '-q', '-D', self.device, '-f', 'S16_LE', '-r', '16000',
                   '-c', str(self.channels), '-t', 'raw']
            try:
                self.arecord = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                                stderr=subprocess.PIPE)
            except OSError as e:
                self.get_logger().error(f'cannot run arecord: {e}')
                time.sleep(5.0)
                continue
            while self.running:
                data = self.arecord.stdout.read(frame_bytes)
                if len(data) < frame_bytes:
                    break
                samples = np.frombuffer(data, dtype='<i2').reshape(-1, self.channels)
                self.on_chunk(np.ascontiguousarray(samples[:, self.channel]))
            if self.running:
                err = self.arecord.stderr.read().decode(errors='replace').strip()
                self.get_logger().warning(f'audio stopped ({err or "end of stream"}); retrying')
                self.arecord.kill()
                time.sleep(2.0)

    def on_chunk(self, chunk):
        with self.lock:
            events = self.listener.feed(chunk)
        for event in events:
            kind = event[0]
            if kind == 'wake':
                if self.wake_beep and not event[1]:
                    self.play_beep()
                self.engaged, self.engaged_since = True, time.monotonic()
                self.wake_pub.publish(Empty())
                self.eyes('open')
            elif kind == 'stop':
                self.get_logger().info('stop word')
                self.stop_pub.publish(Empty())
            elif kind == 'nothing':
                self.engaged = False
                self.eyes('happy')
            elif kind == 'utterance':
                self.publish_utterance(event[1], event[2])

    def publish_utterance(self, samples, follow_up):
        with self.lock:
            command, confidence = self.commands.recognize(samples)
        msg = Utterance(sample_rate=16000, follow_up=follow_up, command=command,
                        command_confidence=float(confidence))
        msg.stamp = self.get_clock().now().to_msg()
        msg.samples = array.array('h', samples.astype('<i2').tobytes())
        self.utterance_pub.publish(msg)
        self.get_logger().info(
            f'utterance {len(samples) / 16000:.1f} s{" (follow-up)" if follow_up else ""}'
            f'{f", command {command!r} {confidence:.2f}" if command else ""}')

    def destroy_node(self):
        self.running = False
        if self.arecord is not None:
            self.arecord.kill()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VoiceNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
