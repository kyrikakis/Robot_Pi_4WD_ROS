"""Rainbow's ears: wake word, stop word, utterances and local commands (docs/brain.md).

Reads the ReSpeaker Lite with arecord (16 kHz, 16-bit; no Python audio
library needed) and runs rainbow_voice.listener on every 80 ms chunk:
openWakeWord models (rainbow_voice.wakeword) for the wake words and the stop word,
webrtcvad for the end of speech. A finished utterance goes through Vosk with
the brain's phrase list (rainbow_voice.commands) and is published for the
brain, which handles local commands itself and sends the rest to the Mac.

Wake words (wake_models, any one wakes, each with its own threshold in
wake_thresholds): hey_rainbow.onnx answers "hey Rainbow" and "Rainbow" alone,
hey_robot.onnx "hey Robot" and "Robot" (the fallback name). The
stop word (stop_model, stop.onnx) is heard at any time, even while the robot talks;
"Rainbow, stop" also works, as a local command after the wake word. These models
were trained for Rainbow (training/wakeword/, synthetic voices) and live in this
package (models/); a name not found there is looked up in models_dir (openWakeWord's
own models, e.g. hey_jarvis_v0.1.onnx). The capture device can only be opened by one
process: nothing else may record from it.

On the wake word the eyes open. A short soft hum, a rising "hmm?", can play too
(wake_beep, beep_volume 0.35; off by default): the recording then skips its 0.32 s,
since the mic hears it, and that skip cut off commands said in one breath ("Rainbow,
stop": on synthetic voices 0-1 of 20 stopped with the hum, 17-18 without).

Publishes:  /voice/wake (std_msgs/Empty), /voice/stop (std_msgs/Empty),
            /voice/utterance (rainbow_interfaces/Utterance), /eyes (std_msgs/String)
Subscribes: /speech/speaking (std_msgs/Bool, latched): no listening while talking
            /voice/grammar (std_msgs/String, latched): JSON list of command phrases
            /cmd_vel (geometry_msgs/Twist): no follow-up recordings while the robot
            moves (motor noise started them)
"""

import array
import json
from pathlib import Path
import subprocess
import threading
import time

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from geometry_msgs.msg import Twist
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


def model_path(name, models_dir):
    """Return this package's model of that name (models/), else the one in models_dir."""
    try:
        own = Path(get_package_share_directory('rainbow_voice')) / 'models' / name
    except PackageNotFoundError:
        own = None
    return own if own is not None and own.exists() else models_dir / name


def beep_samples(volume, seconds=0.3, start_hz=180.0, end_hz=240.0):
    """Return a soft rising hum, "hmm?" (16 kHz mono PCM bytes).

    A low tone gliding up, with a second harmonic so it sounds voiced, fading
    in over 60 ms and out over 100 ms.
    """
    t = np.arange(int(16000 * seconds)) / 16000.0
    hz = start_hz + (end_hz - start_hz) * t / seconds
    phase = 2 * np.pi * np.cumsum(hz) / 16000.0
    wave = (np.sin(phase) + 0.35 * np.sin(2 * phase)) / 1.35
    edge = np.minimum(1.0, np.minimum(t / 0.06, (seconds - t) / 0.10))
    return (wave * edge * volume * 32767).astype('<i2').tobytes()


class VoiceNode(Node):

    def __init__(self):
        super().__init__('voice')
        p = self.declare_parameter
        self.device = p('device', 'plughw:CARD=Lite,DEV=0').value
        self.channels = p('channels', 2).value
        self.channel = p('channel', 0).value
        models = Path(p('models_dir', '/opt/rainbow/voice').value)
        wake_models = list(p('wake_models', ['hey_rainbow.onnx', 'hey_robot.onnx']).value)
        thresholds = list(p('wake_thresholds', [0.3, 0.3]).value)
        stop_model = p('stop_model', 'stop.onnx').value

        self.wake_beep = p('wake_beep', False).value
        self.beep = beep_samples(p('beep_volume', 0.35).value)

        # One threshold per wake model; a shorter list repeats its last value
        thresholds = [thresholds[min(i, len(thresholds) - 1)] for i in range(len(wake_models))]
        words = {f'wake:{m}': model_path(m, models) for m in wake_models}
        if stop_model:
            words['stop'] = model_path(stop_model, models)
        self.wake = WakeWords(words, models / 'melspectrogram.onnx',
                              models / 'embedding_model.onnx')
        # For the log: which model fired with what score, and near misses (a model
        # peaking at log_scores_above or more without reaching its threshold).
        self.thresholds = {f'wake:{m}': t for m, t in zip(wake_models, thresholds)}
        self.stop_threshold = p('stop_threshold', 0.7).value
        if stop_model:
            self.thresholds['stop'] = self.stop_threshold
        self.log_floor = p('log_scores_above', 0.1).value
        self.scores, self.peaks = {}, {}
        self.listener = Listener(
            self.score, Vad(p('vad_aggressiveness', 3).value),
            {f'wake:{m}': t for m, t in zip(wake_models, thresholds)},
            'stop' if stop_model else None,
            stop_threshold=self.stop_threshold,
            end_silence=p('end_silence', 0.8).value,
            max_seconds=p('max_seconds', 10.0).value,
            follow_up=p('follow_up', 8.0).value,
            follow_up_start=p('follow_up_start', 0.48).value,
            max_follow_ups=p('max_follow_ups', 2).value,
            min_speech=p('min_speech', 0.24).value,
            motion_settle=p('motion_settle', 0.5).value,
            skip_after_wake=0.32 if self.wake_beep else 0.0)
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
        self.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)
        # Back to happy eyes if no answer is ever spoken (agent away, no speech node)
        self.create_timer(1.0, self.check_eyes)

        self.arecord = None
        self.running = True
        self.thread = threading.Thread(target=self.audio_loop, name='audio', daemon=True)
        self.thread.start()
        self.get_logger().info(
            f'listening on {self.device} (wake: '
            f'{", ".join(f"{m} >= {t}" for m, t in zip(wake_models, thresholds))}, '
            f'stop: {stop_model or "none"})')

    def on_speaking(self, msg):
        with self.lock:
            self.listener.speaking(msg.data)
        if not msg.data and self.engaged:
            self.engaged = False
            self.eyes('happy')

    def on_cmd_vel(self, msg):
        if msg.linear.x or msg.linear.y or msg.angular.z:
            with self.lock:
                self.listener.moved()

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
        """Play the wake hum without waiting; skipped if the card is busy (espeak)."""
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

    def score(self, chunk):
        self.scores = self.wake.process(chunk)
        return self.scores

    def fired(self, names):
        """Log the model that fired (the highest score over its threshold)."""
        name = max(names, key=lambda n: self.scores.get(n, 0.0) / self.thresholds[n])
        self.peaks[name] = float('inf')   # no near miss for this one
        return f'{name.removeprefix("wake:")} {self.scores.get(name, 0.0):.2f}'

    def log_near_misses(self):
        for name, threshold in self.thresholds.items():
            score = self.scores.get(name, 0.0)
            if score >= self.log_floor:
                self.peaks[name] = max(self.peaks.get(name, 0.0), score)
            elif name in self.peaks:
                peak = self.peaks.pop(name)
                if peak < threshold:
                    self.get_logger().info(
                        f'near miss: {name.removeprefix("wake:")} {peak:.3f} (needs {threshold})')

    def on_chunk(self, chunk):
        with self.lock:
            events = self.listener.feed(chunk)
        for event in events:
            kind = event[0]
            if kind == 'wake':
                if self.wake_beep and not event[1]:
                    self.play_beep()
                if event[1]:
                    self.get_logger().info('follow-up')
                else:
                    self.get_logger().info('wake word ' + self.fired(
                        [n for n in self.thresholds if n != 'stop']))
                self.engaged, self.engaged_since = True, time.monotonic()
                self.wake_pub.publish(Empty())
                self.eyes('open')
            elif kind == 'stop':
                self.get_logger().info('stop word ' + self.fired(['stop']).split()[1])
                self.stop_pub.publish(Empty())
            elif kind == 'nothing':
                self.engaged = False
                self.eyes('happy')
            elif kind == 'utterance':
                self.publish_utterance(event[1], event[2])
        self.log_near_misses()

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
            + (f', command {command!r} {confidence:.2f}' if command
               else f', heard {self.commands.heard!r}'))

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
