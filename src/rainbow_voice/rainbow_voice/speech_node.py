"""Text to speech with espeak-ng.

Interfaces:
  ~/say              rainbow_interfaces/srv/Say   returns once spoken
  /speech/speak      std_msgs/String              say this (kept from the old node)
  /speech/amplitude  std_msgs/UInt16              set the volume, 0-200
  /speech/speaking   std_msgs/Bool (latched)      true while speaking (the voice
                                                  node does not listen meanwhile)
"""

import subprocess

from rainbow_interfaces.srv import Say
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool, String, UInt16


class SpeechNode(Node):

    def __init__(self):
        super().__init__('speech')
        self.voice = self.declare_parameter('voice', 'mb-us2').value
        self.speed = self.declare_parameter('speed', 120).value
        self.word_gap = self.declare_parameter('word_gap', 0.9).value
        self.device = self.declare_parameter('audio_device', 'plughw:2,0').value
        self.amplitude = self.declare_parameter('amplitude', 40).value

        self.create_subscription(String, '/speech/speak', lambda m: self.speak(m.data), 10)
        self.create_subscription(UInt16, '/speech/amplitude', self.on_amplitude, 10)
        self.create_service(Say, '~/say', self.on_say)
        self.speaking_pub = self.create_publisher(
            Bool, '/speech/speaking',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.speaking_pub.publish(Bool(data=False))

    def speak(self, text):
        """Speak text; returns an error message, or '' on success."""
        cmd = ['espeak-ng', '-v', self.voice, '-s', str(self.speed),
               '-g', str(self.word_gap), '-d', self.device,
               '-a', str(self.amplitude), text]
        self.speaking_pub.publish(Bool(data=True))
        try:
            subprocess.run(cmd, check=True, capture_output=True, timeout=60)
            return ''
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired, FileNotFoundError) as e:
            stderr = getattr(e, 'stderr', b'') or b''
            error = f'{e} {stderr.decode(errors="replace").strip()}'.strip()
            self.get_logger().error(f'espeak-ng failed: {error}')
            return error
        finally:
            self.speaking_pub.publish(Bool(data=False))

    def on_amplitude(self, msg):
        self.amplitude = msg.data

    def on_say(self, request, response):
        if request.volume:
            self.amplitude = request.volume
        error = self.speak(request.text)
        response.success = not error
        response.message = error or 'ok'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SpeechNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
