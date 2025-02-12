import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import UInt16
import subprocess

class Speech(Node):

    def __init__(self):
        super().__init__('speech')
        self.speech_subscription = self.create_subscription(
            String,
            '/speech/speak',
            self.speak,
            10)
        self.amplitude_subscription = self.create_subscription(
            UInt16,
            '/speech/amplitude',
            self.set_amplitude,
            10)
        self.amplitude = 40
    
    def speak(self, msg):
        voice="mb-us2" 
        speed=120
        gap=0.9
        device="plughw:2,0"
        cmd = ["espeak-ng"] + (["-v", voice] if voice else []) + (["-s", str(speed)] if speed else []) \
            + (["-d", str(device)] if device else []) + (["-g", str(gap)] if gap else [])
        try:
            cmd += ["-a", str(self.amplitude), msg.data]
            subprocess.run(cmd, check=True, capture_output=True)
        except (subprocess.CalledProcessError, FileNotFoundError) as e: print(f"Error: {e}")

    def set_amplitude(self, msg):
        self.amplitude = msg.data

def main(args=None):
    rclpy.init(args=args)

    speech = Speech()

    rclpy.spin(speech)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    speech.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()