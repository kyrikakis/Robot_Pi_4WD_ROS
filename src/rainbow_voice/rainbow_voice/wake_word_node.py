"""Wake word ("Hey Rainbow"): placeholder, not implemented yet.

Planned design:
  - listen to the microphone and detect "Hey Rainbow" (e.g. openWakeWord,
    with a custom model trained for the phrase)
  - publish /voice/wake (std_msgs/Empty) when it is heard
  - speech to text then runs with Whisper on the Hailo, outside this node

Started with voice.launch.py wake_word:=true. Until it is implemented the
node only logs that it is a placeholder and keeps running, so the launch
slot and topic name are fixed in advance.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty


class WakeWordNode(Node):

    def __init__(self):
        super().__init__('wake_word')
        self.wake_pub = self.create_publisher(Empty, '/voice/wake', 10)
        self.get_logger().warning('wake word detection is not implemented yet (placeholder)')


def main(args=None):
    rclpy.init(args=args)
    node = WakeWordNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
