"""Publishes the Pi's LAN IP address once a second on rpi5_ip (std_msgs/String)."""

import socket

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def local_ip():
    """IP of the interface used for outbound traffic, or '' when offline."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(('8.8.8.8', 80))  # UDP connect sends nothing
            return s.getsockname()[0]
    except OSError:
        return ''


class IpPublisher(Node):

    def __init__(self):
        super().__init__('ip_publisher')
        self.pub = self.create_publisher(String, 'rpi5_ip', 10)
        self.create_timer(1.0, lambda: self.pub.publish(String(data=local_ip())))


def main(args=None):
    rclpy.init(args=args)
    node = IpPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
