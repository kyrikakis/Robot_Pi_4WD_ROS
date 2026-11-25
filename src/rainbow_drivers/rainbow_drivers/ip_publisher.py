"""Publishes the Pi's LAN IP address and CPU temperature once a second.

  rpi5_ip           std_msgs/String
  rpi5_temperature  sensor_msgs/Temperature   degrees C (cpu-thermal; the Pi 5
                                              throttles from 80-85)
"""

import socket

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature
from std_msgs.msg import String


def local_ip():
    """IP of the interface used for outbound traffic, or '' when offline."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(('8.8.8.8', 80))  # UDP connect sends nothing
            return s.getsockname()[0]
    except OSError:
        return ''


def cpu_temperature():
    """CPU temperature in degrees C, or None if it cannot be read."""
    try:
        with open('/sys/class/thermal/thermal_zone0/temp') as f:
            return int(f.read()) / 1000.0
    except (OSError, ValueError):
        return None


class IpPublisher(Node):

    def __init__(self):
        super().__init__('ip_publisher')
        self.pub = self.create_publisher(String, 'rpi5_ip', 10)
        self.temp_pub = self.create_publisher(Temperature, 'rpi5_temperature', 10)
        self.create_timer(1.0, self.publish)

    def publish(self):
        self.pub.publish(String(data=local_ip()))
        celsius = cpu_temperature()
        if celsius is not None:
            msg = Temperature(temperature=celsius)
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            self.temp_pub.publish(msg)


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
