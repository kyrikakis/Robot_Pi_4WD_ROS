"""Velocity watchdog between Nav2/teleop and the ESP motor controller.

Republishes the latest command on cmd_vel at a fixed rate, and sends zero
velocity as soon as commands stop arriving, so the robot never keeps
driving on a stale command.

Subscribes: cmd_vel_stamped (geometry_msgs/Twist)  Nav2's collision monitor
            output; the name is historical, it is not a stamped message.
Publishes:  cmd_vel (geometry_msgs/Twist)          to the ESP board
"""

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


class CmdVelWatchdog(Node):

    def __init__(self):
        super().__init__('cmd_vel_watchdog')
        rate = self.declare_parameter('rate', 10.0).value
        self.timeout = self.declare_parameter('timeout', 0.2).value

        self.create_subscription(Twist, 'cmd_vel_stamped', self.on_cmd, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.last_cmd = Twist()
        self.last_time = None
        self.stopped = True
        self.create_timer(1.0 / rate, self.tick)

    def on_cmd(self, msg):
        self.last_cmd = msg
        self.last_time = self.get_clock().now()

    def tick(self):
        now = self.get_clock().now()
        fresh = (self.last_time is not None
                 and (now - self.last_time).nanoseconds * 1e-9 <= self.timeout)
        if fresh:
            self.pub.publish(self.last_cmd)
            self.stopped = False
        else:
            if not self.stopped:
                self.get_logger().info('no velocity commands, stopping')
                self.stopped = True
            self.pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelWatchdog()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
