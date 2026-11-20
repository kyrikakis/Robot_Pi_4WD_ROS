"""Velocity watchdog between Nav2/teleop and the ESP motor controller.

Republishes the latest command on cmd_vel at a fixed rate, and sends zero
velocity as soon as commands stop arriving, so the robot never keeps
driving on a stale command.

angular_gain scales the turn rate sent to the ESP: the skid-steer robot turns
slower than commanded (wheel slip), so a gain > 1 makes the actual turn rate
match what Nav2 or teleop asked for.

Tight arcs become turns in place: when the inner wheels would run slower than
min_inner_ratio of the outer wheels (the robot pivots on one side, both sides
scrub, and the wheel odometry reports forward motion the robot does not make),
the forward speed is dropped and the robot turns in place instead. It drives
on once the arc opens up again (above min_inner_ratio + hysteresis). 0 turns
this off.

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
        self.angular_gain = self.declare_parameter('angular_gain', 1.0).value
        self.track_width = self.declare_parameter('track_width', 0.12).value
        self.min_inner_ratio = self.declare_parameter('min_inner_ratio', 0.3).value
        self.hysteresis = self.declare_parameter('hysteresis', 0.1).value
        self.in_place = False

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
            out = Twist()
            out.linear.x = self.last_cmd.linear.x
            out.linear.y = self.last_cmd.linear.y
            out.linear.z = self.last_cmd.linear.z
            out.angular.x = self.last_cmd.angular.x
            out.angular.y = self.last_cmd.angular.y
            out.angular.z = self.last_cmd.angular.z * self.angular_gain
            if self.too_tight(out.linear.x, out.angular.z):
                out.linear.x = 0.0
            self.pub.publish(out)
            self.stopped = False
        else:
            if not self.stopped:
                self.get_logger().info('no velocity commands, stopping')
                self.stopped = True
            self.pub.publish(Twist())


    def too_tight(self, v, w):
        """Whether the arc is so tight that the inner wheels nearly stop."""
        if self.min_inner_ratio <= 0.0 or v == 0.0 or w == 0.0:
            self.in_place = False
            return False
        turn = abs(w) * self.track_width / 2.0
        ratio = (abs(v) - turn) / (abs(v) + turn)   # inner / outer wheel speed
        limit = self.min_inner_ratio + (self.hysteresis if self.in_place else 0.0)
        if (ratio < limit) != self.in_place:
            self.in_place = ratio < limit
            self.get_logger().debug('turning in place' if self.in_place else 'driving on')
        return self.in_place


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
