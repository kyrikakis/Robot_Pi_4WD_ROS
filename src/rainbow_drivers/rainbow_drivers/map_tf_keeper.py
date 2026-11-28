"""Keeps the map frame alive while the lidar is in standby.

AMCL publishes map -> odom only when it processes a scan. The lidar is frugal
(lidar_power) and idles at rest, so without scans the transform ages out of
every TF buffer (~10 s) and the map frame disappears: Nav2 then aborts its
bringup ("Timed out waiting for transform from base_link to map") and cannot
plan a goal sent while the robot rests.

This node remembers AMCL's last map -> odom and, while AMCL is silent for
silence_s, republishes it at rate_hz stamped now + transform_tolerance_s (as
AMCL does). The pose does not drift at rest: odom -> base keeps moving with the
wheels and IMU, and AMCL takes over again with the first scan. slam_toolbox
does the same with its last transform.

Until AMCL has published once (at start), there is nothing to hold, so it asks
for the lidar on /lidar/wanted (once a second) until the first map -> odom
arrives.

Run it only with AMCL (localization.launch.py), never with slam_toolbox.

Parameters:
  silence_s              1.0
  rate_hz                10.0
  transform_tolerance_s  0.5   keep equal to AMCL's transform_tolerance
"""

from collections import deque

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage

MAP_FRAME = 'map'
ODOM_FRAME = 'odom'


def find_map_odom(msg, own_stamps):
    """Return AMCL's map -> odom in a TF message (not one this node sent), or None."""
    for t in msg.transforms:
        if (t.header.frame_id.lstrip('/') == MAP_FRAME
                and t.child_frame_id.lstrip('/') == ODOM_FRAME
                and (t.header.stamp.sec, t.header.stamp.nanosec) not in own_stamps):
            return t
    return None


class MapTfKeeper(Node):
    """Republishes AMCL's last map -> odom while AMCL is silent."""

    def __init__(self):
        super().__init__('map_tf_keeper')
        self.silence = self.declare_parameter('silence_s', 1.0).value
        rate = self.declare_parameter('rate_hz', 10.0).value
        self.tolerance = Duration(
            seconds=self.declare_parameter('transform_tolerance_s', 0.5).value)

        self.tf_pub = self.create_publisher(TFMessage, '/tf', 100)
        self.wanted_pub = self.create_publisher(Bool, 'lidar/wanted', 1)
        self.create_subscription(TFMessage, '/tf', self.on_tf, 100)

        self.last = None           # AMCL's last map -> odom
        self.last_at = None        # when it arrived (node clock)
        self.holding = False
        self.own_stamps = deque(maxlen=50)
        self.create_timer(1.0 / rate, self.tick)
        self.create_timer(1.0, self.ask_lidar)

    def on_tf(self, msg):
        """Remember AMCL's map -> odom."""
        t = find_map_odom(msg, self.own_stamps)
        if t is None:
            return
        if self.last is None:
            self.get_logger().info('first map -> odom from AMCL; holding it while it is silent')
        self.last = t
        self.last_at = self.get_clock().now()
        if self.holding:
            self.get_logger().info('AMCL publishes again')
            self.holding = False

    def tick(self):
        """Republish the last transform while AMCL is silent."""
        if self.last is None:
            return
        now = self.get_clock().now()
        if (now - self.last_at).nanoseconds * 1e-9 < self.silence:
            return
        if not self.holding:
            self.get_logger().info('AMCL silent (lidar in standby?); holding map -> odom')
            self.holding = True
        t = TransformStamped()
        t.header.frame_id = self.last.header.frame_id
        t.child_frame_id = self.last.child_frame_id
        t.transform = self.last.transform
        t.header.stamp = (now + self.tolerance).to_msg()
        self.own_stamps.append((t.header.stamp.sec, t.header.stamp.nanosec))
        self.tf_pub.publish(TFMessage(transforms=[t]))

    def ask_lidar(self):
        """Ask for the lidar until AMCL has published its first transform."""
        if self.last is None:
            self.wanted_pub.publish(Bool(data=True))


def main():
    rclpy.init()
    node = MapTfKeeper()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
