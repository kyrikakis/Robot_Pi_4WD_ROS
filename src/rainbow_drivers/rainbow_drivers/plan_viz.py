"""Republish Nav2's global plan for Foxglove, with a frame on every pose.

The Theta* planner sets the frame only on the path's header, not on its poses;
Foxglove rejects such a path ("poses must all have the same frame_id"). Nav2
itself uses the path header and does not care. This copies the header onto
each pose and republishes.

Subscribes: plan (nav_msgs/Path)
Publishes:  plan_viz (nav_msgs/Path)
"""

from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node


class PlanViz(Node):

    def __init__(self):
        super().__init__('plan_viz')
        self.pub = self.create_publisher(Path, 'plan_viz', 1)
        self.create_subscription(Path, 'plan', self.on_plan, 1)

    def on_plan(self, msg):
        for pose in msg.poses:
            pose.header = msg.header
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PlanViz()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
