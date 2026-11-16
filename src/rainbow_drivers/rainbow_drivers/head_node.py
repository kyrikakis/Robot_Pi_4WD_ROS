"""Pan/tilt head.

Drives the two head servos on the ESP board and publishes the head's
/joint_states, so robot_state_publisher moves every frame on the head
(camera, ToF, eyes) with it.

Interfaces:
  ~/set_pose      rainbow_interfaces/srv/SetHeadPose   pan/tilt in radians
  ~/center        std_srvs/srv/Trigger                 back to 0, 0
  /head/z_rotation, /head/y_rotation  std_msgs/Int32   pan/tilt in degrees
                                                       (kept from the old node)
  servo_s1, servo_s2  std_msgs/Int32 (out)             servo angles for the ESP
  /joint_states       sensor_msgs/JointState (out)
"""

import math

from rainbow_interfaces.srv import SetHeadPose
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32
from std_srvs.srv import Trigger


class HeadNode(Node):

    def __init__(self):
        super().__init__('head')

        # Limits are in degrees of head motion; servo_* map head degrees to the
        # servo command the ESP expects: servo = sign * head + offset
        self.pan_min = self.declare_parameter('pan_min_deg', -40.0).value
        self.pan_max = self.declare_parameter('pan_max_deg', 38.0).value
        self.tilt_min = self.declare_parameter('tilt_min_deg', -36.0).value
        self.tilt_max = self.declare_parameter('tilt_max_deg', 19.0).value
        self.pan_sign = self.declare_parameter('pan_servo_sign', -1).value
        self.pan_offset = self.declare_parameter('pan_servo_offset_deg', 0).value
        self.tilt_sign = self.declare_parameter('tilt_servo_sign', -1).value
        self.tilt_offset = self.declare_parameter('tilt_servo_offset_deg', -56).value
        self.pan_joint = self.declare_parameter('pan_joint', 'head_pan_joint').value
        self.tilt_joint = self.declare_parameter('tilt_joint', 'head_tilt_joint').value
        rate = self.declare_parameter('joint_state_rate', 10.0).value

        self.pan_pub = self.create_publisher(Int32, 'servo_s1', 10)
        self.tilt_pub = self.create_publisher(Int32, 'servo_s2', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        self.create_subscription(Int32, '/head/z_rotation', self.on_pan_deg, 10)
        self.create_subscription(Int32, '/head/y_rotation', self.on_tilt_deg, 10)
        self.create_service(SetHeadPose, '~/set_pose', self.on_set_pose)
        self.create_service(Trigger, '~/center', self.on_center)

        self.pan_deg = 0.0
        self.tilt_deg = 0.0
        self.send_servos()
        # The ESP may not be listening yet when we start: resend the pose for a
        # few seconds so the head ends up where we think it is.
        self.resend_count = 0
        self.resend_timer = self.create_timer(1.0, self.resend_initial_pose)
        self.create_timer(1.0 / rate, self.publish_joint_states)

    @staticmethod
    def clamp(value, low, high):
        return max(low, min(high, value))

    def set_pose_deg(self, pan_deg, tilt_deg):
        if pan_deg is not None:
            self.pan_deg = self.clamp(pan_deg, self.pan_min, self.pan_max)
        if tilt_deg is not None:
            self.tilt_deg = self.clamp(tilt_deg, self.tilt_min, self.tilt_max)
        self.send_servos()
        self.publish_joint_states()

    def send_servos(self):
        self.pan_pub.publish(Int32(data=int(round(
            self.pan_sign * self.pan_deg + self.pan_offset))))
        self.tilt_pub.publish(Int32(data=int(round(
            self.tilt_sign * self.tilt_deg + self.tilt_offset))))

    def resend_initial_pose(self):
        self.send_servos()
        self.resend_count += 1
        if self.resend_count >= 5:
            self.resend_timer.cancel()

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [self.pan_joint, self.tilt_joint]
        msg.position = [math.radians(self.pan_deg), math.radians(self.tilt_deg)]
        self.joint_pub.publish(msg)

    def on_pan_deg(self, msg):
        self.set_pose_deg(float(msg.data), None)

    def on_tilt_deg(self, msg):
        self.set_pose_deg(None, float(msg.data))

    def on_set_pose(self, request, response):
        self.set_pose_deg(math.degrees(request.pan), math.degrees(request.tilt))
        response.success = True
        response.pan = math.radians(self.pan_deg)
        response.tilt = math.radians(self.tilt_deg)
        response.message = f'pan {self.pan_deg:.0f} deg, tilt {self.tilt_deg:.0f} deg'
        return response

    def on_center(self, request, response):
        self.set_pose_deg(0.0, 0.0)
        response.success = True
        response.message = 'centered'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = HeadNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
