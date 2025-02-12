import rclpy
import math
from rclpy.node import Node
from threading import Thread

from std_msgs.msg import Int32

import numpy as np
from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import time

SEND_INTERVAL_SEC = 0.20
Y_MIN = -36
Y_MAX = 19
Y_OFFSET = -56
Z_MIN = -40
Z_MAX = 38
Z_OFFSET = 0

def cap_to_y_limits(y_rotation):
    if(y_rotation < Y_MIN):
        return Y_MIN
    elif(y_rotation > Y_MAX):
        return Y_MAX
    else:
        return y_rotation

def cap_to_z_limits(z_rotation):
    if(z_rotation < Z_MIN):
        return Z_MIN
    elif(z_rotation > Z_MAX):
        return Z_MAX
    else:
        return z_rotation

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q
    
class HeadMovement(Node):

    def __init__(self):
        super().__init__('head_movement')
        self.y_subscription = self.create_subscription(
            Int32,
            '/head/y_rotation',
            self.y_callback,
            10)
        self.z_subscription = self.create_subscription(
            Int32,
            '/head/z_rotation',
            self.z_callback,
            10)
        self.y_publisher = self.create_publisher(Int32, 'servo_s2', 10)
        self.z_publisher = self.create_publisher(Int32, 'servo_s1', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.y_rotation = 0
        self.z_rotation = 0
        self.move_y(self.y_rotation)
        self.move_z(self.z_rotation)
        self.move_y(self.y_rotation)
        self.move_z(self.z_rotation)

    def y_callback(self, msg):
        y_rotation = msg.data
        y_rotation = cap_to_y_limits(y_rotation)
        self.y_rotation = y_rotation
        self.move_y(y_rotation)

    def z_callback(self, msg):
        z_rotation = msg.data
        z_rotation = cap_to_z_limits(z_rotation)
        self.z_rotation = z_rotation
        self.move_z(z_rotation)

    def move_y(self, y_rotation):
        y_msg = Int32()
        y_msg.data = y_rotation*-1 + Y_OFFSET #servo has the rotation sign reversed thus the *-1
        self.y_publisher.publish(y_msg)

    def move_z(self, z_rotation):
        z_msg = Int32()
        z_msg.data = z_rotation*-1 + Z_OFFSET #servo has the rotation sign reversed thus the *-1
        self.z_publisher.publish(z_msg)

    def publish_transform(self):
        while(True):
            time.sleep(SEND_INTERVAL_SEC)
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'base_link'
            t.child_frame_id = 'depth_frame'
            t.transform.translation.x = 0.10
            t.transform.translation.y = 0.05
            t.transform.translation.z = 0.06
            quat = quaternion_from_euler(0.0, np.radians(self.y_rotation), np.radians(self.z_rotation))
            t.transform.rotation.x = quat[0]
            t.transform.rotation.y = quat[1]
            t.transform.rotation.z = quat[2]
            t.transform.rotation.w = quat[3]
            self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)

    head_movement = HeadMovement()
    thread = Thread(target = HeadMovement.publish_transform, args=(head_movement,))
    thread.start()

    rclpy.spin(head_movement)
    thread.join()
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    head_movement.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()