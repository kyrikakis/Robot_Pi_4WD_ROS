from threading import Thread
import rclpy
from rclpy.node import Node

from example_interfaces.msg import Int32MultiArray
from std_msgs.msg import Int32

import numpy as np
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped

SEND_INTERVAL_SEC = 0.25
MIN_RECEIVED_INTERVAL_SEC = 0.25
Y_MIN = -19
Y_MAX = 36
Y_OFFSET = -56
Z_MIN = -38
Z_MAX = 40
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
    
class HeadMovement(Node):

    def __init__(self):
        super().__init__('head_movement')
        self.subscription = self.create_subscription(
            Int32MultiArray,
            '/head/yz_rotation',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.y_publisher = self.create_publisher(Int32, 'servo_s2', 50)
        self.z_publisher = self.create_publisher(Int32, 'servo_s1', 50)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        self.y_rotation = 0
        self.z_rotation = 0

    def listener_callback(self, msg):
        y_rotation = msg.data[0]
        z_rotation = msg.data[1]
        y_rotation = cap_to_y_limits(y_rotation)
        z_rotation = cap_to_z_limits(z_rotation)
        self.move_head(y_rotation, z_rotation)
        self.y_rotation = y_rotation
        self.z_rotation = z_rotation
        self.publish_transform()

    def move_head(self, y_rotation, z_rotation):
        y_msg = Int32()
        z_msg = Int32()
        y_msg.data = y_rotation + Y_OFFSET
        z_msg.data = z_rotation + Z_OFFSET
        self.y_publisher.publish(y_msg)
        self.z_publisher.publish(z_msg)

    def publish_transform(self):
        t = TransformStamped()
        t.transform.rotation.y = np.radians(self.y_rotation)
        t.transform.rotation.z = np.radians(self.z_rotation)
        self.tf_static_broadcaster.sendTransform(t)
            

def main(args=None):
    rclpy.init(args=args)

    head_movement = HeadMovement()

    rclpy.spin(head_movement)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    head_movement.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()