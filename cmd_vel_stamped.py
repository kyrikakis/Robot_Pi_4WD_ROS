from threading import Thread
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
import time

SEND_INTERVAL_SEC = 0.25
MIN_RECEIVED_INTERVAL_SEC = 0.25

class CmdVelStamped(Node):

    def __init__(self):
        super().__init__('cmd_vel_stamped')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel_stamped',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.twist = Twist()
        self.last_received = time.time()
        self.last_send = time.time()

    def listener_callback(self, msg):
        self.twist = msg
        self.last_received = time.time()

    
    def check_twist(self):
        while(True):
            send_time_elapsed = time.time() - self.last_send 
            time.sleep(SEND_INTERVAL_SEC)
            self.last_send = time.time()
            received_time_leapsed = time.time() - self.last_received
            if(received_time_leapsed > MIN_RECEIVED_INTERVAL_SEC):
                self.get_logger().info(f'time passed: {send_time_elapsed}, linear.x: zero')
                zero_twist = Twist()
                self.publisher.publish(zero_twist)
            else:
                self.publisher.publish(self.twist)
                self.get_logger().info(f'time passed: {send_time_elapsed}, linear.x: {self.twist.linear.x}')




def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = CmdVelStamped()
    thread = Thread(target = CmdVelStamped.check_twist, args=(minimal_subscriber,))
    thread.start()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()