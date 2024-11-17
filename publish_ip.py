import socket
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class LocalIPPublisher(Node):
    def __init__(self):
        super().__init__('local_ip_publisher')
        self.publisher_ = self.create_publisher(String, 'rpi5_ip', 10)
        timer_period = 1  # publish every 1 second
        self.timer = self.create_timer(timer_period, self.publish_local_ip)

    def get_local_ip(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip_address = s.getsockname()[0]
        s.close()
        return ip_address

    def publish_local_ip(self):
        ip_address = self.get_local_ip()
        msg = String()
        msg.data = ip_address
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    local_ip_publisher = LocalIPPublisher()
    rclpy.spin(local_ip_publisher)
    local_ip_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

