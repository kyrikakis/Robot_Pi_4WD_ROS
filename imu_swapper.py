import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class IMUSwapper(Node):
    def __init__(self):
        super().__init__('imu_swapper')
        self.subscription = self.create_subscription(
            Imu,
            '/imu',  # Replace with your IMU topic
            self.imu_callback,
            10
        )
        self.publisher = self.create_publisher(Imu, 'imu/corrected', 10)

    def imu_callback(self, msg):
        # Swap X and Y in angular velocity
        angular_velocity_x = msg.angular_velocity.x
        angular_velocity_y = msg.angular_velocity.y
        msg.angular_velocity.x = angular_velocity_y
        msg.angular_velocity.y = -abs(angular_velocity_x)

        # Swap X and Y in linear acceleration
        linear_acceleration_x = msg.linear_acceleration.x
        linear_acceleration_y = msg.linear_acceleration.y
        msg.linear_acceleration.x = linear_acceleration_y
        msg.linear_acceleration.y = -abs(linear_acceleration_x)

        # Publish the corrected IMU message
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    imu_swapper = IMUSwapper()
    rclpy.spin(imu_swapper)
    imu_swapper.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
