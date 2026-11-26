"""micro-ROS agent: the bridge to the ESP board.

The ESP runs the motors, servos, lidar, IMU, odometry and battery reading.
lidar_power (rainbow_drivers) turns the lidar on while something needs /scan.

Run as its own supervisor program (rainbow_microros), so restarting the base
does not drop the ESP. Our firmware (firmware/) reconnects by itself within
~10 s when the agent restarts; the stock Yahboom firmware needed a board reset.

Arguments:
  serial_port   /dev/ttyUSB_ROS
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB_ROS'),
        # The ESP board can be switched off; the agent exits when the serial
        # port is missing, so keep retrying until it appears.
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            arguments=['serial', '--dev', LaunchConfiguration('serial_port'),
                       '-b', '921600', '-v4'],
            respawn=True,
            respawn_delay=5.0,
        ),
        # Lidar on only while something needs /scan (the ESP boots it in standby).
        Node(
            package='rainbow_drivers',
            executable='lidar_power',
            name='lidar_power',
            respawn=True,
            respawn_delay=5.0,
        ),
    ])
