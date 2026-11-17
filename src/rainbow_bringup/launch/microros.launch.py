"""micro-ROS agent: the bridge to the ESP board.

The ESP runs the motors, servos, lidar, IMU, odometry and battery reading.

Run as its own supervisor program (rainbow_microros): the ESP firmware does not
reconnect when the agent restarts, so the agent must not restart along with the
base. If the agent does restart, reset the ESP (battery switch off and on).

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
    ])
