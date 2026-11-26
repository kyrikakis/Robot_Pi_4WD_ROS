"""Pan/tilt head (rainbow_drivers head node).

Its own supervisor program (rainbow_head): restarting it (e.g. to tune its speed
or settle times) does not restart the EKF, unlike restarting the base.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='rainbow_drivers', executable='head', respawn=True, respawn_delay=2.0),
    ])
