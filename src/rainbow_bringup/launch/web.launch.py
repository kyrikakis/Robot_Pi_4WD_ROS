"""The phone remote (rainbow_web): a web page on port 80, http://rainbow.local/."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='rainbow_web', executable='web', output='screen'),
    ])
