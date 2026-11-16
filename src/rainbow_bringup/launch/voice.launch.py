"""Rainbow's voice: text to speech."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='rainbow_voice', executable='speech'),
    ])
