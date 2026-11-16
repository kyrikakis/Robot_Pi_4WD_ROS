"""Rainbow's voice.

Arguments:
  wake_word  false  start the "Hey Rainbow" wake word node (placeholder for now)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('wake_word', default_value='false'),
        Node(package='rainbow_voice', executable='speech'),
        Node(
            package='rainbow_voice',
            executable='wake_word',
            condition=IfCondition(LaunchConfiguration('wake_word')),
        ),
    ])
