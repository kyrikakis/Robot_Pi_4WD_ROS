"""Rainbow's voice: text to speech, and the ears (wake word, utterances).

Arguments:
  listen  true  start the voice node (ReSpeaker Lite; models in /opt/rainbow/voice)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('listen', default_value='true'),
        Node(package='rainbow_voice', executable='speech'),
        Node(
            package='rainbow_voice',
            executable='voice',
            condition=IfCondition(LaunchConfiguration('listen')),
            output='screen',
        ),
    ])
