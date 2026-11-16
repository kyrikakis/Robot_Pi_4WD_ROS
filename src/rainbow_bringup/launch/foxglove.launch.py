"""Foxglove bridge on all interfaces, port 8765."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('foxglove_bridge'), 'launch', 'foxglove_bridge_launch.xml'])),
            launch_arguments={'address': '0.0.0.0'}.items(),
        ),
    ])
