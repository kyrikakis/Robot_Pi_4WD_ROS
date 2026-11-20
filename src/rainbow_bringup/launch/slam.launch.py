"""SLAM with slam_toolbox (online async), using config/slam_toolbox.yaml.

For mapping a new place only (rainbow_slam, autostart off): stop
rainbow_localization first, both publish map -> odom.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py'])),
            launch_arguments={
                'slam_params_file': PathJoinSubstitution(
                    [FindPackageShare('rainbow_bringup'), 'config', 'slam_toolbox.yaml']),
            }.items(),
        ),
    ])
