"""Nav2, using config/nav2.yaml.

Nav2's velocity output (collision monitor -> cmd_vel_stamped) goes through
rainbow_drivers' cmd_vel_watchdog, started by base.launch.py.
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
                [FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py'])),
            launch_arguments={
                'params_file': PathJoinSubstitution(
                    [FindPackageShare('rainbow_bringup'), 'config', 'nav2.yaml']),
            }.items(),
        ),
    ])
