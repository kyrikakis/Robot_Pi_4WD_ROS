"""Nav2, using config/nav2.yaml.

All Nav2 servers run composed in one process (nav2_container): run as 11
separate processes they each decoded every /tf update and cost ~66% of a core
on the Pi while idle. component_container_isolated gives each server its own
executor thread, so a slow planner cannot block the controller.

Nav2's velocity output (collision monitor -> cmd_vel_stamped) goes through
rainbow_drivers' cmd_vel_watchdog, started by base.launch.py.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution(
        [FindPackageShare('rainbow_bringup'), 'config', 'nav2.yaml'])

    return LaunchDescription([
        Node(
            package='rclcpp_components',
            executable='component_container_isolated',
            name='nav2_container',
            parameters=[params_file, {'autostart': True}],
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py'])),
            launch_arguments={
                'params_file': params_file,
                'use_composition': 'True',
                'container_name': 'nav2_container',
            }.items(),
        ),
    ])
