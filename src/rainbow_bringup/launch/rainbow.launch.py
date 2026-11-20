"""Everything in one launch, for running by hand.

On the robot, supervisor runs the parts separately (supervisor/rainbow.conf)
so each can be stopped and started on its own.

Arguments (default true): localization, navigation, foxglove, voice, tof,
microros; slam (default false, for mapping: use it with localization:=false)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def part(name, condition=None, arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('rainbow_bringup'), 'launch', f'{name}.launch.py'])),
        condition=IfCondition(LaunchConfiguration(condition)) if condition else None,
        launch_arguments=(arguments or {}).items(),
    )


def generate_launch_description():
    args = ['localization', 'navigation', 'foxglove', 'voice', 'tof', 'microros']
    return LaunchDescription(
        [DeclareLaunchArgument(a, default_value='true') for a in args] + [
            DeclareLaunchArgument('slam', default_value='false'),
            part('microros', 'microros'),
            part('base'),
            part('tof', 'tof'),
            part('localization', 'localization'),
            part('slam', 'slam'),
            part('navigation', 'navigation'),
            part('foxglove', 'foxglove'),
            part('voice', 'voice'),
        ])
