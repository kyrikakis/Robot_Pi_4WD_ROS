"""Localization on a saved map: map_server + AMCL, using config/localization.yaml.

The map is read-only here: losing localization only makes AMCL's pose
wrong, it cannot change the map (slam_toolbox's localization mode adds its
scans to the map and damaged it). For mapping a new place, stop this program
and start rainbow_slam instead (never both: both publish map -> odom).

At start, pose_keeper (rainbow_drivers) gives AMCL the last pose it saved
(power-on, a restart of this program), unless the robot was picked up since.
Without one, global localization spreads AMCL's guesses over the whole map.
They settle when the robot drives or turns a little; until then the pose is
meaningless, so do not send Nav2 goals right after such a start. In rooms that
look alike it can settle in the wrong one; a robot carried somewhere while it
was off starts at its old pose: then give a pose estimate (/initialpose).

AMCL publishes map -> odom only while it gets scans, and the lidar idles at
rest (lidar_power): map_tf_keeper (rainbow_drivers) holds AMCL's last
transform meanwhile, so the map frame (Nav2's bringup, goals sent at rest)
stays available.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml


def map_name():
    path = os.path.join(get_package_share_directory('rainbow_bringup'),
                        'config', 'localization.yaml')
    with open(path) as f:
        config = yaml.safe_load(f)
    map_yaml = config['map_server']['ros__parameters']['yaml_filename']
    return os.path.splitext(os.path.basename(map_yaml))[0]


def generate_launch_description():
    params_file = PathJoinSubstitution(
        [FindPackageShare('rainbow_bringup'), 'config', 'localization.yaml'])
    return LaunchDescription([
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             parameters=[params_file], output='screen'),
        Node(package='nav2_amcl', executable='amcl', name='amcl',
             parameters=[params_file], output='screen'),
        # Holds AMCL's last map -> odom while the lidar idles (AMCL's tolerance)
        Node(package='rainbow_drivers', executable='map_tf_keeper', name='map_tf_keeper',
             parameters=[{'transform_tolerance_s': 0.5}], output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization',
             parameters=[params_file], output='screen'),
        # The last saved pose once AMCL is active and has the map, else global
        # localization
        Node(package='rainbow_drivers', executable='pose_keeper', name='pose_keeper',
             parameters=[{'map_name': map_name(), 'data_dir': '/data/rainbow'}],
             output='screen'),
    ])
