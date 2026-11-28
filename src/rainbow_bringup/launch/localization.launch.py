"""Localization on a saved map: map_server + AMCL, using config/localization.yaml.

The map is read-only here: losing localization only makes AMCL's pose
wrong, it cannot change the map (slam_toolbox's localization mode adds its
scans to the map and damaged it). For mapping a new place, stop this program
and start rainbow_slam instead (never both: both publish map -> odom).

At start there is no known pose: once AMCL is up with the map, global
localization spreads its guesses over the whole map. They settle when the
robot drives or turns a little; until then the pose is meaningless, so do not
send Nav2 goals right after a start. In rooms that look alike it can settle
in the wrong one: then give a pose estimate (/initialpose).

AMCL publishes map -> odom only while it gets scans, and the lidar idles at
rest (lidar_power): map_tf_keeper (rainbow_drivers) holds AMCL's last
transform meanwhile, so the map frame (Nav2's bringup, goals sent at rest)
stays available.
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


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
        # Global localization once AMCL is active and has the map ('ros2
        # service call' waits for the service; the sleep lets the map arrive)
        ExecuteProcess(
            cmd=['bash', '-c',
                 'until ros2 lifecycle get /amcl 2>/dev/null | grep -q active; do sleep 1; done;'
                 ' sleep 3; ros2 service call /reinitialize_global_localization'
                 ' std_srvs/srv/Empty'],
            output='screen'),
    ])
