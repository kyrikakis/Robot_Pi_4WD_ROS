"""Rainbow's local brain (rainbow_brain, see docs/brain.md) and the head tracker.

The map name comes from config/localization.yaml (map_server's yaml_filename),
so the places always belong to the map AMCL is using. Places are stored on the
rainbow_data volume (/data/rainbow, docker-compose.yaml).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import yaml


def map_name():
    path = os.path.join(get_package_share_directory('rainbow_bringup'),
                        'config', 'localization.yaml')
    with open(path) as f:
        config = yaml.safe_load(f)
    map_yaml = config['map_server']['ros__parameters']['yaml_filename']
    return os.path.splitext(os.path.basename(map_yaml))[0]


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rainbow_brain',
            executable='brain',
            parameters=[{'map_name': map_name(), 'data_dir': '/data/rainbow'}],
            output='screen',
        ),
        # Head tracking (the brain's track skill starts and stops it)
        Node(package='rainbow_brain', executable='tracker', output='screen'),
    ])
