"""Arducam ToF depth camera (rainbow_drivers tof node).

Its own supervisor program (rainbow_tof): restarting it does not touch the EKF
or SLAM, unlike restarting the base.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rainbow_drivers',
            executable='tof',
            # The sensor always runs at ~25-30 fps and the SDK's depth computation
            # costs the same either way; frame_rate sets how many clouds Nav2's
            # voxel layers have to process. 15 fps: obstacles within ~70 ms.
            # FOV measured with scripts/calibrate_tof.py (camera height + lidar
            # corridor width); the spec sheet's 64.3 x 50.4 is too wide
            # Far obstacle height lowered from 3.5 cm to pick up shorter bumps.
            # Near stays 2 cm: at 1.5 cm floor points (read ~1-2 cm high) were
            # marked as ghosts in the lowest voxel layer
            parameters=[{'frame_rate': 15,
                         'horizontal_fov_deg': 48.5,
                         'vertical_fov_deg': 39.3,
                         'obstacle_min_height_near': 0.02,
                         'obstacle_min_height_far': 0.025}],
            respawn=True,
            respawn_delay=5.0,
        ),
    ])
