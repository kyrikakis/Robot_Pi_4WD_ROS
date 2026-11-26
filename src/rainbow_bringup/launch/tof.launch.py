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
            # Near 1.2 cm up to 0.8 m: a desk foot on the hard floor next to the
            # carpet reads only ~1.4-2.4 cm above the carpet the robot stands on
            # (measured 2026-11-25), and was missed at 2 cm / 0.7 m. 1.5 cm made
            # floor ghosts on 2026-11-20, before the per-frame floor plane; with
            # it the carpet reads within +-0.5 cm
            parameters=[{'frame_rate': 15,
                         'horizontal_fov_deg': 48.5,
                         'vertical_fov_deg': 39.3,
                         'near_distance': 0.8,
                         'obstacle_min_height_near': 0.012,
                         'obstacle_min_height_far': 0.025}],
            respawn=True,
            respawn_delay=5.0,
        ),
    ])
