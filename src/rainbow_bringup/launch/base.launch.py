"""Rainbow's base: everything needed for the robot to move and know where it is.

The micro-ROS agent (the bridge to the ESP board) is in microros.launch.py,
run as its own supervisor program so restarting the base does not disconnect
the ESP.

  - robot description and TF tree
  - BNO055 IMU
  - EKF: fuses wheel odometry and the BNO055 into /odom
  - velocity watchdog, eyes, IP publisher

The ToF depth camera is in tof.launch.py (supervisor program rainbow_tof), so
changing it does not restart the EKF, which would jump the odometry under SLAM.
The head is in head.launch.py (rainbow_head) for the same reason.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def config(name):
    return PathJoinSubstitution([FindPackageShare('rainbow_bringup'), 'config', name])


def generate_launch_description():
    return LaunchDescription([
        # The head node (head.launch.py) publishes the head joints, so no
        # joint_state_publisher
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('rainbow_description'), 'launch', 'description.launch.py'])),
            launch_arguments={'joint_state_publisher': 'false'}.items(),
        ),

        Node(
            package='bno055',
            executable='bno055',
            parameters=[config('bno055.yaml')],
            respawn=True,
            respawn_delay=2.0,
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[config('ekf.yaml')],
            remappings=[('odometry/filtered', 'odom')],
        ),

        # 20 Hz so commands reach the ESP as often as Nav2's controller sends them.
        # angular_gain 1.1: measured (65 mm wheels, motor PID 3.0/0.6, 2026-11-27),
        # spins in place turn at ~90% of the commanded rate
        Node(package='rainbow_drivers', executable='cmd_vel_watchdog',
             # min_inner_ratio 0.15 (default 0.3): turn in place only when the inner
             # wheels nearly stop; at 0.3 slow curves stopped the robot too often
             parameters=[{'rate': 20.0, 'angular_gain': 1.1, 'min_inner_ratio': 0.15}]),
        Node(package='rainbow_drivers', executable='eyes', respawn=True, respawn_delay=2.0),
        Node(package='rainbow_drivers', executable='ip_publisher'),
    ])
