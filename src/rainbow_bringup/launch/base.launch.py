"""Rainbow's base: everything needed for the robot to move and know where it is.

The micro-ROS agent (the bridge to the ESP board) is in microros.launch.py,
run as its own supervisor program so restarting the base does not disconnect
the ESP.

  - robot description and TF tree
  - BNO055 IMU
  - EKF: fuses wheel odometry and the BNO055 into /odom
  - velocity watchdog, head, eyes, IP publisher
  - ToF depth camera

Arguments:
  tof        true   start the ToF depth camera
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def config(name):
    return PathJoinSubstitution([FindPackageShare('rainbow_bringup'), 'config', name])


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('tof', default_value='true'),

        # The head node publishes the head joints, so no joint_state_publisher
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

        Node(package='rainbow_drivers', executable='cmd_vel_watchdog'),
        Node(package='rainbow_drivers', executable='head'),
        Node(package='rainbow_drivers', executable='eyes', respawn=True, respawn_delay=2.0),
        Node(package='rainbow_drivers', executable='ip_publisher'),
        Node(
            package='rainbow_drivers',
            executable='tof',
            # The sensor always runs at ~25-30 fps and the SDK's depth computation
            # costs the same either way; frame_rate sets how many clouds Nav2's
            # voxel layers have to process. 15 fps: obstacles within ~70 ms.
            # FOV measured with scripts/calibrate_tof.py (camera height + lidar
            # corridor width); the spec sheet's 64.3 x 50.4 is too wide
            parameters=[{'frame_rate': 15,
                         'horizontal_fov_deg': 48.5,
                         'vertical_fov_deg': 39.3}],
            respawn=True,
            respawn_delay=5.0,
            condition=IfCondition(LaunchConfiguration('tof')),
        ),
    ])
