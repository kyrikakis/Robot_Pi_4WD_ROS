"""Rainbow's base: everything needed for the robot to move and know where it is.

  - micro-ROS agent: the bridge to the ESP board (motors, servos, lidar, IMU, odometry)
  - robot description and TF tree
  - BNO055 IMU
  - EKF: fuses wheel odometry and the BNO055 into /odom
  - velocity watchdog, head, eyes, IP publisher
  - ToF depth camera

Arguments:
  microros   true   start the micro-ROS agent
  tof        true   start the ToF depth camera
  serial_port       /dev/ttyUSB_ROS
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
        DeclareLaunchArgument('microros', default_value='true'),
        DeclareLaunchArgument('tof', default_value='true'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB_ROS'),

        # The ESP board can be switched off; the agent exits when the serial
        # port is missing, so keep retrying until it appears.
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            arguments=['serial', '--dev', LaunchConfiguration('serial_port'),
                       '-b', '921600', '-v4'],
            respawn=True,
            respawn_delay=5.0,
            condition=IfCondition(LaunchConfiguration('microros')),
        ),

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
            respawn=True,
            respawn_delay=5.0,
            condition=IfCondition(LaunchConfiguration('tof')),
        ),
    ])
