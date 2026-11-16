"""Publish Rainbow's robot description and TF tree.

Arguments:
  camera_model          imx708 (default) or imx296
  joint_state_publisher true (default): publish the head joints at 0 until
                        a head node publishes real /joint_states
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    xacro_file = PathJoinSubstitution(
        [FindPackageShare('rainbow_description'), 'urdf', 'rainbow.urdf.xacro'])

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file,
                 ' camera_model:=', LaunchConfiguration('camera_model')]),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('camera_model', default_value='imx708'),
        DeclareLaunchArgument('joint_state_publisher', default_value='true'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            condition=IfCondition(LaunchConfiguration('joint_state_publisher')),
        ),
    ])
