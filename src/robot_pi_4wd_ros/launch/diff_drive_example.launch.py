# Copyright 2020 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("pca9685_ros2_control_example"), "description", "description.urdf.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("pca9685_ros2_control_example"),
            "config",
            "diff_drive_controller.yaml",
        ]
    )

    imu_params = PathJoinSubstitution(
        [
            FindPackageShare("pca9685_ros2_control_example"),
            "config",
            "bno055_params.yaml",
        ]
    )

    localization_params = PathJoinSubstitution(
        [
            FindPackageShare("pca9685_ros2_control_example"),
            "config",
            "localization_params.yaml",
        ]
    )

    channel_type =  LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200')
    frame_id = LaunchConfiguration('frame_id', default='laser')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Sensitivity')
    
    return LaunchDescription([

        DeclareLaunchArgument(
            'channel_type',
            default_value=channel_type,
            description='Specifying channel type of lidar'),
        
        DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port,
            description='Specifying usb port to connected lidar'),

        DeclareLaunchArgument(
            'serial_baudrate',
            default_value=serial_baudrate,
            description='Specifying usb port baudrate to connected lidar'),
        
        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='Specifying frame_id of lidar'),

        DeclareLaunchArgument(
            'inverted',
            default_value=inverted,
            description='Specifying whether or not to invert scan data'),

        DeclareLaunchArgument(
            'angle_compensate',
            default_value=angle_compensate,
            description='Specifying whether or not to enable angle_compensate of scan data'),
        DeclareLaunchArgument(
            'scan_mode',
            default_value=scan_mode,
            description='Specifying scan mode of lidar'),

        # Node(
        #     package='sllidar_ros2',
        #     executable='sllidar_node',
        #     name='sllidar_node',
        #     parameters=[{'channel_type':channel_type,
        #                 'serial_port': serial_port, 
        #                 'serial_baudrate': serial_baudrate, 
        #                 'frame_id': frame_id,
        #                 'inverted': inverted, 
        #                 'angle_compensate': angle_compensate}],
        #     output='both'
        # ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='both',
            parameters=[localization_params],
            remappings=[
                ('/cmd_vel', '/diff_drive_controller/cmd_vel_unstamped'),
            ]
        ),

        Node(
            package="bno055",
            executable="bno055",
            parameters=[imu_params],
            output="both",
        ),

        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[robot_description, robot_controllers],
            # prefix=['gdbserver localhost:3000'],
            output="both",
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen" ,
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"]
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen" ,
            arguments=["0", "0", "0", "0", "0", "0", "odom", "laser"]
        ),

        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen" ,
            arguments=["0", "0", "0", "0", "0", "0", "base_link", "imu_link"]
        ),
    ])
