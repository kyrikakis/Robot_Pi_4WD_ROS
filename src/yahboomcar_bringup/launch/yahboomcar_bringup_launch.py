from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import os
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

print("---------------------robot_type = x3---------------------")
def generate_launch_description():
    imu_filter_config = os.path.join(              
        get_package_share_directory('yahboomcar_bringup'),
        'params',
        'imu_filter_param.yaml'
    ) 

    imu_filter_config = os.path.join(get_package_share_directory('imu_complementary_filter'), 'config', 'filter_config.yaml')

    imu_filter_node = Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='complementary_filter_gain_node',
        output='screen',
        remappings=[('/imu/data_raw','/imu')],
        parameters=[imu_filter_config],
    )
    
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('yahboomcar_description'), 'launch'),
         '/description_launch.py'])
    )

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(get_package_share_directory("yahboomcar_bringup"), 'params', 'ekf.yaml')],
        remappings=[('/odometry/filtered','/odom')]
    )
    
    base_link_to_imu_tf_node = Node(
     package='tf2_ros',
     executable='static_transform_publisher',
     name='base_link_to_base_imu',
     arguments=['-0.002999', '-0.0030001','0.031701','0','0','0','base_link','imu_frame']
    ) 

    laser_frame_to_base_link_node = Node(
     package='tf2_ros',
     executable='static_transform_publisher',
     name='base_link_to_base_laser',
     arguments=['-0.0046412', '0' , '0.094079','0','0','0','base_link','laser_frame']
    )
    
    return LaunchDescription([
        imu_filter_node,
        robot_localization_node,
        base_link_to_imu_tf_node,
        laser_frame_to_base_link_node,
        description_launch
    ])
