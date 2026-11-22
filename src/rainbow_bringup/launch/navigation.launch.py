"""Nav2, using config/nav2.yaml.

All Nav2 servers run composed in one process (nav2_container): run as 11
separate processes they each decoded every /tf update and cost ~66% of a core
on the Pi while idle. component_container_isolated gives each server its own
executor thread, so a slow planner cannot block the controller.

Nav2's velocity output (collision monitor -> cmd_vel_stamped) goes through
rainbow_drivers' cmd_vel_watchdog, started by base.launch.py. One exception:
free_spin_server (the recovery spins, see nav2.yaml) publishes straight to
cmd_vel_stamped, past the smoother and the collision monitor, with its own
lifecycle manager. plan_viz
republishes /plan as /plan_viz for Foxglove.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution(
        [FindPackageShare('rainbow_bringup'), 'config', 'nav2.yaml'])

    return LaunchDescription([
        Node(
            package='rclcpp_components',
            executable='component_container_isolated',
            name='nav2_container',
            parameters=[params_file, {'autostart': True}],
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py'])),
            launch_arguments={
                'params_file': params_file,
                'use_composition': 'True',
                'container_name': 'nav2_container',
            }.items(),
        ),
        # The recovery spins: no collision check, past the collision monitor
        LoadComposableNodes(
            target_container='nav2_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='nav2_behaviors',
                    plugin='behavior_server::BehaviorServer',
                    name='free_spin_server',
                    parameters=[params_file],
                    remappings=[('cmd_vel', 'cmd_vel_stamped')],
                ),
                ComposableNode(
                    package='nav2_lifecycle_manager',
                    plugin='nav2_lifecycle_manager::LifecycleManager',
                    name='lifecycle_manager_free_spin',
                    parameters=[{'autostart': True, 'node_names': ['free_spin_server']}],
                ),
            ],
        ),
        # /plan with a frame on every pose, for Foxglove (Theta* leaves them empty)
        Node(package='rainbow_drivers', executable='plan_viz'),
    ])
