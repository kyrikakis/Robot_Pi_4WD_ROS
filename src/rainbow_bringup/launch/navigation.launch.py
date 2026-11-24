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

The servers are loaded here, not through nav2_bringup's navigation_launch.py:
that one always adds the docking server (opennav_docking), which has no
setting to turn it off and publishes straight to cmd_vel, past the watchdog.
The robot has no dock. The list below is navigation_launch.py's (Jazzy 1.3)
without it.
"""

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


# (package, plugin, name, remaps cmd_vel to cmd_vel_nav), in navigation_launch.py's
# lifecycle order
SERVERS = [
    ('nav2_controller', 'nav2_controller::ControllerServer', 'controller_server', True),
    ('nav2_smoother', 'nav2_smoother::SmootherServer', 'smoother_server', False),
    ('nav2_planner', 'nav2_planner::PlannerServer', 'planner_server', False),
    ('nav2_route', 'nav2_route::RouteServer', 'route_server', False),
    ('nav2_behaviors', 'behavior_server::BehaviorServer', 'behavior_server', True),
    ('nav2_velocity_smoother', 'nav2_velocity_smoother::VelocitySmoother',
     'velocity_smoother', True),
    ('nav2_collision_monitor', 'nav2_collision_monitor::CollisionMonitor',
     'collision_monitor', False),
    ('nav2_bt_navigator', 'nav2_bt_navigator::BtNavigator', 'bt_navigator', False),
    ('nav2_waypoint_follower', 'nav2_waypoint_follower::WaypointFollower',
     'waypoint_follower', False),
]


def server(package, plugin, name, params_file, remap_cmd_vel):
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    if remap_cmd_vel:
        remappings.append(('cmd_vel', 'cmd_vel_nav'))
    return ComposableNode(package=package, plugin=plugin, name=name,
                          parameters=[params_file], remappings=remappings)


def generate_launch_description():
    params_file = PathJoinSubstitution(
        [FindPackageShare('rainbow_bringup'), 'config', 'nav2.yaml'])

    return LaunchDescription([
        # Unbuffered log lines, as navigation_launch.py sets them
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        Node(
            package='rclcpp_components',
            executable='component_container_isolated',
            name='nav2_container',
            parameters=[params_file, {'autostart': True}],
            output='screen',
        ),
        LoadComposableNodes(
            target_container='nav2_container',
            composable_node_descriptions=[
                server(package, plugin, name, params_file, remap)
                for package, plugin, name, remap in SERVERS
            ] + [
                ComposableNode(
                    package='nav2_lifecycle_manager',
                    plugin='nav2_lifecycle_manager::LifecycleManager',
                    name='lifecycle_manager_navigation',
                    parameters=[{'autostart': True,
                                 'node_names': [name for _, _, name, _ in SERVERS]}],
                ),
            ],
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
