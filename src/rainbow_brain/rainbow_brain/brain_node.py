"""Rainbow's local brain: skills, places, and (later) the link to the Mac agent.

Design: docs/brain.md. This is build step 1: LOCAL mode only, no link or voice
yet; the skills are called as ROS services and actions.

Every move goes through Nav2 (NavigateToPose, Spin, DriveOnHeading), so it
passes the velocity smoother and the collision monitor; the brain never
publishes cmd_vel. One move runs at a time: a new goal replaces the running one.

Actions (node name brain):
  /brain/go_to_place  rainbow_interfaces/action/GoToPlace  "home" = the place named home
  /brain/turn         rainbow_interfaces/action/Turn       radians, positive = left
  /brain/drive        rainbow_interfaces/action/Drive      metres, forward only
Services:
  /brain/stop         std_srvs/Trigger   cancel every Nav2 goal (not teleop)
  /brain/where_am_i   rainbow_interfaces/srv/WhereAmI
  /brain/get_state    rainbow_interfaces/srv/GetState
  /places/save, /places/edit, /places/delete, /places/list
Topics:
  /places/pick        geometry_msgs/PoseStamped    Foxglove's pose tool (Places tab)
  /places/markers     visualization_msgs/MarkerArray  places on the map (latched)
Reads: TF map -> base_footprint, /amcl_pose, /battery

Places are stored in <data_dir>/places/<map_name>.yaml (rainbow_brain.places).
"""

import math
import threading

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import DriveOnHeading, NavigateToPose, Spin
from rainbow_brain.places import PlaceStore
from rainbow_interfaces.action import Drive, GoToPlace, Turn
from rainbow_interfaces.msg import Place as PlaceMsg
from rainbow_interfaces.srv import (DeletePlace, EditPlace, GetState, ListPlaces,
                                    SavePlace, Say, WhereAmI)
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import UInt16
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)

# Nav2 actions cancelled by /brain/stop
NAV2_ACTIONS = ['navigate_to_pose', 'navigate_through_poses', 'follow_path', 'spin',
                'drive_on_heading', 'backup', 'assisted_teleop', 'wait']


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def to_msg(place):
    return PlaceMsg(name=place.name, x=place.x, y=place.y, yaw=place.yaw, room=place.room,
                    description=place.description, source=place.source,
                    confirmed=place.confirmed)


class BrainNode(Node):

    def __init__(self):
        super().__init__('brain')
        self.data_dir = self.declare_parameter('data_dir', '/data/rainbow').value
        self.map_name = self.declare_parameter('map_name', '').value
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').value
        # A place within this distance counts as "near" (m)
        self.near_distance = self.declare_parameter('near_distance', 1.5).value
        # Longest straight drive allowed (m), and its speed (m/s)
        self.max_drive = self.declare_parameter('max_drive', 1.0).value
        self.drive_speed = self.declare_parameter('drive_speed', 0.15).value
        # A Foxglove pick older than this is not used by /places/save (s)
        self.pick_max_age = self.declare_parameter('pick_max_age', 300.0).value
        if not self.map_name:
            raise RuntimeError('map_name is not set (brain.launch.py takes it from the '
                               'localization config)')

        # local: no agent. remote comes with the link (build step 2).
        self.mode = 'local'
        self.places = PlaceStore(self.data_dir, self.map_name, self.get_logger().warning)
        self.get_logger().info(
            f'{len(self.places.places)} places for map {self.map_name} in {self.places.path}')

        self.group = ReentrantCallbackGroup()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.amcl = None
        self.battery = None
        self.pick = None
        self.pick_time = None
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, LATCHED)
        self.create_subscription(UInt16, '/battery', self.on_battery, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/places/pick', self.on_pick, 10)
        self.markers_pub = self.create_publisher(MarkerArray, '/places/markers', LATCHED)

        # The running move: {'label', 'handle' (ours), 'nav' (Nav2's), 'replaced'}
        self.lock = threading.Lock()
        self.current = None

        g = {'callback_group': self.group}
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose', **g)
        self.spin_client = ActionClient(self, Spin, 'spin', **g)
        self.drive_client = ActionClient(self, DriveOnHeading, 'drive_on_heading', **g)
        self.cancel_clients = [self.create_client(CancelGoal, f'{a}/_action/cancel_goal', **g)
                               for a in NAV2_ACTIONS]
        self.say_client = self.create_client(Say, '/speech/say', **g)

        cancel = {'cancel_callback': self.on_cancel_request, **g}
        self.action_servers = [
            ActionServer(self, GoToPlace, '~/go_to_place', self.execute_go_to_place, **cancel),
            ActionServer(self, Turn, '~/turn', self.execute_turn, **cancel),
            ActionServer(self, Drive, '~/drive', self.execute_drive, **cancel),
        ]

        self.create_service(Trigger, '~/stop', self.on_stop, **g)
        self.create_service(WhereAmI, '~/where_am_i', self.on_where_am_i, **g)
        self.create_service(GetState, '~/get_state', self.on_get_state, **g)
        self.create_service(SavePlace, '/places/save', self.on_save_place, **g)
        self.create_service(EditPlace, '/places/edit', self.on_edit_place, **g)
        self.create_service(DeletePlace, '/places/delete', self.on_delete_place, **g)
        self.create_service(ListPlaces, '/places/list', self.on_list_places, **g)

        self.publish_markers()

    # Inputs

    def on_amcl(self, msg):
        self.amcl = msg

    def on_battery(self, msg):
        self.battery = msg.data / 10.0

    def on_pick(self, msg):
        if msg.header.frame_id != self.map_frame:
            self.get_logger().warning(
                f'pick ignored: it is in frame "{msg.header.frame_id}"; set the Places '
                f"panel's display frame to {self.map_frame}")
            return
        self.pick = msg
        self.pick_time = self.get_clock().now()
        p = msg.pose.position
        self.get_logger().info(f'pick at ({p.x:.2f}, {p.y:.2f}): name it with /places/save')

    # Where the robot is

    def pose(self):
        """(x, y, yaw) in the map frame, or None."""
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, Time())
        except TransformException:
            return None
        return (t.transform.translation.x, t.transform.translation.y,
                yaw_of(t.transform.rotation))

    def certainty(self):
        """(word, position std m, yaw std rad) from AMCL's covariance."""
        if self.amcl is None:
            return 'unknown', -1.0, -1.0
        c = self.amcl.pose.covariance
        pos_std = math.sqrt(max(c[0], c[7], 0.0))
        yaw_std = math.sqrt(max(c[35], 0.0))
        if pos_std < 0.25 and yaw_std < 0.2:
            return 'sure', pos_std, yaw_std
        if pos_std < 1.0:
            return 'roughly', pos_std, yaw_std
        return 'lost', pos_std, yaw_std

    # Moves

    def on_cancel_request(self, goal_handle):
        with self.lock:
            current = self.current
        if current is not None and current['handle'] is goal_handle and current['nav']:
            current['nav'].cancel_goal_async()
        return CancelResponse.ACCEPT

    async def run_nav2(self, label, client, goal, goal_handle, feedback=None):
        """Run one Nav2 action for one of our goals; returns (success, message)."""
        if not client.wait_for_server(timeout_sec=1.0):
            return False, 'navigation is not running'
        entry = {'label': label, 'handle': goal_handle, 'nav': None, 'replaced': False}
        with self.lock:
            previous, self.current = self.current, entry
        if previous is not None:
            previous['replaced'] = True
            if previous['nav']:
                previous['nav'].cancel_goal_async()
        try:
            nav = await client.send_goal_async(goal, feedback_callback=feedback)
            if not nav.accepted:
                return False, 'navigation refused the goal'
            entry['nav'] = nav
            if entry['replaced'] or goal_handle.is_cancel_requested:
                nav.cancel_goal_async()
            response = await nav.get_result_async()
        finally:
            with self.lock:
                if self.current is entry:
                    self.current = None
        if response.status == GoalStatus.STATUS_SUCCEEDED:
            return True, 'done'
        if entry['replaced']:
            return False, 'replaced by a new goal'
        if response.status == GoalStatus.STATUS_CANCELED:
            return False, 'cancelled'
        result = response.result
        return False, result.error_msg or f'navigation error {result.error_code}'

    @staticmethod
    def finish(goal_handle, result, success, message):
        result.success = success
        result.message = message
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
        elif success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return result

    async def execute_go_to_place(self, goal_handle):
        result = GoToPlace.Result()
        try:
            place = self.places.get(goal_handle.request.place)
        except ValueError as e:
            return self.finish(goal_handle, result, False, str(e))
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = place.x
        goal.pose.pose.position.y = place.y
        goal.pose.pose.orientation.z = math.sin(place.yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(place.yaw / 2.0)

        def feedback(msg):
            t = msg.feedback.estimated_time_remaining
            goal_handle.publish_feedback(GoToPlace.Feedback(
                distance_remaining=msg.feedback.distance_remaining,
                seconds_remaining=t.sec + t.nanosec * 1e-9))

        self.get_logger().info(f'going to {place.name}')
        ok, message = await self.run_nav2(
            f'going to {place.name}', self.nav_client, goal, goal_handle, feedback)
        if ok:
            message = f'arrived at {place.name}'
        else:
            message = f'could not get to {place.name}: {message}'
            if self.mode == 'local' and not message.endswith(('cancelled', 'new goal')):
                self.say(f"I couldn't get to the {place.name}.")
        self.get_logger().info(message)
        return self.finish(goal_handle, result, ok, message)

    async def execute_turn(self, goal_handle):
        result = Turn.Result()
        angle = goal_handle.request.angle
        if not 0.0 < abs(angle) <= 2.0 * math.pi:
            return self.finish(goal_handle, result, False,
                               'the angle must be non-zero and at most one full turn')
        goal = Spin.Goal(target_yaw=float(angle))
        goal.time_allowance.sec = int(abs(angle) / 0.3) + 5

        def feedback(msg):
            goal_handle.publish_feedback(
                Turn.Feedback(turned=msg.feedback.angular_distance_traveled))

        ok, message = await self.run_nav2(
            'turning', self.spin_client, goal, goal_handle, feedback)
        return self.finish(goal_handle, result, ok, message)

    async def execute_drive(self, goal_handle):
        result = Drive.Result()
        distance = goal_handle.request.distance
        if not 0.0 < distance <= self.max_drive:
            return self.finish(goal_handle, result, False,
                               f'I only drive forwards, at most {self.max_drive:.1f} m')
        goal = DriveOnHeading.Goal(speed=float(self.drive_speed))
        goal.target.x = float(distance)
        goal.time_allowance.sec = int(2.0 * distance / self.drive_speed) + 5

        def feedback(msg):
            goal_handle.publish_feedback(Drive.Feedback(driven=msg.feedback.distance_traveled))

        ok, message = await self.run_nav2(
            'driving', self.drive_client, goal, goal_handle, feedback)
        return self.finish(goal_handle, result, ok, message)

    def on_stop(self, request, response):
        with self.lock:
            current = self.current
        if current is not None and current['nav']:
            current['nav'].cancel_goal_async()
        # Zero goal id and stamp: cancel every goal of that action
        for client in self.cancel_clients:
            if client.service_is_ready():
                client.call_async(CancelGoal.Request())
        self.get_logger().info('stop: cancelled all Nav2 goals')
        response.success = True
        response.message = 'stopping'
        return response

    def say(self, text):
        if self.say_client.service_is_ready():
            self.say_client.call_async(Say.Request(text=text))

    # Questions

    def on_where_am_i(self, request, response):
        pose = self.pose()
        word, _, _ = self.certainty()
        response.certainty = word
        if pose is None:
            response.localized = False
            response.description = "I don't know where I am: there is no map position yet."
            return response
        x, y, yaw = pose
        sentence, near, ahead = self.places.describe(x, y, yaw, self.near_distance, word)
        response.localized = True
        response.description = sentence
        response.x, response.y, response.yaw = x, y, yaw
        if near is not None:
            response.nearest_place = near.name
            response.nearest_distance = math.hypot(near.x - x, near.y - y)
            response.room = near.room
        if ahead is not None:
            response.facing_place = ahead.name
        return response

    def on_get_state(self, request, response):
        pose = self.pose()
        word, pos_std, yaw_std = self.certainty()
        with self.lock:
            current = self.current
        response.mode = self.mode
        response.map = self.map_name
        response.localized = pose is not None
        response.certainty = word
        if pose is not None:
            response.x, response.y, response.yaw = pose
        response.position_std = pos_std
        response.yaw_std = yaw_std
        response.battery_volts = float(self.battery or 0.0)
        response.activity = current['label'] if current else 'idle'
        return response

    # Places

    def on_save_place(self, request, response):
        if request.from_pick:
            age = None if self.pick_time is None else \
                (self.get_clock().now() - self.pick_time).nanoseconds * 1e-9
            if age is None or age > self.pick_max_age:
                response.message = 'no recent pick: click a pose in the Places panel first'
                return response
            p = self.pick.pose
            x, y, yaw = p.position.x, p.position.y, yaw_of(p.orientation)
            source = 'foxglove'
        else:
            pose = self.pose()
            if pose is None:
                response.message = "I don't know where I am, so I can't save this place"
                return response
            if self.certainty()[0] == 'lost':
                response.message = "I'm not sure where I am yet, so I can't save this place"
                return response
            x, y, yaw = pose
            source = 'robot' if request.guess else 'owner'
        try:
            place = self.places.save(request.name, x, y, yaw, request.room,
                                     request.description, source)
        except (ValueError, OSError) as e:
            response.message = str(e)
            return response
        self.publish_markers()
        response.success = True
        response.message = f'saved {place.name}'
        response.place = to_msg(place)
        self.get_logger().info(f'{response.message} at ({x:.2f}, {y:.2f}) [{source}]')
        return response

    def on_edit_place(self, request, response):
        try:
            place = self.places.edit(request.name, request.new_name, request.room,
                                     request.description, request.confirm)
        except (ValueError, OSError) as e:
            response.message = str(e)
            return response
        self.publish_markers()
        response.success = True
        response.message = f'updated {place.name}'
        response.place = to_msg(place)
        return response

    def on_delete_place(self, request, response):
        try:
            place = self.places.delete(request.name)
        except (ValueError, OSError) as e:
            response.message = str(e)
            return response
        self.publish_markers()
        response.success = True
        response.message = f'deleted {place.name}'
        return response

    def on_list_places(self, request, response):
        response.map = self.map_name
        response.places = [to_msg(p) for p in self.places.sorted_places()]
        return response

    def publish_markers(self):
        """An arrow and a label per place: green confirmed, orange the robot's guesses."""
        msg = MarkerArray()
        msg.markers.append(Marker(action=Marker.DELETEALL))
        for i, place in enumerate(self.places.sorted_places()):
            color = (0.2, 0.8, 0.3) if place.confirmed else (1.0, 0.6, 0.1)
            for kind in (Marker.ARROW, Marker.TEXT_VIEW_FACING):
                m = Marker(ns='places', id=2 * i + (kind == Marker.TEXT_VIEW_FACING),
                           type=kind, action=Marker.ADD)
                m.header.frame_id = self.map_frame
                m.pose.position.x = place.x
                m.pose.position.y = place.y
                m.pose.orientation.z = math.sin(place.yaw / 2.0)
                m.pose.orientation.w = math.cos(place.yaw / 2.0)
                m.color.r, m.color.g, m.color.b = color
                m.color.a = 1.0
                if kind == Marker.ARROW:
                    m.scale.x, m.scale.y, m.scale.z = 0.3, 0.05, 0.05
                else:
                    m.pose.position.z = 0.25
                    m.scale.z = 0.15
                    m.text = place.name if place.confirmed else f'{place.name}?'
                msg.markers.append(m)
        self.markers_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BrainNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
