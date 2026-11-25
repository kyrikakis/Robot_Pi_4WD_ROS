"""Rainbow's local brain: skills, places, and the link to the Mac agent.

Design: docs/brain.md. Skills as ROS services and actions, the same skills
for the Mac agent over the link (rainbow_brain.link), and the voice router.
The mode is remote while an agent is connected, local otherwise.

Voice (from rainbow_voice's voice node): /voice/stop stops at once; an
utterance whose command phrase Vosk matched confidently is handled here
(rainbow_brain.commands) and the agent is told; any other utterance goes to
the agent, or, without one, the robot says it can only do simple things.

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
  /voice/utterance    rainbow_interfaces/Utterance in; /voice/stop std_msgs/Empty in
  /voice/grammar      std_msgs/String (latched) out: JSON list of command phrases
  /places/pick        geometry_msgs/PoseStamped    Foxglove's pose tool (Places tab)
  /places/markers     visualization_msgs/MarkerArray  places on the map (latched)
  /brain/status       rainbow_interfaces/BrainStatus  for Foxglove: the brain's state,
                      health (ESP messages, Nav2, speech) and the agent's status
                      (from the link), every status_period s and on every change
  /agent/log          rcl_interfaces/Log   the agent's log lines (from the link)
Head: look points the head (the head node's /head/move action, which returns once
the head has settled); capture_photo skips frames while /head/moving is true.
track / stop_tracking start and stop the tracker node (/tracker/start, /tracker/stop);
stop and look stop tracking too. A target lost from view becomes a track_lost event.
Camera and faces (the Hailo container, docs/brain.md): capture_photo takes one
frame from photo_topic on demand; recognised faces on faces_topic become
face_seen events; remember_face asks the Hailo node to save a face.
set_pose_estimate publishes /initialpose for AMCL (startup localization).
Reads: TF map -> base_footprint, /amcl_pose, /battery; /scan and /odom_raw only
for their arrival times (not deserialized)
Link: WebSocket on link_port (8766); token in <data_dir>/link_token.

Places are stored in <data_dir>/places/<map_name>.yaml (rainbow_brain.places).
"""

import asyncio
import base64
import json
import math
import os
import threading
import time

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from hailo_msgs.srv import SaveGalleryItem
from nav2_msgs.action import DriveOnHeading, NavigateToPose, Spin
from nav_msgs.msg import Odometry
from rainbow_brain.commands import parse, phrases
from rainbow_brain.link import LinkServer, load_token
from rainbow_brain.places import PlaceStore
from rainbow_interfaces.action import Drive, GoToPlace, MoveHead, Turn
from rainbow_interfaces.msg import (AgentStatus, BrainStatus, Place as PlaceMsg, TrackerStatus,
                                    Utterance)
from rainbow_interfaces.srv import (DeletePlace, EditPlace, GetState, ListPlaces,
                                    SavePlace, Say, Track, WhereAmI)
from rcl_interfaces.msg import Log
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rclpy.task import Future
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, LaserScan
from std_msgs.msg import Bool, Empty, String, UInt16
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import Marker, MarkerArray

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)

# Nav2 actions cancelled by stop
NAV2_ACTIONS = ['navigate_to_pose', 'navigate_through_poses', 'follow_path', 'spin',
                'drive_on_heading', 'backup', 'assisted_teleop', 'wait']


LOG_LEVELS = {'debug': Log.DEBUG, 'info': Log.INFO, 'warn': Log.WARN,
              'warning': Log.WARN, 'error': Log.ERROR}


def agent_status_msg(status, connected, stamp=None):
    """Build an AgentStatus from the agent's dict: known fields only, wrong types skipped."""
    msg = AgentStatus(connected=connected)
    if stamp is not None:
        msg.stamp = stamp
    for name, kind in AgentStatus.get_fields_and_field_types().items():
        if name in ('stamp', 'connected') or name not in status:
            continue
        try:
            if kind == 'string':
                value = str(status[name])
            elif kind == 'boolean':
                value = bool(status[name])
            elif kind.startswith('uint'):
                value = min(max(int(status[name]), 0), 2**32 - 1)
            elif kind in ('float', 'double'):
                value = float(status[name])
            else:
                continue
        except (TypeError, ValueError):
            continue
        setattr(msg, name, value)
    return msg


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def to_msg(place):
    return PlaceMsg(name=place.name, x=place.x, y=place.y, yaw=place.yaw, room=place.room,
                    description=place.description, source=place.source,
                    confirmed=place.confirmed)


def to_dict(place):
    return {'name': place.name, 'room': place.room, 'description': place.description,
            'confirmed': place.confirmed}


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
        # The link to the Mac agent; 0 turns it off
        self.link_port = self.declare_parameter('link_port', 8766).value
        # A local voice command needs at least this Vosk confidence
        self.command_confidence = self.declare_parameter('command_confidence', 0.8).value
        # The Hailo container's camera and face recognition
        self.photo_topic = self.declare_parameter(
            'photo_topic', '/hailo_face_recognition/image_bbx/compressed').value
        self.photo_width = self.declare_parameter('photo_width', 640).value
        faces_topic = self.declare_parameter(
            'faces_topic', '/hailo_face_recognition/detections').value
        # Seconds before the same face is announced to the agent again
        self.face_repeat = self.declare_parameter('face_repeat', 600.0).value
        self.faces_seen = {}
        # "Look around": localize_turns full turns in one go, then again if AMCL is not
        # sure yet, at most localize_rounds times
        self.localize_turns = self.declare_parameter('localize_turns', 4).value
        self.localize_rounds = self.declare_parameter('localize_rounds', 2).value
        self.localizing = False
        # /brain/status every this many seconds (and on every change)
        status_period = self.declare_parameter('status_period', 1.0).value
        if not self.map_name:
            raise RuntimeError('map_name is not set (brain.launch.py takes it from the '
                               'localization config)')

        # local: no agent connected; remote: the agent decides
        self.mode = 'local'
        self.places = PlaceStore(self.data_dir, self.map_name, self.get_logger().warning)
        self.places_lock = threading.Lock()
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
        # Health: when the ESP's messages last arrived (monotonic s); raw, never decoded
        self.started = time.monotonic()
        self.arrived = {'battery': None, 'scan': None, 'odom': None}
        self.create_subscription(LaserScan, '/scan', lambda _: self.mark('scan'),
                                 qos_profile_sensor_data, raw=True)
        self.create_subscription(Odometry, '/odom_raw', lambda _: self.mark('odom'),
                                 qos_profile_sensor_data, raw=True)
        self.last_event = ''
        self.last_command = ''
        # The agent's last status (from the link) and when it arrived
        self.agent_lock = threading.Lock()
        self.agent_status = {}
        self.agent_stamp = None
        self.status_pub = self.create_publisher(BrainStatus, '/brain/status', LATCHED)
        self.agent_log_pub = self.create_publisher(Log, '/agent/log', 50)
        self.create_timer(status_period, self.publish_status)
        self.create_subscription(PoseStamped, '/places/pick', self.on_pick, 10)
        self.markers_pub = self.create_publisher(MarkerArray, '/places/markers', LATCHED)
        self.grammar_pub = self.create_publisher(String, '/voice/grammar', LATCHED)
        self.create_subscription(Utterance, '/voice/utterance', self.on_utterance, 10,
                                 callback_group=self.group)
        self.create_subscription(Empty, '/voice/stop', self.on_voice_stop, 10,
                                 callback_group=self.group)
        self.create_subscription(Detection2DArray, faces_topic, self.on_faces,
                                 qos_profile_sensor_data, callback_group=self.group)
        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        # The head node's state: no photos while it moves or settles
        self.head_moving = False
        self.create_subscription(Bool, '/head/moving', self.on_head_moving, LATCHED)
        self.tracking_lost = False
        self.tracking = ''          # what the head follows ('' = nothing)
        self.create_subscription(TrackerStatus, '/tracker/status', self.on_tracker_status,
                                 LATCHED, callback_group=self.group)

        # The running move: {'label', 'handle' (ours, or None from the link),
        # 'nav' (Nav2's), 'replaced'}
        self.lock = threading.Lock()
        self.current = None

        g = {'callback_group': self.group}
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose', **g)
        self.spin_client = ActionClient(self, Spin, 'spin', **g)
        self.drive_client = ActionClient(self, DriveOnHeading, 'drive_on_heading', **g)
        self.head_client = ActionClient(self, MoveHead, '/head/move', **g)
        self.track_client = self.create_client(Track, '/tracker/start', **g)
        self.untrack_client = self.create_client(Trigger, '/tracker/stop', **g)
        self.cancel_clients = [self.create_client(CancelGoal, f'{a}/_action/cancel_goal', **g)
                               for a in NAV2_ACTIONS]
        self.say_client = self.create_client(Say, '/speech/say', **g)
        self.save_face_client = self.create_client(
            SaveGalleryItem, '/hailo_face_recognition/save_face', **g)

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
        self.publish_grammar()

        self.link = None
        if self.link_port:
            token = load_token(os.path.join(self.data_dir, 'link_token'))
            self.link = LinkServer(LinkHandler(self), token, port=self.link_port,
                                   info={'robot': 'rainbow', 'map': self.map_name},
                                   log=self.get_logger().info)
            self.link.start()
            self.get_logger().info(f'link listening on port {self.link_port}')

    def destroy_node(self):
        if self.link is not None:
            self.link.stop()
        super().destroy_node()

    def emit(self, event):
        """Send an event to the agent (kept for later if none is connected)."""
        self.last_event = ' '.join(str(v) for v in event.values() if isinstance(v, str))
        if self.link is not None:
            self.link.send_event(event)
        self.publish_status()

    def state_changed(self):
        if self.link is not None:
            self.link.send_state()
        self.publish_status()

    # Status for Foxglove

    def mark(self, source):
        self.arrived[source] = time.monotonic()

    def age(self, source):
        t = self.arrived[source]
        return -1.0 if t is None else time.monotonic() - t

    def publish_status(self):
        """Publish /brain/status (from any thread)."""
        s = self.state()
        msg = BrainStatus(
            mode=s['mode'], map=s['map'], activity=s['activity'],
            localized=s['localized'], certainty=s['certainty'],
            x=float(s.get('x', 0.0)), y=float(s.get('y', 0.0)), yaw=float(s.get('yaw', 0.0)),
            position_std=float(s['position_std']), yaw_std=float(s['yaw_std']),
            battery_volts=float(s['battery_volts']),
            battery_age=self.age('battery'), scan_age=self.age('scan'),
            odom_age=self.age('odom'),
            nav2_ready=self.nav_client.server_is_ready(),
            speech_ready=self.say_client.service_is_ready(),
            faces_ready=self.save_face_client.service_is_ready(),
            places=len(s['places']), last_event=self.last_event,
            last_command=self.last_command, uptime=time.monotonic() - self.started)
        msg.stamp = self.get_clock().now().to_msg()
        if self.localizing:
            msg.activity = 'looking around' + (
                '' if msg.activity == 'idle' else f' ({msg.activity})')
        with self.agent_lock:
            status, stamp = dict(self.agent_status), self.agent_stamp
        msg.agent = agent_status_msg(status, self.mode == 'remote', stamp)
        self.status_pub.publish(msg)

    def on_agent_status(self, status):
        with self.agent_lock:
            self.agent_status.update(status)
            self.agent_stamp = self.get_clock().now().to_msg()
        self.publish_status()

    def on_agent_log(self, level, text):
        msg = Log(level=LOG_LEVELS.get(level.lower(), Log.INFO), name='rainbow_agent',
                  msg=text)
        msg.stamp = self.get_clock().now().to_msg()
        self.agent_log_pub.publish(msg)

    # Inputs

    def on_amcl(self, msg):
        self.amcl = msg

    def on_battery(self, msg):
        self.battery = msg.data / 10.0
        self.mark('battery')

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

    def where_am_i(self):
        """Return where the robot is, for the agent and the WhereAmI service."""
        pose = self.pose()
        word = self.certainty()[0]
        if pose is None:
            return {'ok': True, 'localized': False, 'certainty': word,
                    'description': "I don't know where I am: there is no map position yet."}
        x, y, yaw = pose
        with self.places_lock:
            sentence, near, ahead = self.places.describe(x, y, yaw, self.near_distance, word)
        # Only 'sure' counts as localized: 'roughly' can be a metre off
        out = {'ok': True, 'localized': word == 'sure', 'certainty': word,
               'description': sentence,
               'x': round(x, 2), 'y': round(y, 2), 'yaw': round(yaw, 2)}
        if near is not None:
            out['nearest_place'] = near.name
            out['nearest_distance'] = round(math.hypot(near.x - x, near.y - y), 2)
            out['room'] = near.room
        if ahead is not None:
            out['facing_place'] = ahead.name
        return out

    def state(self):
        """Return the state, for the agent and the GetState service."""
        pose = self.pose()
        word, pos_std, yaw_std = self.certainty()
        with self.lock:
            current = self.current
        with self.places_lock:
            names = [p.name for p in self.places.sorted_places()]
        out = {'ok': True, 'mode': self.mode, 'map': self.map_name,
               'localized': pose is not None and word == 'sure', 'certainty': word,
               'position_std': round(pos_std, 2), 'yaw_std': round(yaw_std, 2),
               'battery_volts': round(float(self.battery or 0.0), 1),
               'activity': current['label'] if current else 'idle', 'places': names}
        if self.tracking:
            out['head_following'] = self.tracking
        if pose is not None:
            out['x'], out['y'], out['yaw'] = (round(v, 2) for v in pose)
        return out

    # Moves

    def on_cancel_request(self, goal_handle):
        with self.lock:
            current = self.current
        if current is not None and current['handle'] is goal_handle and current['nav']:
            current['nav'].cancel_goal_async()
        return CancelResponse.ACCEPT

    async def run_nav2(self, label, client, goal, goal_handle=None, feedback=None):
        """Run one Nav2 action; returns (success, message). goal_handle: our action's, or None."""
        if not client.wait_for_server(timeout_sec=1.0):
            return False, 'navigation is not running'
        entry = {'label': label, 'handle': goal_handle, 'nav': None, 'replaced': False}
        with self.lock:
            previous, self.current = self.current, entry
        if previous is not None:
            previous['replaced'] = True
            if previous['nav']:
                previous['nav'].cancel_goal_async()
        self.state_changed()
        try:
            nav = await client.send_goal_async(goal, feedback_callback=feedback)
            if not nav.accepted:
                return False, 'navigation refused the goal'
            entry['nav'] = nav
            if entry['replaced'] or (goal_handle is not None and goal_handle.is_cancel_requested):
                nav.cancel_goal_async()
            response = await nav.get_result_async()
        finally:
            with self.lock:
                if self.current is entry:
                    self.current = None
            self.state_changed()
        if response.status == GoalStatus.STATUS_SUCCEEDED:
            return True, 'done'
        if entry['replaced']:
            return False, 'replaced by a new goal'
        if response.status == GoalStatus.STATUS_CANCELED:
            return False, 'cancelled'
        result = response.result
        return False, result.error_msg or f'navigation error {result.error_code}'

    def find_place(self, name):
        with self.places_lock:
            return self.places.get(name)

    async def go_to(self, place, goal_handle=None, feedback=None):
        """Drive to a place; tell the agent (event) or, without one, say failures."""
        if self.pose() is None:
            message = f"could not get to {place.name}: I don't know where I am"
            self.get_logger().info(message)
            self.emit({'kind': 'failed', 'place': place.name, 'message': message})
            if self.mode == 'local':
                self.say("I don't know where I am yet, so I can't go there.")
            return False, message
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = place.x
        goal.pose.pose.position.y = place.y
        goal.pose.pose.orientation.z = math.sin(place.yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(place.yaw / 2.0)
        self.get_logger().info(f'going to {place.name}')
        ok, reason = await self.run_nav2(
            f'going to {place.name}', self.nav_client, goal, goal_handle, feedback)
        if ok:
            message = f'arrived at {place.name}'
            self.emit({'kind': 'arrived', 'place': place.name})
        else:
            message = f'could not get to {place.name}: {reason}'
            if reason == 'cancelled':
                self.emit({'kind': 'cancelled', 'place': place.name})
            elif reason != 'replaced by a new goal':
                self.emit({'kind': 'failed', 'place': place.name, 'message': message})
                if self.mode == 'local':
                    self.say(f"I couldn't get to the {place.name}.")
        self.get_logger().info(message)
        return ok, message

    async def turn(self, angle, goal_handle=None, feedback=None):
        """Turn in place by angle (rad, positive left); returns (success, message)."""
        if not 0.0 < abs(angle) <= 2.0 * math.pi:
            return False, 'the angle must be non-zero and at most one full turn'
        return await self.spin(angle, goal_handle, feedback)

    async def spin(self, angle, goal_handle=None, feedback=None):
        """Nav2's Spin by angle (rad, any number of turns); returns (success, message)."""
        goal = Spin.Goal(target_yaw=float(angle))
        # 0.2 rad/s: worn wheels turn the robot at a third of the wheel rate (AGENTS.md)
        goal.time_allowance.sec = int(abs(angle) / 0.2) + 5
        return await self.run_nav2('turning', self.spin_client, goal, goal_handle, feedback)

    async def drive(self, distance, goal_handle=None, feedback=None):
        """Drive straight ahead (m, forward only); returns (success, message)."""
        if not 0.0 < distance <= self.max_drive:
            return False, f'I only drive forwards, at most {self.max_drive:.1f} m'
        goal = DriveOnHeading.Goal(speed=float(self.drive_speed))
        goal.target.x = float(distance)
        goal.time_allowance.sec = int(2.0 * distance / self.drive_speed) + 5
        return await self.run_nav2('driving', self.drive_client, goal, goal_handle, feedback)

    def stop(self):
        self.stop_tracking()
        with self.lock:
            current = self.current
        if current is not None and current['nav']:
            current['nav'].cancel_goal_async()
        # Zero goal id and stamp: cancel every goal of that action
        for client in self.cancel_clients:
            if client.service_is_ready():
                client.call_async(CancelGoal.Request())
        self.get_logger().info('stop: cancelled all Nav2 goals')

    def say(self, text):
        if not text:
            return
        self.get_logger().info(f'say: {text}')
        if self.say_client.service_is_ready():
            self.say_client.call_async(Say.Request(text=text))

    # Places (each raises ValueError or OSError with a speakable reason)

    def save_place(self, name, description='', room='', from_pick=False, guess=False):
        if from_pick:
            age = None if self.pick_time is None else \
                (self.get_clock().now() - self.pick_time).nanoseconds * 1e-9
            if age is None or age > self.pick_max_age:
                raise ValueError('no recent pick: click a pose in the Places panel first')
            p = self.pick.pose
            x, y, yaw = p.position.x, p.position.y, yaw_of(p.orientation)
            source = 'foxglove'
            self.pick = self.pick_time = None  # one pick, one place
        else:
            pose = self.pose()
            if pose is None:
                raise ValueError("I don't know where I am, so I can't save this place")
            if self.certainty()[0] != 'sure':
                raise ValueError("I'm not sure where I am yet, so I can't save this place")
            x, y, yaw = pose
            source = 'robot' if guess else 'owner'
        with self.places_lock:
            place = self.places.save(name, x, y, yaw, room, description, source,
                                     confirmed=not guess)
        self.places_changed()
        self.get_logger().info(f'saved {place.name} at ({x:.2f}, {y:.2f}) [{source}'
                               f'{", guess" if guess else ""}]')
        return place

    def edit_place(self, name, new_name='', description='', room='', confirm=False):
        with self.places_lock:
            place = self.places.edit(name, new_name, room, description, confirm)
        self.places_changed()
        self.get_logger().info(f'edited {name} -> {place.name}')
        return place

    def delete_place(self, name):
        with self.places_lock:
            place = self.places.delete(name)
        self.places_changed()
        self.get_logger().info(f'deleted {place.name}')
        return place

    def places_changed(self):
        self.publish_markers()
        self.publish_grammar()
        self.state_changed()

    def place_names(self):
        with self.places_lock:
            return [p.name for p in self.places.sorted_places()]

    def publish_grammar(self):
        self.grammar_pub.publish(String(data=json.dumps(phrases(self.place_names()))))

    # Camera and faces

    def on_faces(self, msg):
        """Tell the agent who is in view; the Hailo node puts names in class_id."""
        now = self.get_clock().now().nanoseconds * 1e-9
        for detection in msg.detections:
            for result in detection.results:
                label, _, name = result.hypothesis.class_id.partition(': ')
                name = name.strip()
                if not name or name.lower() == 'unknown':
                    continue
                if now - self.faces_seen.get(name, -1e9) >= self.face_repeat:
                    self.get_logger().info(f'face seen: {name}')
                    self.emit({'kind': 'face_seen', 'name': name})
                self.faces_seen[name] = now

    def on_head_moving(self, msg):
        self.head_moving = msg.data

    def on_tracker_status(self, msg):
        lost = msg.active and msg.lost
        if msg.target != self.tracking:
            self.tracking = msg.target
            self.state_changed()
        if lost and not self.tracking_lost:
            self.get_logger().info(f'tracking: lost {msg.target}')
            self.emit({'kind': 'track_lost', 'target': msg.target})
        self.tracking_lost = lost

    async def track(self, target):
        """Follow target with the head (tracker node); returns (ok, message)."""
        if not self.track_client.service_is_ready():
            return False, 'my head tracking is not running'
        response = await self.track_client.call_async(Track.Request(target=target))
        return response.success, response.message

    def stop_tracking(self):
        if self.untrack_client.service_is_ready():
            self.untrack_client.call_async(Trigger.Request())

    async def look(self, pan_deg, tilt_deg):
        """Point the head (degrees, 0, 0 = straight ahead); returns (ok, message, pan, tilt)."""
        # The tracker would turn the head straight back (and replace this move)
        if self.untrack_client.service_is_ready():
            await self.untrack_client.call_async(Trigger.Request())
        if not self.head_client.wait_for_server(timeout_sec=1.0):
            return False, 'my head is not working right now', None, None
        goal = MoveHead.Goal(pan=math.radians(pan_deg), tilt=math.radians(tilt_deg))
        handle = await self.head_client.send_goal_async(goal)
        if not handle.accepted:
            return False, 'my head refused to move', None, None
        response = await handle.get_result_async()
        result = response.result
        pan, tilt = math.degrees(result.pan), math.degrees(result.tilt)
        return response.status == GoalStatus.STATUS_SUCCEEDED, result.message, pan, tilt

    async def capture_photo(self, timeout=3.0):
        """Return one JPEG from the camera (shrunk to photo_width), or None.

        Frames that arrive while the head moves or settles are skipped (blurred,
        or not where the head points); the timeout counts from the call.
        """
        future = Future()

        def on_image(msg):
            if not future.done() and not self.head_moving:
                future.set_result(msg)

        sub = self.create_subscription(CompressedImage, self.photo_topic, on_image,
                                       qos_profile_sensor_data, callback_group=self.group)
        timer = self.create_timer(timeout, lambda: future.done() or future.set_result(None),
                                  callback_group=self.group)
        try:
            msg = await future
        finally:
            self.destroy_subscription(sub)
            self.destroy_timer(timer)
        if msg is None:
            return None
        return self.shrink(bytes(msg.data))

    def shrink(self, jpeg):
        try:
            import cv2
            import numpy as np
        except ImportError:
            return jpeg
        image = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None or image.shape[1] <= self.photo_width:
            return jpeg
        height = round(image.shape[0] * self.photo_width / image.shape[1])
        image = cv2.resize(image, (self.photo_width, height), interpolation=cv2.INTER_AREA)
        ok, out = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 80])
        return out.tobytes() if ok else jpeg

    async def remember_face(self, name):
        """Ask the Hailo node to save the face in view as name; returns (ok, message)."""
        if not self.save_face_client.service_is_ready():
            return False, 'face recognition is not running'
        response = await self.save_face_client.call_async(
            SaveGalleryItem.Request(id=name, append=False))
        # hailo_msgs/srv/SaveGalleryItem result codes
        messages = {0: f'I will remember {name}',
                    1: f'I already know you, {name}',
                    2: 'I already know this face under another name',
                    3: f'I know someone else called {name}',
                    4: 'I see more than one face; only one person please',
                    5: "I can't see a face"}
        return response.result in (0, 1), messages.get(response.result, response.message)

    def set_pose_estimate(self, place):
        """Tell AMCL the robot is at a place (startup localization from a photo)."""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = place.x
        msg.pose.pose.position.y = place.y
        msg.pose.pose.orientation.z = math.sin(place.yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(place.yaw / 2.0)
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.25   # 0.5 m
        msg.pose.covariance[35] = 0.25                           # 0.5 rad
        self.initialpose_pub.publish(msg)
        self.get_logger().info(f'pose estimate: at {place.name}')

    # Voice

    def on_voice_stop(self, msg):
        self.get_logger().info('stop (voice)')
        self.stop()
        self.emit({'kind': 'stopped_by_voice'})

    def on_utterance(self, msg):
        command = None
        if msg.command and msg.command_confidence >= self.command_confidence:
            command = parse(msg.command, self.place_names())
        if command is not None:
            self.local_command(msg.command, command)
        elif self.mode == 'remote':
            audio = base64.b64encode(bytes(msg.samples.tobytes())).decode('ascii')
            self.link.send({'type': 'utterance', 'audio': audio,
                            'rate': msg.sample_rate, 'follow_up': msg.follow_up})
        elif not msg.follow_up:
            self.say("I'm offline right now. I can only go to places I know, "
                     'come back, or wait.')

    async def sleep(self, seconds):
        """Wait without blocking the executor (for coroutines on it)."""
        future = Future()
        timer = self.create_timer(seconds, lambda: future.done() or future.set_result(True),
                                  callback_group=self.group)
        try:
            await future
        finally:
            self.destroy_timer(timer)

    async def spin_localize(self):
        """Turn in place until AMCL is sure where the robot is.

        localize_turns full turns in one go with the normal Spin (with its
        collision checks), then a check; again if AMCL is not 'sure', at most
        localize_rounds times. Then says so and sends the agent a 'lost' event
        (it may then try a photo).
        """
        if self.localizing:
            return
        self.localizing = True
        try:
            if self.certainty()[0] == 'sure':
                self.say('I already know where I am.')
                return
            self.say('Let me look around.')
            for round_ in range(self.localize_rounds):
                ok, message = await self.spin(2.0 * math.pi * self.localize_turns)
                if not ok:
                    self.get_logger().warning(f'look around: turn failed: {message}')
                    if message in ('cancelled', 'replaced by a new goal'):
                        return
                    break
                await self.sleep(1.0)  # let AMCL take in the new scans
                word, pos_std, yaw_std = self.certainty()
                self.get_logger().info(f'look around: round {round_ + 1}: {word} '
                                       f'(+-{pos_std:.2f} m, +-{math.degrees(yaw_std):.0f} deg)')
                if word == 'sure':
                    self.say('Got it, I know where I am now.')
                    self.emit({'kind': 'localized'})
                    return
            self.say("I'm not sure where I am.")
            self.emit({'kind': 'lost'})
        finally:
            self.localizing = False

    def local_command(self, text, command):
        """Do a recognised voice command; tell the agent it was done."""
        self.get_logger().info(f'voice command: {text}')
        self.last_command = text
        if command['action'] == 'stop':
            self.stop()
            done = 'stopped'
            self.say('Okay.')
        elif command['action'] == 'localize':
            self.executor.create_task(self.spin_localize)
            done = 'started turning in place until I know where I am'
        else:
            try:
                place = self.find_place(command['place'])
            except ValueError:
                done = f"did not go: I don't know a place called {command['place']}"
                self.say(f"I don't know where {command['place']} is yet.")
            else:
                self.executor.create_task(self.go_to, place)
                done = f'started going to {place.name}'
                self.say('Going home.' if place.name == 'home' else
                         f'Going to the {place.name}.')
        if self.link is not None:
            self.link.send({'type': 'utterance', 'handled': True, 'text': text,
                            'action': done})

    # ROS actions and services

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
            place = self.find_place(goal_handle.request.place)
        except ValueError as e:
            return self.finish(goal_handle, result, False, str(e))

        def feedback(msg):
            t = msg.feedback.estimated_time_remaining
            goal_handle.publish_feedback(GoToPlace.Feedback(
                distance_remaining=msg.feedback.distance_remaining,
                seconds_remaining=t.sec + t.nanosec * 1e-9))

        ok, message = await self.go_to(place, goal_handle, feedback)
        return self.finish(goal_handle, result, ok, message)

    async def execute_turn(self, goal_handle):
        def feedback(msg):
            goal_handle.publish_feedback(
                Turn.Feedback(turned=msg.feedback.angular_distance_traveled))

        ok, message = await self.turn(goal_handle.request.angle, goal_handle, feedback)
        return self.finish(goal_handle, Turn.Result(), ok, message)

    async def execute_drive(self, goal_handle):
        def feedback(msg):
            goal_handle.publish_feedback(Drive.Feedback(driven=msg.feedback.distance_traveled))

        ok, message = await self.drive(goal_handle.request.distance, goal_handle, feedback)
        return self.finish(goal_handle, Drive.Result(), ok, message)

    def on_stop(self, request, response):
        self.stop()
        response.success = True
        response.message = 'stopping'
        return response

    def on_where_am_i(self, request, response):
        w = self.where_am_i()
        response.localized = w['localized']
        response.certainty = w['certainty']
        response.description = w['description']
        response.x, response.y, response.yaw = w.get('x', 0.0), w.get('y', 0.0), w.get('yaw', 0.0)
        response.nearest_place = w.get('nearest_place', '')
        response.nearest_distance = float(w.get('nearest_distance', 0.0))
        response.room = w.get('room', '')
        response.facing_place = w.get('facing_place', '')
        return response

    def on_get_state(self, request, response):
        s = self.state()
        response.mode = s['mode']
        response.map = s['map']
        response.localized = s['localized']
        response.certainty = s['certainty']
        response.x, response.y, response.yaw = s.get('x', 0.0), s.get('y', 0.0), s.get('yaw', 0.0)
        response.position_std = float(s['position_std'])
        response.yaw_std = float(s['yaw_std'])
        response.battery_volts = float(s['battery_volts'])
        response.activity = s['activity']
        return response

    def on_save_place(self, request, response):
        try:
            place = self.save_place(request.name, request.description, request.room,
                                    request.from_pick, request.guess)
        except (ValueError, OSError) as e:
            response.message = str(e)
            return response
        response.success = True
        response.message = f'saved {place.name}'
        response.place = to_msg(place)
        return response

    def on_edit_place(self, request, response):
        try:
            place = self.edit_place(request.name, request.new_name, request.description,
                                    request.room, request.confirm)
        except (ValueError, OSError) as e:
            response.message = str(e)
            return response
        response.success = True
        response.message = f'updated {place.name}'
        response.place = to_msg(place)
        return response

    def on_delete_place(self, request, response):
        try:
            place = self.delete_place(request.name)
        except (ValueError, OSError) as e:
            response.message = str(e)
            return response
        response.success = True
        response.message = f'deleted {place.name}'
        return response

    def on_list_places(self, request, response):
        response.map = self.map_name
        with self.places_lock:
            response.places = [to_msg(p) for p in self.places.sorted_places()]
        return response

    def publish_markers(self):
        """Publish an arrow and a label per place: green confirmed, orange guesses."""
        msg = MarkerArray()
        msg.markers.append(Marker(action=Marker.DELETEALL))
        with self.places_lock:
            places = self.places.sorted_places()
        for i, place in enumerate(places):
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


class LinkHandler:
    """The skills as the Mac agent calls them (rainbow_agent/skills.py, one to one).

    Calls arrive on the link's asyncio loop; each skill runs on the ROS executor
    (where the Nav2 clients and the TF buffer live) and the result comes back.
    Units are the agent's: degrees and metres.
    """

    def __init__(self, node):
        self.node = node

    def on_connected(self, connected):
        node = self.node
        node.mode = 'remote' if connected else 'local'
        node.get_logger().info(f'mode: {node.mode}')
        if not connected:
            node.on_agent_status({'phase': 'offline', 'skill': ''})
        else:
            node.publish_status()

    def on_agent_status(self, status):
        self.node.on_agent_status(status)

    def on_agent_log(self, level, text):
        self.node.on_agent_log(level, text)

    def state(self):
        s = self.node.state()
        s.pop('ok', None)
        return s

    def say(self, text):
        self.node.executor.create_task(self.node.say, text)

    async def call(self, skill, args):
        handler = getattr(self, f'skill_{skill}', None)
        if handler is None:
            return {'ok': False, 'message': f'unknown skill {skill}'}
        return await self._in_ros(skill, handler, **args)

    def _in_ros(self, skill, fn, **kwargs):
        """Run fn (sync or async) on the ROS executor; an asyncio future for its result."""
        loop = asyncio.get_running_loop()
        future = loop.create_future()

        async def run():
            try:
                result = fn(**kwargs)
                if asyncio.iscoroutine(result):
                    result = await result
                outcome = (result, None)
            except TypeError:
                outcome = ({'ok': False, 'message': f'wrong arguments for {skill}'}, None)
            except (ValueError, OSError) as e:
                outcome = ({'ok': False, 'message': str(e)}, None)
            except Exception as e:  # report, do not lose the call
                outcome = (None, e)
            loop.call_soon_threadsafe(self._settle, future, outcome)

        self.node.executor.create_task(run)
        return future

    @staticmethod
    def _settle(future, outcome):
        if future.done():
            return
        result, error = outcome
        if error is not None:
            future.set_exception(error)
        else:
            future.set_result(result)

    # Skills (on the ROS executor)

    def skill_go_to_place(self, place):
        node = self.node
        target = node.find_place(place)
        node.executor.create_task(node.go_to, target)
        return {'ok': True, 'message': f'started going to {target.name}'}

    async def skill_turn(self, degrees):
        ok, message = await self.node.turn(math.radians(float(degrees)))
        out = {'ok': ok, 'message': f'turned {float(degrees):.0f} degrees' if ok else message}
        if ok:
            out['now'] = self.node.where_am_i()['description']
        return out

    async def skill_drive_forward(self, meters):
        ok, message = await self.node.drive(float(meters))
        out = {'ok': ok, 'message': f'drove {float(meters):.2f} m' if ok else message}
        if ok:
            out['now'] = self.node.where_am_i()['description']
        return out

    def skill_stop(self):
        self.node.stop()
        return {'ok': True, 'message': 'stopped'}

    def skill_where_am_i(self):
        return self.node.where_am_i()

    def skill_get_state(self):
        return self.node.state()

    def skill_list_places(self):
        node = self.node
        with node.places_lock:
            places = [to_dict(p) for p in node.places.sorted_places()]
        return {'ok': True, 'places': places}

    def skill_save_place(self, name, description='', room='', guess=False):
        place = self.node.save_place(name, description, room, guess=bool(guess))
        return {'ok': True, 'message': f'saved {place.name}'}

    def skill_edit_place(self, name, new_name='', description='', room='', confirm=False):
        place = self.node.edit_place(name, new_name, description, room, bool(confirm))
        return {'ok': True, 'message': f'updated {place.name}'}

    def skill_delete_place(self, name):
        place = self.node.delete_place(name)
        return {'ok': True, 'message': f'deleted {place.name}'}

    async def skill_look(self, pan_degrees=0.0, tilt_degrees=0.0):
        # The agent's tilt is positive up; the head's (URDF) positive down
        ok, message, pan, tilt = await self.node.look(float(pan_degrees), -float(tilt_degrees))
        out = {'ok': ok, 'message': message}
        if pan is not None:
            out['pan_degrees'], out['tilt_degrees'] = round(pan), round(-tilt)
            if ok:
                out['message'] = f'looking {round(pan)} deg left, {round(-tilt)} deg up'
                # Without this the model made up what was there
                out['seen'] = 'nothing yet: call capture_photo to see what is there'
        return out

    async def skill_track(self, what):
        ok, message = await self.node.track(str(what).strip())
        out = {'ok': ok, 'message': message}
        if ok:
            # As for look: without this the model made up where the thing was
            out['seen'] = ('not known: you do not know if it is in view now; you are '
                           'told if it is lost; call capture_photo to see')
        return out

    def skill_stop_tracking(self):
        self.node.stop_tracking()
        return {'ok': True, 'message': 'stopped following; my head stays where it is'}

    async def skill_capture_photo(self):
        jpeg = await self.node.capture_photo()
        if jpeg is None:
            return {'ok': False, 'message': 'my camera is not sending pictures right now'}
        return {'ok': True, 'image': base64.b64encode(jpeg).decode('ascii'),
                'where': self.node.where_am_i()['description']}

    async def skill_remember_face(self, name):
        ok, message = await self.node.remember_face(str(name).strip())
        return {'ok': ok, 'message': message}

    def skill_set_pose_estimate(self, place):
        target = self.node.find_place(place)
        self.node.set_pose_estimate(target)
        return {'ok': True, 'message': f'told the map I am at {target.name}'}


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
