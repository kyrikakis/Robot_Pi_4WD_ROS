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
  /lifted             std_msgs/Bool (latched) out: the robot is in someone's hands
  /lidar/wanted       std_msgs/Bool out: a move needs the lidar (lidar_power)
Head: look points the head (the head node's /head/move action, which returns once
the head has settled); capture_photo skips frames while /head/moving is true.
track / stop_tracking start and stop the tracker node (/tracker/start, /tracker/stop);
stop and look stop tracking too. A target lost from view becomes a track_lost event.
Camera and faces (the Hailo container, docs/brain.md): capture_photo takes one
frame from photo_topic on demand; recognised faces on faces_topic become
face_seen events, and a face it does not know, steadily in view for unknown_after
seconds, a face_unknown event (the agent then asks the name); remember_face and
forget_face ask the Hailo node to save or delete a face.
set_pose_estimate publishes /initialpose for AMCL (startup localization).
Greeting people (rainbow_brain.greeting, on unless switched off with the
greet_people skill; saved in <data_dir>/settings.json): someone in view (a person
or a face) for a second while the robot is idle gets one look: the tracker
follows "face" for greet_follow_s, then the head centres. A known face is a
face_seen event the agent answers with a greeting (in local mode the brain says
hi itself); an unknown one is face_unknown. Driving, a look or track asked for,
or low battery take the head back.
Low battery (rainbow_brain.battery: <= 7.0 V for 30 s, until >= 7.6 V for 1 min):
every motion is cancelled and refused, the head centred, battery_stop_programs
stopped with supervisorctl (no dock yet, so driving is pointless and Nav2's CPU
is wasted), and every battery_repeat_s the robot says it is out of battery with
sad eyes; a battery_low event goes to the agent. Once charged the stopped
programs start again (battery_ok event).
Picked up (rainbow_brain.lift, from /bno055/imu: the robot rises > lift_rise_m or
tilts > lift_tilt_deg; put down when it lands, coming down and braking to a stop,
then quiet for lift_land_quiet_s, or after lift_still_s still): every motion is
cancelled and refused, /lifted (latched) is true, so the lidar goes to standby,
the robot says one of lift_phrases ("Wheee!") and a lifted event goes to the
agent. Put down: it says one of put_down_phrases, AMCL starts global
localization again (the location is lost) and a put_down event goes to the
agent (the agent does not speak on either); the robot
does not move by itself, the next move or "look around" relocalizes.
Lidar (lidar_power in rainbow_drivers keeps it in standby unless needed): every
move asks for it on /lidar/wanted (at least once a second) and waits up to
lidar_wait_s for the first scan before Nav2 gets the goal.
Reads: TF map -> base_footprint, /amcl_pose, /battery, /bno055/imu, /cmd_vel (to
know whether the robot is commanded to move); /scan and /odom_raw only for their
arrival times (not deserialized)
Link: WebSocket on link_port (8766); token in <data_dir>/link_token.

Places are stored in <data_dir>/places/<map_name>.yaml (rainbow_brain.places).
"""

import asyncio
import base64
import json
import math
import os
import subprocess
import threading
import time

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from hailo_msgs.srv import DeleteGalleryItem, SaveGalleryItem
from nav2_msgs.action import DriveOnHeading, NavigateToPose, Spin
from nav_msgs.msg import Odometry
from rainbow_brain.battery import BatteryMonitor
from rainbow_brain.commands import parse, phrases
from rainbow_brain.faces import save_outcome
from rainbow_brain.greeting import Attention
from rainbow_brain.lift import LIFT_PHRASES, LiftDetector, pick_phrase, PUT_DOWN_PHRASES
from rainbow_brain.link import LinkServer, load_token
from rainbow_brain.places import PlaceStore
from rainbow_interfaces.action import Drive, GoToPlace, MoveHead, Turn
from rainbow_interfaces.msg import (AgentStatus, BrainStatus, Place as PlaceMsg, TrackerStatus,
                                    Utterance)
from rainbow_interfaces.srv import (DeletePlace, EditPlace, GetState, ListPlaces, SavePlace, Say,
                                    SetExpression, Track, WhereAmI)
from rcl_interfaces.msg import Log
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import (ExternalShutdownException, MultiThreadedExecutor,
                             SingleThreadedExecutor)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rclpy.task import Future
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, Imu, LaserScan
from std_msgs.msg import Bool, Empty, String, UInt16
from std_srvs.srv import Empty as EmptySrv, Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection2DArray
from visualization_msgs.msg import Marker, MarkerArray

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)

# Nav2 actions cancelled by stop
BATTERY_EMPTY = 'my battery is empty, so I am not driving; please charge me'
CARRIED = 'someone is holding me, so I am not driving; put me down first'
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
        # A face it does not know, in view this long (gaps under 1 s), becomes a
        # face_unknown event, at most once per face_repeat
        self.unknown_after = self.declare_parameter('unknown_after', 3.0).value
        self.unknown_since = None
        self.unknown_last = -1e9
        self.unknown_asked = -1e9
        self.attention = Attention(
            notice_after=self.declare_parameter('greet_notice_s', 1.0).value,
            follow_for=self.declare_parameter('greet_follow_s', 20.0).value,
            gone_after=self.declare_parameter('greet_gone_s', 10.0).value)
        # A look that turns into a conversation (greeting, asking a name) lasts this long
        self.greet_talk_s = self.declare_parameter('greet_talk_s', 45.0).value
        self.settings_path = os.path.join(self.data_dir, 'settings.json')
        self.greet = self.load_settings().get('greet_people', True)
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
        # Frequent inputs (/tf ~40 Hz, the IMU 50 Hz, /cmd_vel and /odom_raw 20 Hz) go to
        # a second node with its own single-threaded executor (spin_inputs). On this node
        # every message woke the multi-threaded executor, which rebuilds its wait set over
        # all of the brain's ~70 entities in Python each time: ~23% of a core (2026-11-26).
        self.inputs = rclpy.create_node('brain_inputs')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.inputs)

        self.amcl = None
        self.battery = None
        param = self.declare_parameter
        self.battery_monitor = BatteryMonitor(
            low_volts=param('battery_low_volts', 7.0).value,
            low_for=param('battery_low_seconds', 30.0).value,
            ok_volts=param('battery_ok_volts', 7.6).value,
            ok_for=param('battery_ok_seconds', 60.0).value)
        self.battery_repeat = param('battery_repeat_s', 120.0).value
        # Supervisor programs stopped in low-battery mode, in the order to start them
        # again (the map before Nav2)
        self.battery_programs = list(param(
            'battery_stop_programs',
            ['rainbow_localization', 'rainbow_tof', 'rainbow_slam',
             'rainbow_navigation']).value)
        self.low_battery = False
        self.battery_stopped = []       # those of them that were running
        self.battery_timer = None
        # Picked up (rainbow_brain.lift)
        # 30 cm and 30 deg (the owner, 2026-11-28: driving was taken for a pick-up at
        # 10/12 cm and 20 deg;
        # 5 cm before 2026-11-26, when hand lifts read 5-6 cm right at it)
        self.lift = LiftDetector(rise_m=param('lift_rise_m', 0.30).value,
                                 rise_moving_m=param('lift_rise_moving_m', 0.30).value,
                                 tilt_deg=param('lift_tilt_deg', 30.0).value,
                                 still_s=param('lift_still_s', 1.0).value,
                                 land_quiet_s=param('lift_land_quiet_s', 0.25).value)
        self.lifted = False
        # Said when picked up / put down (a random one, not the last again); [] = quiet
        self.lift_phrases = list(param('lift_phrases', LIFT_PHRASES).value)
        self.put_down_phrases = list(param('put_down_phrases', PUT_DOWN_PHRASES).value)
        self.last_phrase = None
        self.last_motion = None         # when /cmd_vel last asked for motion (monotonic s)
        self.lifted_pub = self.create_publisher(Bool, '/lifted', LATCHED)
        self.lifted_pub.publish(Bool(data=False))
        self.inputs.create_subscription(Imu, param('imu_topic', '/bno055/imu').value,
                                        self.on_imu, qos_profile_sensor_data)
        self.inputs.create_subscription(Twist, '/cmd_vel', self.on_cmd_vel, 10)
        self.global_localization_client = self.create_client(
            EmptySrv, '/reinitialize_global_localization')
        # The lidar for moves (lidar_power): wait this long for the first scan
        self.lidar_wait = param('lidar_wait_s', 8.0).value
        self.lidar_users = 0            # moves running or about to (they need the lidar)
        self.lidar_wanted_pub = self.create_publisher(Bool, '/lidar/wanted', 10)
        self.create_timer(1.0, self.publish_lidar_wanted)
        self.pick = None
        self.pick_time = None
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, LATCHED)
        self.create_subscription(UInt16, '/battery', self.on_battery, qos_profile_sensor_data)
        # Health: when the ESP's messages last arrived (monotonic s); raw, never decoded
        self.started = time.monotonic()
        self.arrived = {'battery': None, 'scan': None, 'odom': None}
        self.inputs.create_subscription(LaserScan, '/scan', lambda _: self.mark('scan'),
                                        qos_profile_sensor_data, raw=True)
        self.inputs.create_subscription(Odometry, '/odom_raw', lambda _: self.mark('odom'),
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
        self.create_timer(1.0, self.update_attention)
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
        self.eyes_client = self.create_client(SetExpression, '/eyes/set_expression', **g)
        self.center_client = self.create_client(Trigger, '/head/center', **g)
        self.save_face_client = self.create_client(
            SaveGalleryItem, '/hailo_face_recognition/save_face', **g)
        self.delete_face_client = self.create_client(
            DeleteGalleryItem, '/hailo_face_recognition/delete_face', **g)

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
        change = self.battery_monitor.update(time.monotonic(), self.battery)
        if change == 'low':
            self.enter_low_battery()
        elif change == 'ok':
            self.leave_low_battery()

    # Picked up

    def on_cmd_vel(self, msg):
        if abs(msg.linear.x) > 1e-3 or abs(msg.angular.z) > 1e-3:
            self.last_motion = time.monotonic()

    def on_imu(self, msg):
        now = time.monotonic()
        moving = self.last_motion is not None and now - self.last_motion < 0.5
        o, a, w = msg.orientation, msg.linear_acceleration, msg.angular_velocity
        change = self.lift.update(now, (o.x, o.y, o.z, o.w), (a.x, a.y, a.z),
                                  (w.x, w.y, w.z), moving)
        if change == 'lifted':
            self.on_lifted(self.lift.why)
        elif change == 'down':
            self.on_put_down(self.lift.why)

    def on_lifted(self, why):
        self.get_logger().info(f'picked up ({why}): stopping; the lidar goes to standby')
        self.lifted = True
        self.lifted_pub.publish(Bool(data=True))
        self.stop()
        self.say_playful(self.lift_phrases)
        self.emit({'kind': 'lifted'})
        self.state_changed()

    def on_put_down(self, why=''):
        self.lifted = False
        self.lifted_pub.publish(Bool(data=False))
        relocalizing = self.global_localization_client.service_is_ready()
        if relocalizing:
            # The location is lost: spread AMCL's particles over the whole map
            self.global_localization_client.call_async(EmptySrv.Request())
        self.get_logger().info(f'put down ({why})' + (
            ': AMCL starts global localization again' if relocalizing else ''))
        self.say_playful(self.put_down_phrases)
        self.emit({'kind': 'put_down', 'relocalizing': relocalizing})
        self.state_changed()

    def say_playful(self, phrases):
        phrase = pick_phrase(phrases, self.last_phrase)
        if phrase:
            self.last_phrase = phrase
            self.say(phrase)

    # The lidar for moves

    def publish_lidar_wanted(self):
        wanted = (self.lidar_users > 0 or self.localizing) and not self.lifted
        self.lidar_wanted_pub.publish(Bool(data=wanted))

    async def wait_for_lidar(self):
        """Ask for the lidar and wait for a fresh scan (lidar_wait_s at most)."""
        self.publish_lidar_wanted()
        deadline = time.monotonic() + self.lidar_wait
        while time.monotonic() < deadline:
            seen = self.arrived['scan']
            if seen is not None and time.monotonic() - seen < 0.5:
                return True
            await self.sleep(0.2)
        self.get_logger().warning(f'no /scan after {self.lidar_wait:.0f} s: moving without '
                                  'the lidar')
        return False

    # Low battery

    def enter_low_battery(self):
        self.get_logger().warn(f'battery low ({self.battery:.1f} V): pausing everything')
        self.low_battery = True
        self.stop()
        if self.center_client.service_is_ready():
            self.center_client.call_async(Trigger.Request())
        threading.Thread(target=self.stop_programs, daemon=True).start()
        self.announce_low_battery()
        if self.battery_timer is None:
            self.battery_timer = self.create_timer(self.battery_repeat,
                                                   self.announce_low_battery)
        self.emit({'kind': 'battery_low', 'volts': round(self.battery, 1)})
        self.state_changed()

    def leave_low_battery(self):
        self.get_logger().info(f'battery charged ({self.battery:.1f} V): resuming')
        self.low_battery = False
        if self.battery_timer is not None:
            self.destroy_timer(self.battery_timer)
            self.battery_timer = None
        threading.Thread(target=self.start_programs, daemon=True).start()
        self.set_eyes('happy')
        self.say('Thank you! My battery is charged again.')
        self.emit({'kind': 'battery_ok', 'volts': round(self.battery, 1)})
        self.state_changed()

    def announce_low_battery(self):
        self.set_eyes('sad', fallback='closed')
        self.say('My battery is empty. Please charge me.')

    def set_eyes(self, expression, fallback=None):
        if not self.eyes_client.service_is_ready():
            return
        future = self.eyes_client.call_async(SetExpression.Request(expression=expression))
        if fallback:
            # An eyes node without this expression (not restarted yet) says so
            future.add_done_callback(
                lambda f: f.result() is not None and not f.result().success
                and self.eyes_client.call_async(SetExpression.Request(expression=fallback)))

    def supervisorctl(self, *args):
        try:
            out = subprocess.run(['supervisorctl', *args], capture_output=True, text=True,
                                 timeout=60)
            return out.stdout
        except (OSError, subprocess.SubprocessError) as e:
            self.get_logger().error(f'supervisorctl {" ".join(args)}: {e}')
            return ''

    def stop_programs(self):
        status = self.supervisorctl('status', *self.battery_programs)
        running = [line.split()[0] for line in status.splitlines()
                   if len(line.split()) > 1 and line.split()[1] in ('RUNNING', 'STARTING')]
        # Remember what ran, unless already stopped by an earlier low (brain restarted)
        if running or not self.battery_stopped:
            self.battery_stopped = running
        if running:
            self.get_logger().info(f'low battery: stopping {" ".join(running)}')
            self.supervisorctl('stop', *running)

    def start_programs(self):
        # Those that ran, in the configured order; after a brain restart in low-battery
        # mode nothing is remembered: then all but SLAM (mapping is started by hand)
        wanted = self.battery_stopped or [p for p in self.battery_programs
                                          if p != 'rainbow_slam']
        wanted = [p for p in self.battery_programs if p in wanted]
        if wanted:
            self.get_logger().info(f'battery charged: starting {" ".join(wanted)}')
            for program in wanted:
                self.supervisorctl('start', program)
        self.battery_stopped = []

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
               'battery_low': self.low_battery,
               'lifted': self.lifted,
               'greeting_people': self.greet,
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
        if self.low_battery:
            return False, BATTERY_EMPTY
        if self.lifted:
            return False, CARRIED
        self.lidar_users += 1
        try:
            return await self._run_nav2(label, client, goal, goal_handle, feedback)
        finally:
            self.lidar_users -= 1
            self.publish_lidar_wanted()

    async def _run_nav2(self, label, client, goal, goal_handle, feedback):
        if self.attention.attending:
            # The ToF in the head has to look ahead while driving
            self.release_attention()
            self.stop_tracking()
        if not client.wait_for_server(timeout_sec=1.0):
            return False, 'navigation is not running'
        await self.wait_for_lidar()
        if self.lifted:
            return False, CARRIED
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
        self.release_attention()
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
        unknown = False
        someone = False
        for detection in msg.detections:
            for result in detection.results:
                label, _, name = result.hypothesis.class_id.partition(':')
                label, name = label.strip(), name.strip()
                if label in ('face', 'person') and result.hypothesis.score >= 0.5:
                    someone = True
                if label != 'face':
                    continue        # the object detector's boxes (person, chair, ...)
                if not name or name.lower() == 'unknown':
                    unknown = unknown or result.hypothesis.score >= 0.5
                    continue
                if now - self.faces_seen.get(name, -1e9) >= self.face_repeat:
                    self.get_logger().info(f'face seen: {name}')
                    self.emit({'kind': 'face_seen', 'name': name, 'greet': self.greet})
                    self.attention.extend(time.monotonic(), self.greet_talk_s)
                    if self.greet and self.mode == 'local':
                        self.say(f'Hi {name}!')
                self.faces_seen[name] = now
        if someone:
            self.attention.seen(time.monotonic())
        if not unknown or not self.greet:
            return
        if now - self.unknown_last > 1.0:
            self.unknown_since = now
        self.unknown_last = now
        if now - self.unknown_since >= self.unknown_after and \
                now - self.unknown_asked >= self.face_repeat:
            self.unknown_asked = now
            self.get_logger().info('face seen: someone unknown')
            self.emit({'kind': 'face_unknown'})
            # Asking the name, then saving the face: keep them in view meanwhile
            self.attention.extend(time.monotonic(), self.greet_talk_s)
            if self.mode == 'local':
                self.say("Hello! I don't know you yet.")

    # Greeting people

    def update_attention(self):
        idle = self.current is None and not self.low_battery
        busy_head = bool(self.tracking) and not self.attention.attending
        change = self.attention.update(time.monotonic(), idle, self.greet, busy_head)
        if change == 'start':
            self.get_logger().info('someone is here: looking at them')
            self.unknown_asked = -1e9   # a stranger in this look is asked their name
            self.executor.create_task(self.track, 'face')
        elif change == 'end':
            self.get_logger().info('done looking at them')
            self.stop_tracking()

    def release_attention(self):
        """The head is wanted for something else (a look, track or drive)."""
        self.attention.release()

    def load_settings(self):
        try:
            with open(self.settings_path) as f:
                return json.load(f)
        except (OSError, ValueError):
            return {}

    def set_greet(self, enabled):
        self.greet = bool(enabled)
        if self.greet:
            self.attention.rearm()      # look at whoever is here now, not only newcomers
        settings = self.load_settings()
        settings['greet_people'] = self.greet
        try:
            os.makedirs(self.data_dir, exist_ok=True)
            with open(self.settings_path, 'w') as f:
                json.dump(settings, f)
        except OSError as e:
            self.get_logger().error(f'could not save {self.settings_path}: {e}')
        if not self.greet and self.attention.attending:
            self.attention.release()
            self.stop_tracking()
        self.get_logger().info(f'greeting people: {"on" if self.greet else "off"}')
        self.state_changed()

    def faces_again(self):
        """Report the faces in view again (agent just connected, or a new look):
        face events sent while nobody answered them are not lost."""
        self.faces_seen = {}
        self.unknown_asked = -1e9

    def names_in_view(self, within=2.0):
        now = self.get_clock().now().nanoseconds * 1e-9
        return [n for n, t in self.faces_seen.items() if now - t < within]

    def on_head_moving(self, msg):
        self.head_moving = msg.data

    def on_tracker_status(self, msg):
        lost = msg.active and msg.lost
        if msg.target != self.tracking:
            self.tracking = msg.target
            self.state_changed()
        if lost and not self.tracking_lost:
            self.get_logger().info(f'tracking: lost {msg.target}')
            if self.attention.attending:
                self.attention.release()   # a look of its own: the agent did not ask
            else:
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
        self.release_attention()
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

    async def remember_face(self, name, same_person=False):
        """Ask the Hailo node to save the face in view as name; returns (ok, message, next).

        same_person: they confirmed they are the one already saved under name (another
        angle or light), or this is one more view while enrolling: a face that looks
        different from the saved one is then added to that name. next: what the agent
        should do after (more views of a new face), or ''.
        """
        if not self.save_face_client.service_is_ready():
            return False, 'face recognition is not running', ''
        if name.islower():
            name = name.title()     # one spelling per person: "stefanos" -> "Stefanos"
        known = [n for n in self.names_in_view() if n != name]

        async def save(append):
            r = await self.save_face_client.call_async(
                SaveGalleryItem.Request(id=name, append=append))
            return save_outcome(r.result, r.message), r.message

        outcome, raw = await save(False)
        if outcome == 'distant' and same_person:
            # Safe here: append only renames when the face is close to another name's
            outcome, raw = await save(True)
            if outcome == 'added':
                self.get_logger().info(f'added a view of {name}')
                return True, f'I added this view of {name}', ''
        self.get_logger().info(f'remember face {name}: {outcome} ({raw})')
        if outcome == 'added':
            self.unknown_asked = self.get_clock().now().nanoseconds * 1e-9
            if same_person:
                return True, f'I added this view of {name}', ''
            self.executor.create_task(self.collect_views, name)
            return True, (f'I will remember {name}. Tell them to turn their head slowly to '
                          f'one side and the other, so I learn their face from more angles'), (
                'the extra views are added by themselves: call nothing')
        if outcome == 'same':
            return True, (f'I already know this view of {name}' if same_person
                          else f'I already know you, {name}'), ''
        if outcome == 'other_name':
            return False, (f'I already know this face as {known[0]}' if len(known) == 1
                           else 'I already know this face under another name'), ''
        if outcome == 'distant':
            return False, (f'I know a face called {name} that looks different'), (
                f'ask if they are the {name} you know (another angle or other light); if '
                f'yes, call remember_face again with same_person true; if not, ask for '
                f'another name, like with their last name')
        if outcome == 'many':
            return False, 'I see more than one face; only one person please', ''
        if outcome == 'none':
            return False, "I can't see a face", ''
        return False, raw or 'face recognition failed', ''

    async def collect_views(self, name, seconds=12.0, views=2):
        """After a new face: add up to views more views of it while they turn their head
        (one photo from one angle recognises poorly). A view only goes in when it looks
        different from the saved ones ('distant'), so append cannot rename anyone."""
        added = 0
        end = time.monotonic() + seconds
        while added < views and time.monotonic() < end:
            await self.sleep(1.0)
            r = await self.save_face_client.call_async(
                SaveGalleryItem.Request(id=name, append=False))
            outcome = save_outcome(r.result, r.message)
            if outcome in ('other_name', 'many'):
                break               # someone else, or more than one face: stop
            if outcome == 'distant':
                r = await self.save_face_client.call_async(
                    SaveGalleryItem.Request(id=name, append=True))
                if save_outcome(r.result, r.message) == 'added':
                    added += 1
        self.get_logger().info(f'{name}: {added} more view(s) saved')

    async def forget_face(self, name):
        """Ask the Hailo node to delete a face; returns (ok, message)."""
        if not self.delete_face_client.service_is_ready():
            return False, 'face recognition is not running'
        # The gallery's ids are exact: try the usual spellings
        for candidate in dict.fromkeys([name, name.title(), name.lower(), name.capitalize()]):
            response = await self.delete_face_client.call_async(
                DeleteGalleryItem.Request(id=candidate))
            if response.result == 0:
                self.faces_seen.pop(candidate, None)
                self.get_logger().info(f'forgot the face of {candidate}')
                return True, f'I forgot the face of {candidate}'
            if response.result != 1:
                return False, response.message or 'face recognition failed'
        return False, f'I do not know a face called {name}'

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
        # An answer to the agent's question goes to the agent: Vosk squeezes anything
        # into its few phrases ("Stefanos", the answer to "what is your name?", came out
        # as "stop" at 1.00). "stop" still works then: the stop word is its own detector.
        answer = msg.follow_up and self.mode == 'remote'
        if msg.command and msg.command_confidence >= self.command_confidence and not answer:
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
        elif command['action'] == 'stop_watching':
            self.release_attention()
            self.stop_tracking()
            self.set_greet(False)
            done = 'stopped following with my head and switched greeting people off'
            self.say("Okay, I'll stop watching.")
        elif command['action'] == 'greet_on':
            self.set_greet(True)
            done = 'switched greeting people on'
            self.say("Okay, I'll look out for people.")
        elif command['action'] == 'watch_me':
            self.release_attention()        # asked for: not ended by the greeting's timer
            self.executor.create_task(self.track, 'face')
            done = 'started following the face in front of me with my head'
            self.say("Okay, I'm watching you.")
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
            node.faces_again()      # who is in view now gets greeted or asked

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
        if node.low_battery:
            return {'ok': False, 'message': BATTERY_EMPTY}
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
        self.node.release_attention()      # asked for: not ended by the greeting's timer
        ok, message = await self.node.track(str(what).strip())
        out = {'ok': ok, 'message': message}
        if ok:
            # As for look: without this the model made up where the thing was
            out['seen'] = ('not known: you do not know if it is in view now; you are '
                           'told if it is lost; call capture_photo to see')
        return out

    def skill_stop_tracking(self):
        self.node.release_attention()
        self.node.stop_tracking()
        return {'ok': True, 'message': 'stopped following; my head is back in the middle'}

    async def skill_capture_photo(self):
        jpeg = await self.node.capture_photo()
        if jpeg is None:
            return {'ok': False, 'message': 'my camera is not sending pictures right now'}
        return {'ok': True, 'image': base64.b64encode(jpeg).decode('ascii'),
                'where': self.node.where_am_i()['description']}

    async def skill_remember_face(self, name, same_person=False):
        ok, message, next_step = await self.node.remember_face(
            str(name).strip(), same_person in (True, 'true', 'True', 1))
        out = {'ok': ok, 'message': message}
        if next_step:
            out['next'] = next_step
        return out

    def skill_greet_people(self, enabled):
        self.node.set_greet(enabled in (True, 'true', 'True', 1))
        return {'ok': True, 'message': 'I will greet people I see' if self.node.greet
                else 'I will not turn to people or greet them until you say so'}

    async def skill_forget_face(self, name):
        ok, message = await self.node.forget_face(str(name).strip())
        return {'ok': ok, 'message': message}

    def skill_set_pose_estimate(self, place):
        target = self.node.find_place(place)
        self.node.set_pose_estimate(target)
        return {'ok': True, 'message': f'told the map I am at {target.name}'}


def spin_inputs(node):
    """Spin the brain's frequent inputs (BrainNode.inputs) on their own thread."""
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except ExternalShutdownException:
        pass


def main(args=None):
    rclpy.init(args=args)
    node = BrainNode()
    threading.Thread(target=spin_inputs, args=(node.inputs,), daemon=True).start()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.inputs.destroy_node()
        node.destroy_node()
        rclpy.try_shutdown()
