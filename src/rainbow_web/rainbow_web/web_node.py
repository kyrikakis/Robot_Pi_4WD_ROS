"""Rainbow's phone remote: a web page served from the robot, http://rainbow.local/.

One process serves the page (static files from this package's share/static) and a
WebSocket (/ws) over which the page drives the robot, moves the head, and gets
the camera, the map and the robot's state. Everything the Foxglove layout does,
in a layout for a phone held sideways.

Nothing is subscribed while no page is open: the node subscribes to a topic only
while some page needs it (the camera only while it is on screen, the map, scan,
plan and TF only while the map is, costmaps only when switched on).

Page -> robot (JSON):
  {"t": "want", "camera": bool, "map": bool, "global_costmap": bool,
   "local_costmap": bool, "quality": "low" | "high"}
  {"t": "drive", "x": -1..1 (forward), "z": -1..1 (turn left), "speed": 0..1}
      sent at ~20 Hz while the stick is held. Published on cmd_vel_stamped (the
      velocity watchdog's input, like Foxglove's Teleop panel; past Nav2's
      collision monitor). The watchdog stops the robot 0.2 s after the last one,
      so a dropped connection stops it.
  {"t": "head", "x": -1..1 (right), "y": -1..1 (up)}   on /head/jog
  {"t": "call", "id": n, "op": "...", "args": {...}}    answered with a "result"
Robot -> page: JSON messages (status, map, pose, scan, plan, places, log, result)
and binary messages: camera JPEG frames.

Parameters: port (80), camera_topic, max_linear (m/s), max_reverse (m/s),
max_angular (rad/s), max_head_rate (rad/s), low_width (px, the "low" camera
quality's width).
"""

import asyncio
import base64
from functools import partial
import http
import json
import mimetypes
import os
import signal
import threading
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from rainbow_interfaces.action import GoToPlace
from rainbow_interfaces.msg import BrainStatus, TrackerStatus
from rainbow_interfaces.srv import (DeletePlace, ListPlaces, SavePlace, Say,
                                    SetExpression, SetHeadPose, Track)
from rainbow_web import grids
from rcl_interfaces.msg import Log
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, qos_profile_sensor_data, QoSProfile,
                       ReliabilityPolicy)
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, LaserScan, Temperature
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty, Trigger
import tf2_ros
from visualization_msgs.msg import MarkerArray
import websockets

try:
    import cv2
    import numpy as np
except ImportError:  # only the "low" camera quality needs them
    cv2 = None

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
RELIABLE_1 = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
LOG_KEEP = 60
LOG_NAMES = {10: 'debug', 20: 'info', 30: 'warn', 40: 'error', 50: 'fatal'}


class Client:
    """One open page."""

    def __init__(self, ws):
        self.ws = ws
        self.want = {'camera': False, 'map': False, 'global_costmap': False,
                     'local_costmap': False}
        self.quality = 'low'
        self.camera_busy = False
        self.driving = False
        self.last_drive = 0.0


class WebNode(Node):

    def __init__(self):
        super().__init__('web')
        param = self.declare_parameter
        self.port = param('port', 80).value
        self.camera_topic = param(
            'camera_topic', '/hailo_face_recognition/image_bbx/compressed').value
        self.max_linear = param('max_linear', 0.30).value
        self.max_reverse = param('max_reverse', 0.10).value
        self.max_angular = param('max_angular', 1.2).value
        self.max_head_rate = param('max_head_rate', 1.2).value
        self.low_width = param('low_width', 640).value
        self.static_dir = os.path.join(get_package_share_directory('rainbow_web'), 'static')

        self.loop = None          # the asyncio loop, set by Server
        self.server = None
        self.lock = threading.Lock()
        self.wanted = set()       # topic groups some page needs (written by the loop)
        self.subs = {}            # group -> [subscriptions]
        self.tf_buffer = None
        self.tf_listener = None
        self.scan = None
        self.scan_sent = 0.0
        self.plan_sent = 0.0
        self.costmap_sent = {}
        self.map_frame = 'map'
        self.base_frame = 'base_footprint'

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_stamped', 10)
        self.jog_pub = self.create_publisher(Twist, '/head/jog', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.pick_pub = self.create_publisher(PoseStamped, '/places/pick', 10)

        self.srv = {
            'stop': self.create_client(Trigger, '/brain/stop'),
            'relocalize': self.create_client(Empty, '/reinitialize_global_localization'),
            'head_center': self.create_client(Trigger, '/head/center'),
            'head_pose': self.create_client(SetHeadPose, '/head/set_pose'),
            'eyes': self.create_client(SetExpression, '/eyes/set_expression'),
            'say': self.create_client(Say, '/speech/say'),
            'track': self.create_client(Track, '/tracker/start'),
            'track_stop': self.create_client(Trigger, '/tracker/stop'),
            'places': self.create_client(ListPlaces, '/places/list'),
            'place_save': self.create_client(SavePlace, '/places/save'),
            'place_delete': self.create_client(DeletePlace, '/places/delete'),
        }
        self.go_client = ActionClient(self, GoToPlace, '/brain/go_to_place')

        self.create_timer(0.25, self.reconcile)
        self.create_timer(0.1, self.send_pose)

    # ---- from the ROS thread to the pages -------------------------------------

    def push(self, fn, *args):
        if self.server is not None:
            self.loop.call_soon_threadsafe(fn, *args)

    def broadcast(self, message, group=None):
        self.push(self.server.broadcast, message, group)

    # ---- subscriptions, made and dropped as pages need them --------------------

    def set_wanted(self, groups):
        with self.lock:
            self.wanted = set(groups)

    def reconcile(self):
        with self.lock:
            wanted = set(self.wanted)
        for group in list(self.subs):
            if group not in wanted:
                for sub in self.subs.pop(group):
                    self.destroy_subscription(sub)
                if group == 'map':
                    self.stop_tf()
        for group in wanted - set(self.subs):
            self.subs[group] = self.make_subs(group)

    def make_subs(self, group):
        sub = self.create_subscription
        if group == 'base':
            return [
                sub(BrainStatus, '/brain/status', self.on_status, LATCHED),
                sub(Bool, '/head/moving', partial(self.on_flag, 'head_moving'), LATCHED),
                sub(Bool, '/lifted', partial(self.on_flag, 'lifted'), LATCHED),
                sub(TrackerStatus, '/tracker/status', self.on_tracker, LATCHED),
                sub(Temperature, '/rpi5_temperature', self.on_temperature, 10),
                sub(String, '/esp_firmware', self.on_firmware, LATCHED),
                sub(MarkerArray, '/places/markers', self.on_markers, LATCHED),
                sub(Log, '/agent/log', partial(self.on_log, 'agent'), 50),
                sub(Log, '/rosout', partial(self.on_log, 'ros'), 50),
            ]
        if group == 'camera':
            return [sub(CompressedImage, self.camera_topic, self.on_image,
                        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))]
        if group == 'map':
            self.start_tf()
            return [
                sub(OccupancyGrid, '/map', self.on_map, LATCHED),
                sub(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data),
                sub(Path, '/plan_viz', self.on_plan, RELIABLE_1),
            ]
        if group in ('global_costmap', 'local_costmap'):
            self.costmap_sent[group] = 0.0
            return [sub(OccupancyGrid, f'/{group}/costmap',
                        partial(self.on_costmap, group), RELIABLE_1)]
        return []

    def start_tf(self):
        if self.tf_listener is None:
            self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=False)

    def stop_tf(self):
        if self.tf_listener is not None:
            self.tf_listener.unregister()
            self.tf_listener = None
            self.tf_buffer = None
            self.scan = None

    def lookup(self, frame):
        """Return the latest pose of frame in the map, (x, y, yaw), or None."""
        if self.tf_buffer is None or not frame:
            return None
        try:
            t = self.tf_buffer.lookup_transform(self.map_frame, frame, Time())
        except tf2_ros.TransformException:
            return None
        q = t.transform.rotation
        return (t.transform.translation.x, t.transform.translation.y,
                grids.yaw_from_quaternion(q.x, q.y, q.z, q.w))

    # ---- topic callbacks ------------------------------------------------------

    def on_status(self, msg):
        a = msg.agent
        self.broadcast({
            't': 'status', 'mode': msg.mode, 'map': msg.map, 'activity': msg.activity,
            'localized': msg.localized, 'certainty': msg.certainty,
            'x': msg.x, 'y': msg.y, 'yaw': msg.yaw,
            'position_std': msg.position_std, 'yaw_std': msg.yaw_std,
            'battery': round(msg.battery_volts, 2),
            'battery_age': round(msg.battery_age, 1), 'scan_age': round(msg.scan_age, 1),
            'odom_age': round(msg.odom_age, 1),
            'nav2': msg.nav2_ready, 'speech': msg.speech_ready, 'faces': msg.faces_ready,
            'places': msg.places, 'last_event': msg.last_event,
            'last_command': msg.last_command, 'uptime': round(msg.uptime),
            'agent': {k: getattr(a, k) for k in a.get_fields_and_field_types()
                      if isinstance(getattr(a, k), (bool, int, float, str))},
        })

    def on_flag(self, name, msg):
        self.broadcast({'t': 'flag', 'name': name, 'value': msg.data})

    def on_tracker(self, msg):
        self.broadcast({'t': 'tracker', 'target': msg.target, 'active': msg.active,
                        'seen': msg.seen, 'lost': msg.lost, 'label': msg.label})

    def on_temperature(self, msg):
        self.broadcast({'t': 'flag', 'name': 'temperature',
                        'value': round(msg.temperature, 1)})

    def on_firmware(self, msg):
        self.broadcast({'t': 'flag', 'name': 'firmware', 'value': msg.data})

    def on_markers(self, _msg):
        # The places changed (the brain republishes its markers): send the new list
        self.push(self.server.refresh_places)

    def on_log(self, source, msg):
        if source == 'ros' and msg.level < 30:
            return
        entry = {'t': 'log', 'source': source, 'level': LOG_NAMES.get(msg.level, 'info'),
                 'name': msg.name, 'msg': msg.msg[:500],
                 'time': msg.stamp.sec + msg.stamp.nanosec * 1e-9}
        self.push(self.server.add_log, entry)

    def on_image(self, msg):
        self.push(self.server.on_frame, bytes(msg.data))

    def on_map(self, msg):
        info = msg.info
        o = info.origin
        png = grids.encode_png(grids.map_rgba(msg.data, info.width, info.height))
        self.map_frame = msg.header.frame_id or 'map'
        self.push(self.server.on_map, {
            't': 'map', 'width': info.width, 'height': info.height,
            'resolution': info.resolution,
            'origin': [o.position.x, o.position.y, grids.yaw_from_quaternion(
                o.orientation.x, o.orientation.y, o.orientation.z, o.orientation.w)],
            'png': base64.b64encode(png).decode('ascii')})

    def on_costmap(self, group, msg):
        now = time.monotonic()
        if now - self.costmap_sent.get(group, 0.0) < 1.0:
            return
        frame = self.lookup(msg.header.frame_id)
        if frame is None:
            return
        self.costmap_sent[group] = now
        info = msg.info
        o = info.origin
        origin = grids.compose(frame, (o.position.x, o.position.y, grids.yaw_from_quaternion(
            o.orientation.x, o.orientation.y, o.orientation.z, o.orientation.w)))
        png = grids.encode_png(grids.costmap_rgba(msg.data, info.width, info.height))
        self.broadcast({'t': 'costmap', 'name': group, 'width': info.width,
                        'height': info.height, 'resolution': info.resolution,
                        'origin': list(origin),
                        'png': base64.b64encode(png).decode('ascii')}, group)

    def on_scan(self, msg):
        now = time.monotonic()
        if now - self.scan_sent < 0.2:
            return
        laser = self.lookup(msg.header.frame_id)
        if laser is None:
            return
        self.scan_sent = now
        step, ranges = grids.compact_scan(msg.ranges, msg.range_min, msg.range_max)
        self.broadcast({'t': 'scan', 'pose': [round(v, 3) for v in laser],
                        'angle_min': msg.angle_min, 'angle_inc': msg.angle_increment * step,
                        'ranges': ranges, 'time': time.time()}, 'map')

    def on_plan(self, msg):
        now = time.monotonic()
        if msg.poses and now - self.plan_sent < 0.5:
            return
        self.plan_sent = now
        pts = grids.thin_path([(p.pose.position.x, p.pose.position.y) for p in msg.poses])
        self.broadcast({'t': 'plan', 'points': pts}, 'map')

    def send_pose(self):
        if self.tf_buffer is None:
            return
        pose = self.lookup(self.base_frame)
        if pose is not None:
            self.broadcast({'t': 'pose', 'x': round(pose[0], 3), 'y': round(pose[1], 3),
                            'yaw': round(pose[2], 3)}, 'map')

    # ---- commands from the pages (called on the ROS side via the loop) ---------

    def drive(self, x, z, speed):
        linear, angular = grids.drive_command(x, z, speed, self.max_linear,
                                              self.max_reverse, self.max_angular)
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_pub.publish(msg)

    def jog(self, x, y):
        msg = Twist()
        msg.angular.z = -grids.clamp(float(x), -1.0, 1.0) * self.max_head_rate
        msg.linear.x = grids.clamp(float(y), -1.0, 1.0) * self.max_head_rate
        self.jog_pub.publish(msg)

    def pose_stamped(self, args):
        msg = PoseStamped()
        msg.header.frame_id = self.map_frame
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = float(args['x'])
        msg.pose.position.y = float(args['y'])
        q = grids.quaternion_from_yaw(float(args.get('yaw', 0.0)))
        o = msg.pose.orientation
        o.x, o.y, o.z, o.w = q
        return msg

    def initial_pose(self, args):
        p = self.pose_stamped(args)
        msg = PoseWithCovarianceStamped()
        msg.header = p.header
        msg.pose.pose = p.pose
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.25 ** 2      # as Foxglove's pose estimate
        cov[35] = 0.26 ** 2
        msg.pose.covariance = cov
        return msg


class Server:
    """The asyncio side: HTTP for the page, a WebSocket per open page."""

    def __init__(self, node):
        self.node = node
        self.loop = asyncio.get_running_loop()
        node.loop = self.loop
        node.server = self
        self.pages = set()
        self.logs = []
        self.last = {}           # latest status-like messages, for new pages
        self.map_msg = None
        self.places_msg = None
        self.frame = None
        self.frame_low = None
        self.low_task = None
        self.nav = None          # the running go-to-place goal, as {'place', 'remaining'}

    # ---- HTTP ------------------------------------------------------------------

    async def http(self, path, headers):
        if path.split('?')[0] == '/ws':
            return None          # go on with the WebSocket handshake
        name = path.split('?')[0].lstrip('/') or 'index.html'
        # normpath, not realpath: with colcon's --symlink-install the files are
        # symlinks into build/
        root = os.path.normpath(self.node.static_dir)
        full = os.path.normpath(os.path.join(root, name))
        if not full.startswith(root + os.sep) or not os.path.isfile(full):
            return http.HTTPStatus.NOT_FOUND, [('Content-Type', 'text/plain')], b'not found\n'
        with open(full, 'rb') as f:
            body = f.read()
        kind = mimetypes.guess_type(full)[0] or 'application/octet-stream'
        if full.endswith('.webmanifest'):
            kind = 'application/manifest+json'
        headers = [('Content-Type', kind), ('Content-Length', str(len(body))),
                   ('Cache-Control', 'no-cache'), ('Connection', 'close')]
        return http.HTTPStatus.OK, headers, body

    async def run(self):
        async with websockets.serve(self.serve, '0.0.0.0', self.node.port,
                                    process_request=self.http, max_size=2 ** 16,
                                    ping_interval=5, ping_timeout=10):
            self.node.get_logger().info(f'web UI on http://0.0.0.0:{self.node.port}/')
            await asyncio.Future()

    # ---- pages -----------------------------------------------------------------

    async def serve(self, ws, path=None):
        page = Client(ws)
        self.pages.add(page)
        self.update_wanted()
        try:
            for msg in self.last.values():
                await ws.send(json.dumps(msg))
            if self.places_msg:
                await ws.send(json.dumps(self.places_msg))
            if self.logs:
                await ws.send(json.dumps({'t': 'logs', 'entries': self.logs}))
            self.refresh_places()
            async for raw in ws:
                if isinstance(raw, str):
                    await self.on_message(page, raw)
        except websockets.ConnectionClosed:
            pass
        finally:
            self.pages.discard(page)
            if page.driving:
                self.node.drive(0.0, 0.0, 0.0)
            self.update_wanted()

    def update_wanted(self):
        groups = set()
        if self.pages:
            groups.add('base')
        for page in self.pages:
            groups.update(g for g, on in page.want.items() if on)
        self.node.set_wanted(groups)

    async def on_message(self, page, raw):
        try:
            msg = json.loads(raw)
        except ValueError:
            return
        kind = msg.get('t')
        if kind == 'drive':
            x, z = float(msg.get('x', 0.0)), float(msg.get('z', 0.0))
            page.driving = x != 0.0 or z != 0.0
            page.last_drive = time.monotonic()
            self.node.drive(x, z, float(msg.get('speed', 1.0)))
        elif kind == 'head':
            self.node.jog(msg.get('x', 0.0), msg.get('y', 0.0))
        elif kind == 'want':
            had_map = page.want['map']
            for key in page.want:
                if key in msg:
                    page.want[key] = bool(msg[key])
            if msg.get('quality') in ('low', 'high'):
                page.quality = msg['quality']
            self.update_wanted()
            if page.want['map'] and not had_map and self.map_msg:
                await page.ws.send(json.dumps(self.map_msg))
        elif kind == 'call':
            asyncio.ensure_future(self.call(page, msg))
        elif kind == 'ping':
            await page.ws.send(json.dumps({'t': 'pong', 'time': msg.get('time')}))

    def broadcast(self, message, group=None):
        kind = message.get('t')
        if kind in ('status', 'tracker') or (kind == 'flag'):
            key = kind if kind != 'flag' else 'flag:' + message['name']
            self.last[key] = message
        data = json.dumps(message)
        for page in list(self.pages):
            if group is None or page.want.get(group):
                asyncio.ensure_future(self.send(page, data))

    async def send(self, page, data):
        try:
            await page.ws.send(data)
        except websockets.ConnectionClosed:
            pass

    def add_log(self, entry):
        self.logs.append(entry)
        del self.logs[:-LOG_KEEP]
        self.broadcast(entry)

    def on_map(self, message):
        self.map_msg = message
        self.broadcast(message, 'map')

    # ---- camera ----------------------------------------------------------------

    def on_frame(self, jpeg):
        self.frame = jpeg
        if any(p.want['camera'] and p.quality == 'high' for p in self.pages):
            for page in self.pages:
                if page.want['camera'] and page.quality == 'high':
                    self.send_frame(page, jpeg)
        if any(p.want['camera'] and p.quality == 'low' for p in self.pages):
            if self.low_task is None or self.low_task.done():
                self.low_task = asyncio.ensure_future(self.make_low(jpeg))

    async def make_low(self, jpeg):
        if cv2 is None:
            low = jpeg
        else:
            low = await self.loop.run_in_executor(None, shrink, jpeg, self.node.low_width)
        for page in list(self.pages):
            if page.want['camera'] and page.quality == 'low':
                self.send_frame(page, low)

    def send_frame(self, page, jpeg):
        if page.camera_busy:
            return               # still sending the last one: drop this frame
        page.camera_busy = True

        async def go():
            try:
                await page.ws.send(jpeg)
            except websockets.ConnectionClosed:
                pass
            finally:
                page.camera_busy = False
        asyncio.ensure_future(go())

    # ---- calls -----------------------------------------------------------------

    async def ros_call(self, name, request, timeout=10.0):
        client = self.node.srv[name]
        if not client.service_is_ready():
            raise RuntimeError(f'{client.srv_name} is not available')
        return await self.ros_future(client.call_async(request), timeout)

    async def ros_future(self, rfut, timeout):
        afut = self.loop.create_future()

        def done(f):
            self.loop.call_soon_threadsafe(lambda: afut.done() or afut.set_result(f))
        rfut.add_done_callback(done)
        f = await asyncio.wait_for(afut, timeout)
        return f.result()

    def refresh_places(self):
        asyncio.ensure_future(self._refresh_places())

    async def _refresh_places(self):
        try:
            res = await self.ros_call('places', ListPlaces.Request(), 5.0)
        except (RuntimeError, asyncio.TimeoutError):
            return False
        self.places_msg = {'t': 'places', 'map': res.map, 'places': [
            {'name': p.name, 'x': p.x, 'y': p.y, 'yaw': p.yaw, 'room': p.room,
             'description': p.description, 'source': p.source, 'confirmed': p.confirmed}
            for p in res.places]}
        self.broadcast(self.places_msg)
        return True

    async def call(self, page, msg):
        op, args, cid = msg.get('op'), msg.get('args') or {}, msg.get('id')
        try:
            ok, text = await self.do(op, args)
        except asyncio.TimeoutError:
            ok, text = False, f'{op}: no answer'
        except Exception as e:  # noqa: B902 - any failure goes back to the page
            ok, text = False, str(e) or type(e).__name__
        try:
            await page.ws.send(json.dumps({'t': 'result', 'id': cid, 'op': op,
                                           'ok': ok, 'message': text}))
        except websockets.ConnectionClosed:
            pass

    async def do(self, op, args):
        node = self.node
        if op == 'stop':
            res = await self.ros_call('stop', Trigger.Request())
            return res.success, res.message
        if op == 'relocalize':
            await self.ros_call('relocalize', Empty.Request())
            return True, 'searching the whole map: drive or turn a little'
        if op == 'head_center':
            res = await self.ros_call('head_center', Trigger.Request())
            return res.success, res.message
        if op == 'head_pose':
            res = await self.ros_call('head_pose', SetHeadPose.Request(
                pan=float(args.get('pan', 0.0)), tilt=float(args.get('tilt', 0.0))))
            return res.success, res.message
        if op == 'eyes':
            res = await self.ros_call('eyes', SetExpression.Request(
                expression=str(args.get('expression', 'happy'))))
            return res.success, res.message
        if op == 'say':
            res = await self.ros_call('say', Say.Request(text=str(args.get('text', ''))[:300]),
                                      60.0)
            return res.success, res.message or 'said'
        if op == 'track':
            res = await self.ros_call('track', Track.Request(
                target=str(args.get('target', 'face'))))
            return res.success, res.message
        if op == 'track_stop':
            res = await self.ros_call('track_stop', Trigger.Request())
            return res.success, res.message
        if op == 'goal':
            node.goal_pub.publish(node.pose_stamped(args))
            return True, 'going there'
        if op == 'initialpose':
            node.pose_pub.publish(node.initial_pose(args))
            return True, 'pose set'
        if op == 'place_save':
            from_pick = 'x' in args
            if from_pick:
                node.pick_pub.publish(node.pose_stamped(args))
                await asyncio.sleep(0.3)     # let the brain take the pick first
            res = await self.ros_call('place_save', SavePlace.Request(
                name=str(args.get('name', '')).strip(), room=str(args.get('room', '')),
                description=str(args.get('description', '')), from_pick=from_pick))
            self.refresh_places()
            return res.success, res.message
        if op == 'place_delete':
            res = await self.ros_call('place_delete', DeletePlace.Request(
                name=str(args.get('name', ''))))
            self.refresh_places()
            return res.success, res.message
        if op == 'place_go':
            return await self.go_to_place(str(args.get('place', '')))
        if op == 'places':
            ok = await self._refresh_places()
            return ok, '' if ok else 'the brain is not answering'
        return False, f'unknown command {op}'

    async def go_to_place(self, place):
        client = self.node.go_client
        if not client.server_is_ready():
            return False, 'the brain is not running'

        def feedback(fb):
            self.loop.call_soon_threadsafe(self.broadcast, {
                't': 'nav', 'place': place, 'active': True,
                'remaining': round(fb.feedback.distance_remaining, 2),
                'eta': round(fb.feedback.seconds_remaining)})
        handle = await self.ros_future(
            client.send_goal_async(GoToPlace.Goal(place=place), feedback_callback=feedback),
            10.0)
        if not handle.accepted:
            return False, f'not going to {place}'
        self.broadcast({'t': 'nav', 'place': place, 'active': True})
        asyncio.ensure_future(self.nav_result(place, handle))
        return True, f'going to {place}'

    async def nav_result(self, place, handle):
        try:
            res = await self.ros_future(handle.get_result_async(), 3600.0)
            ok, text = res.result.success, res.result.message
        except Exception as e:  # noqa: B902
            ok, text = False, str(e)
        self.broadcast({'t': 'nav', 'place': place, 'active': False, 'ok': ok,
                        'message': text})


def shrink(jpeg, width):
    """Return the JPEG at half its size or less (width px at most), quality 70."""
    buf = np.frombuffer(jpeg, dtype=np.uint8)
    img = cv2.imdecode(buf, cv2.IMREAD_REDUCED_COLOR_2)
    if img is None:
        return jpeg
    h, w = img.shape[:2]
    if w > width:
        img = cv2.resize(img, (width, int(round(h * width / w))), interpolation=cv2.INTER_AREA)
    ok, out = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 70])
    return out.tobytes() if ok else jpeg


def main():
    rclpy.init()
    node = WebNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def spin():
        try:
            executor.spin()
        except ExternalShutdownException:
            pass
    threading.Thread(target=spin, daemon=True).start()

    async def serve():
        # Stop on SIGINT / SIGTERM (supervisor). rclpy's own SIGTERM handler
        # does not end the process while the asyncio loop runs.
        loop = asyncio.get_running_loop()
        stopped = asyncio.Event()
        for sig in (signal.SIGINT, signal.SIGTERM):
            loop.add_signal_handler(sig, stopped.set)
        server = asyncio.ensure_future(Server(node).run())
        waiter = asyncio.ensure_future(stopped.wait())
        await asyncio.wait({server, waiter}, return_when=asyncio.FIRST_COMPLETED)
        for task in (server, waiter):
            task.cancel()
        if server.done() and not server.cancelled() and server.exception():
            raise server.exception()
    try:
        asyncio.run(serve())
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.drive(0.0, 0.0, 0.0)
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
