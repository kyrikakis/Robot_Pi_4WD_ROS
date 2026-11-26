"""Head tracker: keeps the head pointed at something the detectors see.

Started with a target ("face", a person's name, or an object detector's label;
rainbow_brain.tracking matches them), it follows the matching detection on
detection_topics (vision_msgs/Detection2DArray: the Hailo's face recognition
and object detector) by sending /head/target. Each step aims from the head
pose at the time the frame was taken (from /joint_states), so the head's own
motion and the detector's delay do not make it overshoot. Following "face" with a
person in view but neither a face nor a usable body box (the Hailo sends NaN boxes
for near, big things), the head tilts up a step every search_period_s. The head stays still
while the target is in a quiet zone in the middle of the view (zone_pan_deg /
zone_tilt_deg). When the target is lost (out of view for lost_after_s) the head
goes back to the centre.

The detections' boxes are turned into angles with the camera's field of view
(hfov_deg / vfov_deg). The Hailo's boxes are normalized (0-1, normalized_boxes);
for a detector that sends pixels, set normalized_boxes false and the image size
(image_width / image_height; 0 = read once from the first frame on photo_topic).
The FOV defaults are the Camera Module 3's datasheet values, not measured on the
robot. The Hailo stamps its detections when it publishes them, after the frame
was taken: frame_latency_s is taken off every stamp (estimated, to be measured).

Interfaces (node name tracker):
  ~/start   rainbow_interfaces/srv/Track   start (or switch) tracking
  ~/stop    std_srvs/srv/Trigger           stop; the head goes back to the centre
  ~/status  rainbow_interfaces/msg/TrackerStatus (latched), on change and every second
  /head/target  rainbow_interfaces/msg/HeadTarget (out)
"""

from collections import deque
import math
import threading

from rainbow_brain.tracking import (aim_point, FALLBACK, has_label, head_target, matches, normalize,
                                    pick, pixel_to_angles, search_up)
from rainbow_interfaces.msg import HeadTarget, TrackerStatus
from rainbow_interfaces.srv import Track
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, JointState
from std_srvs.srv import Trigger
from vision_msgs.msg import Detection2DArray

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class TrackerNode(Node):

    def __init__(self):
        super().__init__('tracker')
        param = self.declare_parameter
        topics = param('detection_topics', ['/hailo_face_recognition/detections']).value
        self.photo_topic = param(
            'photo_topic', '/hailo_face_recognition/image_bbx/compressed').value
        if param('normalized_boxes', True).value:
            self.width = self.height = 1.0
        else:
            self.width = param('image_width', 0).value
            self.height = param('image_height', 0).value
        # Camera Module 3 (standard lens) datasheet; to be measured
        self.hfov = param('hfov_deg', 66.0).value
        self.vfov = param('vfov_deg', 41.0).value
        self.min_score = param('min_score', 0.4).value
        # Fraction of the offset corrected per detection, and the quiet zone in
        # the middle of the view where the head does not move (deg off centre)
        self.gain = param('gain', 0.8).value
        self.zone_pan = param('zone_pan_deg', 8.0).value
        self.zone_tilt = param('zone_tilt_deg', 6.0).value
        # 0 = the head's default (60 deg/s): the owner preferred it to 90 and 200
        # watching an emulated face (2026-11-26). 200 was set when the head moved
        # at 400 deg/s and full speed looked jumpy
        self.speed = math.radians(param('speed_deg_s', 0.0).value)
        # Frame taken this long before its detections' stamp (or their arrival
        # when the stamp is missing or off), in s
        self.frame_latency = param('frame_latency_s', 0.15).value
        self.lost_after = param('lost_after_s', 3.0).value
        # Someone in view without a usable face or body box: tilt up a step this often (s)
        self.search_period = int(param('search_period_s', 0.8).value * 1e9)
        self.last_search = 0
        self.pan_joint = param('pan_joint', 'head_pan_joint').value
        self.tilt_joint = param('tilt_joint', 'head_tilt_joint').value

        self.lock = threading.Lock()
        self.target = ''
        self.started = None
        self.last_seen = None
        self.previous = None       # centre of the detection followed last
        self.label = ''
        self.sent_status = None
        self.joints = deque(maxlen=200)   # (ns, pan deg, tilt deg)

        self.target_pub = self.create_publisher(HeadTarget, '/head/target', 10)
        self.status_pub = self.create_publisher(TrackerStatus, '~/status', LATCHED)
        self.create_subscription(JointState, '/joint_states', self.on_joints, 50)
        for topic in topics:
            self.create_subscription(Detection2DArray, topic, self.on_detections,
                                     qos_profile_sensor_data)
        self.image_sub = None
        if not (self.width and self.height):
            self.image_sub = self.create_subscription(
                CompressedImage, self.photo_topic, self.on_image, qos_profile_sensor_data)
        self.create_service(Track, '~/start', self.on_start)
        self.create_service(Trigger, '~/stop', self.on_stop)
        self.create_timer(1.0, lambda: self.publish_status(force=True))
        self.create_timer(0.2, self.publish_status)
        self.publish_status(force=True)

    def now_ns(self):
        return self.get_clock().now().nanoseconds

    # Inputs

    def on_image(self, msg):
        try:
            import cv2
            import numpy as np
        except ImportError:
            self.get_logger().error('no OpenCV: set image_width and image_height')
            self.destroy_subscription(self.image_sub)
            self.image_sub = None
            return
        image = cv2.imdecode(np.frombuffer(bytes(msg.data), dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            return
        self.height, self.width = image.shape[:2]
        self.get_logger().info(f'image size from {self.photo_topic}: '
                               f'{self.width} x {self.height}')
        self.destroy_subscription(self.image_sub)
        self.image_sub = None

    def on_joints(self, msg):
        try:
            pan = msg.position[msg.name.index(self.pan_joint)]
            tilt = msg.position[msg.name.index(self.tilt_joint)]
        except (ValueError, IndexError):
            return
        stamp = Time.from_msg(msg.header.stamp).nanoseconds
        self.joints.append((stamp, math.degrees(pan), math.degrees(tilt)))

    def head_at(self, stamp_ns):
        """Return the head's pose (deg) at a time, from /joint_states; None before any."""
        if not self.joints:
            return None
        pan, tilt = self.joints[0][1:]
        for t, p, q in self.joints:
            if t > stamp_ns:
                break
            pan, tilt = p, q
        return pan, tilt

    def matching(self, msg, wanted):
        """[(x, y, w, h, label)] of the detections whose label is wanted; x, y the aim point."""
        found = []
        for detection in msg.detections:
            for result in detection.results:
                label = result.hypothesis.class_id
                if result.hypothesis.score >= self.min_score and wanted(label):
                    box = detection.bbox
                    if not all(math.isfinite(v) for v in (
                            box.center.position.x, box.center.position.y,
                            box.size_x, box.size_y)):
                        break           # the Hailo sends NaN boxes for near, big things
                    x, y = aim_point(label, box.center.position.x, box.center.position.y,
                                     box.size_x, box.size_y)
                    found.append((x, y, box.size_x, box.size_y, label))
                    break
        return found

    def search(self, target):
        """Someone is in view, but no face or usable body box: tilt up a step."""
        now = self.now_ns()
        with self.lock:
            if self.target != target or now - self.last_search < self.search_period:
                return
            self.last_search = now
            self.last_seen = now        # someone is there: not lost
            self.label = FALLBACK[target]
        if not self.joints:
            return
        pan, tilt = self.joints[-1][1:]
        tilt = search_up(tilt)
        if tilt is None:
            return
        self.get_logger().info(f'no face in view: looking up ({tilt:.0f} deg)',
                               throttle_duration_sec=5.0)
        self.target_pub.publish(HeadTarget(pan=math.radians(pan), tilt=math.radians(tilt),
                                           speed=self.speed))

    def on_detections(self, msg):
        with self.lock:
            target = self.target
        if not target or not (self.width and self.height):
            return
        found = self.matching(msg, lambda label: matches(target, label))
        if not found and target in FALLBACK:
            found = self.matching(msg, lambda label: has_label(FALLBACK[target], label))
        if not found:
            if target in FALLBACK and any(
                    has_label(FALLBACK[target], r.hypothesis.class_id)
                    and r.hypothesis.score >= self.min_score
                    for d in msg.detections for r in d.results):
                self.search(target)
            return
        now = self.now_ns()
        with self.lock:
            if self.target != target:
                return
            if self.last_seen is not None and now - self.last_seen > 1e9:
                self.previous = None   # out of view for a while: pick afresh
            chosen = pick([c[:4] for c in found], self.previous)
            self.label = next(c[4] for c in found if c[:4] == chosen)
            self.previous = chosen[:2]
            self.last_seen = now
        stamp = Time.from_msg(msg.header.stamp).nanoseconds
        if stamp == 0 or abs(now - stamp) > 1e9:
            stamp = now
        stamp -= int(self.frame_latency * 1e9)
        head = self.head_at(stamp)
        if head is None:
            self.get_logger().warn('target seen, but no /joint_states from the head: '
                                   'not aiming', throttle_duration_sec=10.0)
            return
        off = pixel_to_angles(chosen[0], chosen[1], self.width, self.height,
                              self.hfov, self.vfov)
        aim = head_target(head[0], head[1], off[0], off[1], self.gain, self.zone_pan,
                          self.zone_tilt)
        if aim is not None:
            # An axis inside the zone keeps where the head is now
            now_pan, now_tilt = self.joints[-1][1:]
            pan = now_pan if aim[0] is None else aim[0]
            tilt = now_tilt if aim[1] is None else aim[1]
            self.target_pub.publish(HeadTarget(pan=math.radians(pan), tilt=math.radians(tilt),
                                               speed=self.speed))
        self.publish_status()

    # Services

    def on_start(self, request, response):
        target = normalize(request.target)
        if not target:
            response.success = False
            response.message = 'tell me what to follow'
            return response
        with self.lock:
            self.target = target
            self.started = self.now_ns()
            self.last_seen = None
            self.previous = None
            self.label = ''
        self.get_logger().info(f'tracking {target}')
        self.publish_status(force=True)
        response.success = True
        response.message = f'I will turn my head after the {target} whenever I can see it'
        if not (self.width and self.height):
            response.message += " (my camera's pictures have not arrived yet)"
        return response

    def on_stop(self, request, response):
        with self.lock:
            was = self.target
            self.target = ''
            self.label = ''
        if was:
            self.get_logger().info(f'stopped tracking {was}')
            # Back to the centre (look, which stops tracking first, then sends
            # its own move, which replaces this one)
            self.target_pub.publish(HeadTarget(pan=0.0, tilt=0.0, speed=self.speed))
        self.publish_status(force=True)
        response.success = True
        response.message = f'stopped following {was}' if was else 'I was not following anything'
        return response

    # Status

    def status(self):
        now = self.now_ns()
        with self.lock:
            msg = TrackerStatus(target=self.target, active=bool(self.target), label=self.label)
            if self.target:
                since = self.last_seen if self.last_seen is not None else self.started
                msg.seen = self.last_seen is not None and now - self.last_seen < 0.5e9
                msg.lost = now - since > self.lost_after * 1e9
        return msg

    def publish_status(self, force=False):
        msg = self.status()
        # Just lost: back to the centre
        if msg.lost and not (self.sent_status and self.sent_status[2]):
            self.target_pub.publish(HeadTarget(pan=0.0, tilt=0.0, speed=self.speed))
        key = (msg.target, msg.seen, msg.lost, msg.label)
        if force or key != self.sent_status:
            self.sent_status = key
            self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TrackerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
