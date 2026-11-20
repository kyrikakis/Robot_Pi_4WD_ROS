"""Arducam ToF depth camera.

Publishes:
  point_cloud           sensor_msgs/PointCloud2  in tof_optical_frame
                                                 (z forward, x right, y down)
  tof/depth/image_raw   sensor_msgs/Image        32FC1, metres, 0 = no reading
  tof/obstacles         sensor_msgs/PointCloud2  in tof_optical_frame: only the points
                                                 that are obstacles (Nav2 marks these)

Obstacles are points between a floor-height threshold and obstacle_max_height
above base_footprint: obstacle_min_height_near up to near_distance from the
robot, where the calibrated floor is cleanest, obstacle_min_height_far beyond.

The floor test is levelled with the IMU (imu_topic): when the body pitches or
rolls (braking, accelerating, rocking after a turn) the floor ahead would
otherwise look raised (1 deg is 1.7 cm at 1 m) and be marked as obstacles. The
correction is the tilt relative to the robot's average attitude over
tilt_time_constant, since the ToF calibration was made with the robot at rest.

Isolated obstacle pixels are dropped: one is kept only if at least
obstacle_min_fraction of its obstacle_window_px square neighbourhood in the
image are obstacle pixels too. Specks on the floor (stains, reflections) cover
a pixel or two; a real obstacle 2 cm wide covers ~6 pixels at 1 m.

Subscribes:
  bno055/imu            sensor_msgs/Imu          body orientation (imu_topic)

The sensor runs at a fixed ~25-30 fps; frames are published at frame_rate,
and each output only when something subscribes to it. Pixels below the
confidence threshold, outside [min_range, max_range], or next to a depth jump
larger than edge_threshold (flying pixels) are dropped.
"""

import math
import threading
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu, PointCloud2, PointField
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener


class TofNode(Node):

    def __init__(self):
        super().__init__('tof')
        self.frame_id = self.declare_parameter('frame_id', 'tof_optical_frame').value
        self.camera_index = self.declare_parameter('camera_index', 8).value
        self.frame_rate = self.declare_parameter('frame_rate', 12).value
        self.range_mm = self.declare_parameter('range_mm', 4000).value
        self.confidence = self.declare_parameter('confidence_threshold', 30).value
        self.min_range = self.declare_parameter('min_range', 0.05).value
        self.max_range = self.declare_parameter('max_range', 4.0).value
        # Frames carry no timestamp: they are stamped on arrival minus this
        # estimate of the sensor's exposure and depth computation (not measured).
        # A late stamp puts obstacles in the wrong place while turning
        self.latency = self.declare_parameter('latency', 0.04).value
        self.hfov_deg = self.declare_parameter('horizontal_fov_deg', 64.3).value
        self.vfov_deg = self.declare_parameter('vertical_fov_deg', 50.4).value
        # Flying pixels: at depth edges a ToF pixel mixes the near and far surface and
        # reports a point in mid-air. Drop pixels whose depth differs from a neighbour
        # by more than this (m); 0 disables the filter.
        self.edge_threshold = self.declare_parameter('edge_threshold', 0.05).value
        # Drop this many pixels at the image edges, where the ToF is least reliable.
        # Keep the published view inside Nav2's clearing frustum, or edge noise is
        # marked but never cleared.
        self.crop_x = self.declare_parameter('crop_x_px', 14).value
        self.crop_y = self.declare_parameter('crop_y_px', 10).value
        # Floor-aware obstacle cloud (heights above base_footprint, m)
        self.base_frame = self.declare_parameter('base_frame', 'base_footprint').value
        self.near_distance = self.declare_parameter('near_distance', 0.7).value
        self.min_h_near = self.declare_parameter('obstacle_min_height_near', 0.02).value
        self.min_h_far = self.declare_parameter('obstacle_min_height_far', 0.035).value
        self.max_h = self.declare_parameter('obstacle_max_height', 0.16).value
        # Speckle filter: majority of the neighbourhood must be obstacle pixels
        self.obst_window = self.declare_parameter('obstacle_window_px', 5).value
        self.obst_fraction = self.declare_parameter('obstacle_min_fraction', 0.5).value
        # Level the floor test with the IMU; empty imu_topic disables it
        imu_topic = self.declare_parameter('imu_topic', '/bno055/imu').value
        self.tilt_tau = self.declare_parameter('tilt_time_constant', 5.0).value
        self.imu_timeout = self.declare_parameter('imu_timeout', 0.5).value
        self.rest_tilt = None       # (roll, pitch) averaged over tilt_tau
        self.tilt = None            # (roll, pitch, monotonic time) of the latest sample
        self.last_imu_time = None
        if imu_topic:
            self.create_subscription(Imu, imu_topic, self.on_imu, qos_profile_sensor_data)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        qos = qos_profile_sensor_data
        self.cloud_pub = self.create_publisher(PointCloud2, 'point_cloud', qos)
        self.depth_pub = self.create_publisher(Image, 'tof/depth/image_raw', qos)
        self.obstacle_pub = self.create_publisher(PointCloud2, 'tof/obstacles', qos)

        self.camera = self.open_camera()
        info = self.camera.getCameraInfo()
        self.width, self.height = info.width, info.height
        self.rays_x, self.rays_y = self.make_rays(self.width, self.height)
        self.cloud_fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32,
                                        count=1) for i, n in enumerate('xyz')]

        self.running = True
        self.thread = threading.Thread(target=self.capture_loop, daemon=True)
        self.thread.start()
        self.get_logger().info(f'ToF started: {self.width}x{self.height} at {self.frame_rate} fps')

    def open_camera(self):
        from ArducamDepthCamera import (ArducamCamera, Connection, Control, FrameType,
                                        TofErrorCode)

        camera = ArducamCamera()
        ret = camera.open(Connection.CSI, self.camera_index)
        if ret != TofErrorCode.ArducamSuccess:
            raise RuntimeError(f'failed to open ToF camera (error {ret})')
        ret = camera.start(FrameType.DEPTH)
        if ret != TofErrorCode.ArducamSuccess:
            camera.close()
            raise RuntimeError(f'failed to start ToF camera (error {ret})')
        # Range in mm (2000 or 4000); only accepted after start(). The sensor
        # has no frame-rate control (it runs at ~25-30 fps), so frame_rate is
        # applied in capture_loop by skipping frames.
        ret = camera.setControl(Control.RANGE, self.range_mm)
        if ret != TofErrorCode.ArducamSuccess:
            self.get_logger().warning(f'ToF rejected range {self.range_mm} mm (error {ret})')
        return camera

    def make_rays(self, width, height):
        """Per-pixel x/z and y/z ratios for a pinhole model from the field of view."""
        fx = width / (2 * np.tan(np.radians(self.hfov_deg) / 2))
        fy = height / (2 * np.tan(np.radians(self.vfov_deg) / 2))
        u, v = np.meshgrid(np.arange(width), np.arange(height))
        return ((u - width / 2) / fx).astype(np.float32), ((v - height / 2) / fy).astype(np.float32)

    def capture_loop(self):
        from ArducamDepthCamera import DepthData

        # The sensor may run faster than frame_rate; skip frames to hold it
        period = 1.0 / self.frame_rate
        next_time = 0.0
        while self.running and rclpy.ok():
            frame = self.camera.requestFrame(200)
            if frame is None:
                continue
            stamp = self.get_clock().now() - Duration(seconds=self.latency)
            try:
                now = time.monotonic()
                if now < next_time:
                    continue
                # Keep to the schedule, but do not try to catch up after a gap
                next_time = max(next_time + period, now)
                want_cloud = self.cloud_pub.get_subscription_count() > 0
                want_image = self.depth_pub.get_subscription_count() > 0
                want_obst = self.obstacle_pub.get_subscription_count() > 0
                if isinstance(frame, DepthData) and (want_cloud or want_image or want_obst):
                    self.publish(frame, stamp, want_cloud, want_image, want_obst)
            finally:
                self.camera.releaseFrame(frame)

    def flying_pixels(self, depth, valid):
        """Pixels next to a depth jump larger than edge_threshold (8-neighbourhood)."""
        d = np.where(valid, depth, np.nan)
        h, w = d.shape
        p = np.pad(d, 1, constant_values=np.nan)
        jump = np.zeros((h, w), dtype=bool)
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dy or dx:
                    nb = p[1 + dy:h + 1 + dy, 1 + dx:w + 1 + dx]
                    with np.errstate(invalid='ignore'):
                        jump |= np.abs(nb - d) > self.edge_threshold
        return jump

    def on_imu(self, msg):
        q = msg.orientation
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
        now = time.monotonic()
        if self.rest_tilt is None:
            self.rest_tilt = (roll, pitch)
        else:
            a = min(1.0, (now - self.last_imu_time) / self.tilt_tau)
            r0, p0 = self.rest_tilt
            self.rest_tilt = (r0 + a * (roll - r0), p0 + a * (pitch - p0))
        self.last_imu_time = now
        self.tilt = (roll, pitch, now)

    @staticmethod
    def tilt_matrix(roll, pitch):
        """Body-to-level rotation for roll and pitch (yaw left out)."""
        cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
        return np.array([[cp, sp * sr, sp * cr],
                         [0.0, cr, -sr],
                         [-sp, cp * sr, cp * cr]], dtype=np.float32)

    def level_correction(self):
        """Rotation from the tilted body to its rest attitude, or None if no fresh IMU."""
        tilt, rest = self.tilt, self.rest_tilt
        if tilt is None or rest is None or time.monotonic() - tilt[2] > self.imu_timeout:
            return None
        return self.tilt_matrix(*rest).T @ self.tilt_matrix(tilt[0], tilt[1])

    def despeckle(self, mask_img):
        """Keep obstacle pixels whose neighbourhood is mostly obstacle pixels."""
        k = self.obst_window
        if k <= 1:
            return mask_img
        r = k // 2
        p = np.pad(mask_img.astype(np.int32), r)
        integral = np.pad(p.cumsum(0).cumsum(1), ((1, 0), (1, 0)))
        h, w = mask_img.shape
        count = (integral[k:k + h, k:k + w] - integral[:h, k:k + w]
                 - integral[k:k + h, :w] + integral[:h, :w])
        return mask_img & (count >= self.obst_fraction * k * k)

    def obstacle_mask(self, points):
        """Which points are obstacles: above the floor threshold, below max height."""
        try:
            t = self.tf_buffer.lookup_transform(self.base_frame, self.frame_id, rclpy.time.Time())
        except TransformException:
            return None
        q, tr = t.transform.rotation, t.transform.translation
        x, y, z, w = q.x, q.y, q.z, q.w
        rot = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]],
                       dtype=np.float32)
        base = points @ rot.T + np.array([tr.x, tr.y, tr.z], dtype=np.float32)
        level = self.level_correction()
        if level is not None:
            base = base @ level.T
        height = base[:, 2]
        distance = np.hypot(base[:, 0], base[:, 1])
        min_h = np.where(distance <= self.near_distance, self.min_h_near, self.min_h_far)
        return (height >= min_h) & (height <= self.max_h)

    def make_cloud(self, header, points):
        cloud = PointCloud2(header=header, height=1, width=len(points),
                            fields=self.cloud_fields, is_bigendian=False, point_step=12,
                            row_step=12 * len(points), is_dense=True)
        cloud.data = points.tobytes()
        return cloud

    def publish(self, frame, stamp, want_cloud, want_image, want_obst=False):
        """Publish one frame. Each output is only built when something subscribes."""
        # depth_data is float32 millimetres; convert to metres in one new array
        depth = np.multiply(frame.depth_data, 0.001, dtype=np.float32)
        valid = frame.confidence_data >= self.confidence
        valid &= depth >= self.min_range
        valid &= depth <= self.max_range
        if self.edge_threshold > 0:
            valid &= ~self.flying_pixels(depth, valid)
        if self.crop_x:
            valid[:, :self.crop_x] = False
            valid[:, -self.crop_x:] = False
        if self.crop_y:
            valid[:self.crop_y, :] = False
            valid[-self.crop_y:, :] = False
        header = Header(frame_id=self.frame_id, stamp=stamp.to_msg())

        if want_image:
            depth[~valid] = 0.0
            image = Image(header=header, height=self.height, width=self.width,
                          encoding='32FC1', is_bigendian=0, step=self.width * 4)
            image.data = depth.tobytes()
            self.depth_pub.publish(image)

        if want_cloud or want_obst:
            z = depth[valid]
            points = np.empty((len(z), 3), dtype=np.float32)
            np.multiply(self.rays_x[valid], z, out=points[:, 0])
            np.multiply(self.rays_y[valid], z, out=points[:, 1])
            points[:, 2] = z
            if want_cloud:
                self.cloud_pub.publish(self.make_cloud(header, points))
            if want_obst:
                mask = self.obstacle_mask(points)
                if mask is not None:
                    mask_img = np.zeros(valid.shape, dtype=bool)
                    mask_img[valid] = mask
                    mask = self.despeckle(mask_img)[valid]
                    self.obstacle_pub.publish(self.make_cloud(header, points[mask]))

    def stop(self):
        self.running = False
        self.thread.join(timeout=1.0)
        self.camera.stop()
        self.camera.close()


def main(args=None):
    rclpy.init(args=args)
    node = TofNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()
