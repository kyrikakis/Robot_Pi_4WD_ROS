"""Arducam ToF depth camera.

Publishes:
  point_cloud           sensor_msgs/PointCloud2  in tof_optical_frame
                                                 (z forward, x right, y down)
  tof/depth/image_raw   sensor_msgs/Image        32FC1, metres, 0 = no reading

The sensor runs at a fixed ~25-30 fps; frames are published at frame_rate,
and each output only when something subscribes to it. Pixels below the
confidence threshold, outside [min_range, max_range], or next to a depth jump
larger than edge_threshold (flying pixels) are dropped.
"""

import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header


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
        self.hfov_deg = self.declare_parameter('horizontal_fov_deg', 64.3).value
        self.vfov_deg = self.declare_parameter('vertical_fov_deg', 50.4).value
        # Flying pixels: at depth edges a ToF pixel mixes the near and far surface and
        # reports a point in mid-air. Drop pixels whose depth differs from a neighbour
        # by more than this (m); 0 disables the filter.
        self.edge_threshold = self.declare_parameter('edge_threshold', 0.05).value

        qos = qos_profile_sensor_data
        self.cloud_pub = self.create_publisher(PointCloud2, 'point_cloud', qos)
        self.depth_pub = self.create_publisher(Image, 'tof/depth/image_raw', qos)

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
            try:
                now = time.monotonic()
                if now < next_time:
                    continue
                # Keep to the schedule, but do not try to catch up after a gap
                next_time = max(next_time + period, now)
                want_cloud = self.cloud_pub.get_subscription_count() > 0
                want_image = self.depth_pub.get_subscription_count() > 0
                if isinstance(frame, DepthData) and (want_cloud or want_image):
                    self.publish(frame, want_cloud, want_image)
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

    def publish(self, frame, want_cloud, want_image):
        """Publish one frame. Each output is only built when something subscribes."""
        # depth_data is float32 millimetres; convert to metres in one new array
        depth = np.multiply(frame.depth_data, 0.001, dtype=np.float32)
        valid = frame.confidence_data >= self.confidence
        valid &= depth >= self.min_range
        valid &= depth <= self.max_range
        if self.edge_threshold > 0:
            valid &= ~self.flying_pixels(depth, valid)
        header = Header(frame_id=self.frame_id, stamp=self.get_clock().now().to_msg())

        if want_image:
            depth[~valid] = 0.0
            image = Image(header=header, height=self.height, width=self.width,
                          encoding='32FC1', is_bigendian=0, step=self.width * 4)
            image.data = depth.tobytes()
            self.depth_pub.publish(image)

        if want_cloud:
            z = depth[valid]
            points = np.empty((len(z), 3), dtype=np.float32)
            np.multiply(self.rays_x[valid], z, out=points[:, 0])
            np.multiply(self.rays_y[valid], z, out=points[:, 1])
            points[:, 2] = z
            cloud = PointCloud2(header=header, height=1, width=len(points),
                                fields=self.cloud_fields, is_bigendian=False, point_step=12,
                                row_step=12 * len(points), is_dense=True)
            cloud.data = points.tobytes()
            self.cloud_pub.publish(cloud)

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
