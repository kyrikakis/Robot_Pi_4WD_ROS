"""Remembers where the robot is, so localization starts from there.

AMCL forgets its pose whenever it starts (power-on, a restart of
rainbow_localization). This node saves AMCL's pose (/amcl_pose) to
<data_dir>/last_pose/<map_name>.yaml while AMCL is sure of it (position std
under save_pos_std_m, yaw std under save_yaw_std_rad, the brain's "sure") and
the robot is not lifted, at most once every save_period_s and only when it has
moved. A pick-up (/lifted true) deletes the file: the robot can be put down
anywhere (the brain then starts global localization, and the pose is saved
again once AMCL is sure).

At start, once AMCL is active and has the map (start_delay_s after it is
active), it sends the saved pose as a pose estimate (/initialpose, std pose_std_m
and yaw_std_rad, so AMCL can correct a small error with the first scans; the
estimate also turns the lidar on for 30 s, lidar_power). Without a saved pose
for this map it asks AMCL for global localization instead (guesses over the
whole map). A robot carried somewhere while it was off starts at its old pose:
give a pose estimate then.

Run it only with AMCL (localization.launch.py), never with slam_toolbox.

Parameters:
  data_dir          '/data/rainbow'
  map_name          ''    required (localization.launch.py takes it from localization.yaml)
  save_period_s     2.0
  save_pos_std_m    0.25
  save_yaw_std_rad  0.2
  pose_std_m        0.25
  yaw_std_rad       0.26
  start_delay_s     3.0
"""

import math
import os
import time

from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import Empty
import yaml

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
MOVED_M = 0.02
TURNED_RAD = 0.03


def yaw_of(q):
    """Yaw (rad) of a quaternion with x, y, z, w."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def is_sure(covariance, pos_std_max, yaw_std_max):
    """Whether AMCL's pose covariance is tight enough to keep the pose."""
    pos_std = math.sqrt(max(covariance[0], covariance[7], 0.0))
    yaw_std = math.sqrt(max(covariance[35], 0.0))
    return pos_std < pos_std_max and yaw_std < yaw_std_max


def has_moved(old, new):
    """Whether pose new (x, y, yaw) differs from old enough to save it again."""
    if old is None:
        return True
    turn = abs(math.atan2(math.sin(new[2] - old[2]), math.cos(new[2] - old[2])))
    return math.hypot(new[0] - old[0], new[1] - old[1]) > MOVED_M or turn > TURNED_RAD


def save_pose(path, map_name, pose):
    """Write the pose atomically (a power cut must not leave half a file)."""
    os.makedirs(os.path.dirname(path), exist_ok=True)
    data = {'map': map_name, 'x': round(pose[0], 3), 'y': round(pose[1], 3),
            'yaw': round(pose[2], 4), 'saved': time.strftime('%Y-%m-%d %H:%M:%S')}
    tmp = path + '.tmp'
    with open(tmp, 'w') as f:
        yaml.safe_dump(data, f)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)


def load_pose(path, map_name):
    """Return the saved (x, y, yaw) for this map, or None."""
    try:
        with open(path) as f:
            data = yaml.safe_load(f)
        if data.get('map') != map_name:
            return None
        return float(data['x']), float(data['y']), float(data['yaw'])
    except (OSError, yaml.YAMLError, AttributeError, KeyError, TypeError, ValueError):
        return None


class PoseKeeper(Node):
    """Saves AMCL's pose and gives it back to AMCL when it starts."""

    def __init__(self):
        super().__init__('pose_keeper')
        param = self.declare_parameter
        data_dir = param('data_dir', '/data/rainbow').value
        self.map_name = param('map_name', '').value
        if not self.map_name:
            raise RuntimeError('map_name is not set')
        self.save_period = param('save_period_s', 2.0).value
        self.save_pos_std = param('save_pos_std_m', 0.25).value
        self.save_yaw_std = param('save_yaw_std_rad', 0.2).value
        self.pose_std = param('pose_std_m', 0.25).value
        self.yaw_std = param('yaw_std_rad', 0.26).value
        self.start_delay = param('start_delay_s', 3.0).value
        self.path = os.path.join(data_dir, 'last_pose', self.map_name + '.yaml')

        self.lifted = False
        self.saved = load_pose(self.path, self.map_name)
        self.saved_at = 0.0
        self.started = False       # pose estimate or global localization sent
        self.active_at = None
        self.state_future = None

        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.global_client = self.create_client(Empty, '/reinitialize_global_localization')
        self.state_client = self.create_client(GetState, '/amcl/get_state')
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, LATCHED)
        self.create_subscription(Bool, '/lifted', self.on_lifted, LATCHED)
        self.start_timer = self.create_timer(1.0, self.try_start)
        self.get_logger().info(
            f'saved pose for map {self.map_name}: ' + (
                f'x {self.saved[0]:.2f} y {self.saved[1]:.2f} '
                f'yaw {math.degrees(self.saved[2]):.0f} deg' if self.saved else 'none'))

    def on_lifted(self, msg):
        if msg.data and not self.lifted:
            self.forget('picked up')
        self.lifted = msg.data

    def forget(self, why):
        self.saved = None
        try:
            os.remove(self.path)
            self.get_logger().info(f'{why}: forgot the saved pose')
        except FileNotFoundError:
            pass
        except OSError as e:
            self.get_logger().warning(f'cannot remove {self.path}: {e}')

    def on_amcl(self, msg):
        if not self.started or self.lifted:
            return
        if not is_sure(msg.pose.covariance, self.save_pos_std, self.save_yaw_std):
            return
        now = time.monotonic()
        if now - self.saved_at < self.save_period:
            return
        p = msg.pose.pose
        pose = (p.position.x, p.position.y, yaw_of(p.orientation))
        if not has_moved(self.saved, pose):
            return
        try:
            save_pose(self.path, self.map_name, pose)
        except OSError as e:
            self.get_logger().warning(f'cannot save the pose to {self.path}: {e}')
            return
        self.saved, self.saved_at = pose, now

    def try_start(self):
        """Once AMCL is active (and start_delay_s later), restore the pose or localize."""
        if self.active_at is None:
            if self.state_future is not None:
                if not self.state_future.done():
                    return
                result = self.state_future.result()
                self.state_future = None
                if result is not None and result.current_state.id == State.PRIMARY_STATE_ACTIVE:
                    self.active_at = time.monotonic()
                return
            if self.state_client.service_is_ready():
                self.state_future = self.state_client.call_async(GetState.Request())
            return
        if time.monotonic() - self.active_at < self.start_delay:
            return
        if self.saved is not None and not self.lifted:
            self.send_pose(self.saved)
        elif self.global_client.service_is_ready():
            self.global_client.call_async(Empty.Request())
            self.get_logger().info('no saved pose: AMCL starts global localization')
        else:
            return
        self.started = True
        self.start_timer.cancel()

    def send_pose(self, pose):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = pose[0]
        msg.pose.pose.position.y = pose[1]
        msg.pose.pose.orientation.z = math.sin(pose[2] / 2.0)
        msg.pose.pose.orientation.w = math.cos(pose[2] / 2.0)
        msg.pose.covariance[0] = msg.pose.covariance[7] = self.pose_std ** 2
        msg.pose.covariance[35] = self.yaw_std ** 2
        self.initialpose_pub.publish(msg)
        self.get_logger().info(
            f'pose estimate from the saved pose: x {pose[0]:.2f} y {pose[1]:.2f} '
            f'yaw {math.degrees(pose[2]):.0f} deg')


def main():
    rclpy.init()
    node = PoseKeeper()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
