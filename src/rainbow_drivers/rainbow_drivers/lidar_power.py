"""Keeps the lidar in standby unless the robot needs it (it is frugal on purpose).

The lidar is on:
  - while rainbow_slam runs (mapping needs every scan);
  - while the robot is commanded to move (non-zero /cmd_vel, what reaches the
    ESP), and for rest_delay_s after the last motion;
  - while /lidar/wanted (std_msgs/Bool) is true and fresh (wanted_timeout_s):
    the brain asks for it before a Nav2 goal and while localizing, and repeats
    it at least once a second.
Never while /lifted (std_msgs/Bool) is true: a robot in the hand has no use for
it (its location is lost anyway), unless it is mapping. Subscribers to /scan do
not count: Foxglove or a status check never turn it on.

Sets it through the ESP firmware's lidar/enable service (std_srvs/SetBool:
MS200 standby / run, answered once the data stream has stopped or started). The
firmware boots with the lidar in standby; the wanted state is sent again
whenever the ESP (re)connects, seen as a new message on the latched
/esp_firmware. A failed call is retried every retry_s.

Parameters:
  rest_delay_s       30.0
  wanted_timeout_s   3.0
  mapping_program    'rainbow_slam'
  programs_period_s  5.0   how often supervisor is asked
  retry_s            30.0
"""

import subprocess
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool

CALL_TIMEOUT_S = 15.0  # the firmware answers within ~5 s (10 s while changing speed)
MOTION_EPS = 1e-3


def running_programs(status_output):
    """Names of the RUNNING/STARTING programs in `supervisorctl status` output."""
    names = []
    for line in status_output.splitlines():
        parts = line.split()
        if len(parts) > 1 and parts[1] in ('RUNNING', 'STARTING'):
            names.append(parts[0])
    return names


def is_motion(twist):
    """Whether a command moves the robot (skid-steer: forward speed or turn)."""
    return abs(twist.linear.x) > MOTION_EPS or abs(twist.angular.z) > MOTION_EPS


def lidar_reasons(now, mapping, last_motion, rest_delay, wanted_at, wanted_timeout, lifted):
    """Why the lidar should be on now; empty when it should be in standby."""
    if mapping:
        return ['mapping']
    if lifted:
        return []
    reasons = []
    if last_motion is not None and now - last_motion < rest_delay:
        reasons.append('moving')
    if wanted_at is not None and now - wanted_at < wanted_timeout:
        reasons.append('asked')
    return reasons


class LidarPower(Node):
    """Decides whether the lidar runs and tells the ESP."""

    def __init__(self):
        super().__init__('lidar_power')
        self.rest_delay = self.declare_parameter('rest_delay_s', 30.0).value
        self.wanted_timeout = self.declare_parameter('wanted_timeout_s', 3.0).value
        self.mapping_program = self.declare_parameter('mapping_program', 'rainbow_slam').value
        self.programs_period = self.declare_parameter('programs_period_s', 5.0).value
        self.retry = self.declare_parameter('retry_s', 30.0).value

        self.client = self.create_client(SetBool, 'lidar/enable')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(String, 'esp_firmware', self.on_esp_session, latched)
        self.create_subscription(Twist, 'cmd_vel', self.on_cmd_vel, 10)
        self.create_subscription(Bool, 'lidar/wanted', self.on_wanted, 10)
        self.create_subscription(Bool, 'lifted', self.on_lifted, latched)

        self.last_motion = None
        self.wanted_at = None
        self.lifted = False
        self.mapping = False
        self.programs_checked = -1e9
        self.reasons = None
        self.sent = None             # state the ESP confirmed (None: unknown)
        self.in_flight = None        # (state, future, started)
        self.retry_at = 0.0
        self.create_timer(0.5, self.tick)

    def on_esp_session(self, msg):
        """Resend the wanted state: a new ESP session boots the lidar in standby."""
        self.get_logger().info(f'ESP connected ({msg.data}); lidar state will be sent again')
        self.sent = None
        self.retry_at = 0.0

    def on_cmd_vel(self, msg):
        """Remember the last motion command."""
        if is_motion(msg):
            self.last_motion = time.monotonic()

    def on_wanted(self, msg):
        """Remember the last request for the lidar."""
        self.wanted_at = time.monotonic() if msg.data else None

    def on_lifted(self, msg):
        """Follow the lifted state."""
        self.lifted = msg.data

    def check_mapping(self, now):
        """Ask supervisor whether the mapping program runs (every programs_period_s)."""
        if now - self.programs_checked < self.programs_period:
            return
        self.programs_checked = now
        try:
            out = subprocess.run(['supervisorctl', 'status', self.mapping_program],
                                 capture_output=True, text=True, timeout=10).stdout
        except (OSError, subprocess.SubprocessError) as e:
            self.get_logger().warn(f'supervisorctl status: {e}', throttle_duration_sec=60.0)
            out = ''
        self.mapping = self.mapping_program in running_programs(out)

    def tick(self):
        """Work out the wanted state and send it when it differs from the ESP's."""
        now = time.monotonic()
        self.check_mapping(now)
        reasons = lidar_reasons(now, self.mapping, self.last_motion, self.rest_delay,
                                self.wanted_at, self.wanted_timeout, self.lifted)
        if reasons != self.reasons:
            why = ', '.join(reasons) if reasons else ('lifted' if self.lifted else 'not needed')
            self.get_logger().info(f'lidar {"wanted" if reasons else "standby"}: {why}')
            self.reasons = reasons
        want = bool(reasons)

        if self.in_flight is not None:
            self.check_call(now)
            return
        if want != self.sent and now >= self.retry_at and self.client.service_is_ready():
            self.in_flight = (want, self.client.call_async(SetBool.Request(data=want)), now)

    def check_call(self, now):
        """Handle the answer of the lidar/enable call in flight."""
        state, future, started = self.in_flight
        if not future.done():
            if now - started > CALL_TIMEOUT_S:
                self.get_logger().warn('lidar/enable: no answer; retrying later')
                future.cancel()
                self.in_flight = None
                self.retry_at = now + self.retry
            return
        self.in_flight = None
        result = future.result()
        if result is not None and result.success:
            self.sent = state
            self.get_logger().info(f'lidar {"on" if state else "standby"}: {result.message}')
        else:
            why = result.message if result is not None else 'call failed'
            self.get_logger().warn(f'lidar {"on" if state else "standby"} failed: {why}; '
                                   f'retrying in {self.retry:.0f} s')
            self.retry_at = now + self.retry


def main():
    rclpy.init()
    node = LidarPower()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
