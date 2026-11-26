"""Turns the lidar on while something needs it and puts it in standby otherwise.

The lidar is wanted while a node other than those in ignore_nodes subscribes to
/scan (slam_toolbox, AMCL, the Nav2 costmaps and collision monitor, a Foxglove
panel, ros2 topic echo), or while one of the supervisor programs in `programs`
runs (so a Nav2 that has become undiscoverable still keeps it on). The brain
only watches /scan's age for its status, so it does not count. The lidar goes to
standby once nothing has wanted it for off_delay_s.

Sets it through the ESP firmware's lidar/enable service (std_srvs/SetBool:
MS200 standby / run, answered once the data stream has stopped or started). The
firmware boots with the lidar in standby; the wanted state is sent again
whenever the ESP (re)connects, seen as a new message on the latched
/esp_firmware. A failed call is retried every retry_s.

Parameters:
  ignore_nodes  ['brain']
  programs      ['rainbow_slam', 'rainbow_localization', 'rainbow_navigation']
  off_delay_s   10.0
  period_s      1.0     how often subscribers are checked
  programs_period_s 5.0 how often supervisor is asked
  retry_s       30.0
"""

import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import SetBool

SCAN = '/scan'
CALL_TIMEOUT_S = 15.0  # the firmware answers within ~5 s (10 s while changing speed)


def running_programs(status_output):
    """Names of the RUNNING/STARTING programs in `supervisorctl status` output."""
    names = []
    for line in status_output.splitlines():
        parts = line.split()
        if len(parts) > 1 and parts[1] in ('RUNNING', 'STARTING'):
            names.append(parts[0])
    return names


def lidar_wanted(subscriber_nodes, running, ignore_nodes, programs):
    """Whether anything needs the lidar now, and what (for the log)."""
    users = sorted({n for n in subscriber_nodes if n not in ignore_nodes})
    users += [p for p in programs if p in running]
    return bool(users), users


class LidarPower(Node):

    def __init__(self):
        super().__init__('lidar_power')
        self.ignore_nodes = list(self.declare_parameter('ignore_nodes', ['brain']).value)
        self.programs = list(self.declare_parameter(
            'programs', ['rainbow_slam', 'rainbow_localization', 'rainbow_navigation']).value)
        self.off_delay = self.declare_parameter('off_delay_s', 10.0).value
        self.retry = self.declare_parameter('retry_s', 30.0).value
        period = self.declare_parameter('period_s', 1.0).value
        self.programs_period = self.declare_parameter('programs_period_s', 5.0).value

        self.client = self.create_client(SetBool, 'lidar/enable')
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(String, 'esp_firmware', self.on_esp_session, latched)

        self.last_wanted = None      # monotonic time something last wanted it
        self.sent = None             # state the ESP confirmed (None: unknown)
        self.in_flight = None        # (state, future, started)
        self.retry_at = 0.0
        self.running = []
        self.programs_checked = 0.0
        self.users = []
        self.create_timer(period, self.tick)

    def on_esp_session(self, msg):
        # A new ESP session boots with the lidar in standby: send the wanted state again.
        self.get_logger().info(f'ESP connected ({msg.data}); lidar state will be sent again')
        self.sent = None
        self.retry_at = 0.0

    def check_programs(self, now):
        if now - self.programs_checked < self.programs_period:
            return
        self.programs_checked = now
        try:
            out = subprocess.run(['supervisorctl', 'status', *self.programs],
                                 capture_output=True, text=True, timeout=10).stdout
        except (OSError, subprocess.SubprocessError) as e:
            self.get_logger().warn(f'supervisorctl status: {e}', throttle_duration_sec=60.0)
            out = ''
        self.running = running_programs(out)

    def tick(self):
        now = time.monotonic()
        self.check_programs(now)
        subs = [i.node_name for i in self.get_subscriptions_info_by_topic(SCAN)]
        wanted_now, users = lidar_wanted(subs, self.running, self.ignore_nodes, self.programs)
        if wanted_now:
            self.last_wanted = now
            if users != self.users:
                self.get_logger().info(f'lidar wanted by: {", ".join(users)}')
        self.users = users if wanted_now else []
        want = self.last_wanted is not None and now - self.last_wanted < self.off_delay

        if self.in_flight is not None:
            self.check_call(now)
            return
        if want != self.sent and now >= self.retry_at and self.client.service_is_ready():
            self.in_flight = (want, self.client.call_async(SetBool.Request(data=want)), now)

    def check_call(self, now):
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
