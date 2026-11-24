"""Pan/tilt head.

Drives the two head servos on the ESP board and publishes the head's
/joint_states, so robot_state_publisher moves every frame on the head
(camera, ToF, eyes) with it.

The servos have no position feedback and jump at full speed to the angle they
are sent, so the node moves the commanded angle smoothly (rainbow_drivers.head_motion:
limited speed and acceleration, both axes arriving together) and /joint_states
follows that commanded angle while the head moves, a little late (servo_latency_s)
to match the servos. After each move the head counts as settled only after a
settle time that grows with the size of the move; photos should wait for it.

Interfaces:
  ~/move          rainbow_interfaces/action/MoveHead   move and wait until settled
  /head/target    rainbow_interfaces/msg/HeadTarget    continuous tracking (e.g.
                                                       faces): latest message wins
  ~/set_pose      rainbow_interfaces/srv/SetHeadPose   start a move, no waiting
  ~/center        std_srvs/srv/Trigger                 move back to 0, 0
  /head/z_rotation, /head/y_rotation  std_msgs/Int32   pan/tilt in degrees
                                                       (kept from the old node)
  ~/moving        std_msgs/Bool (out, latched)         true while moving or settling
  servo_s1, servo_s2  std_msgs/Int32 (out)             servo angles for the ESP,
                                                       on every change and every second
  /joint_states       sensor_msgs/JointState (out)

Every new target (from any of these) replaces a running ~/move goal.
At start the servos' real position is unknown: the first command (centre) is
reached at the servos' own full speed.
"""

from collections import deque
import math
import threading
import time

from rainbow_drivers.head_motion import HeadMotion
from rainbow_interfaces.action import MoveHead
from rainbow_interfaces.msg import HeadTarget
from rainbow_interfaces.srv import SetHeadPose
import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Int32
from std_srvs.srv import Trigger

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class HeadNode(Node):

    def __init__(self):
        super().__init__('head')

        # Limits are in degrees of head motion; servo_* map head degrees to the
        # servo command the ESP expects: servo = sign * head + offset
        param = self.declare_parameter
        pan_limits = (param('pan_min_deg', -40.0).value, param('pan_max_deg', 38.0).value)
        tilt_limits = (param('tilt_min_deg', -36.0).value, param('tilt_max_deg', 19.0).value)
        self.pan_sign = param('pan_servo_sign', -1).value
        self.pan_offset = param('pan_servo_offset_deg', 0).value
        self.tilt_sign = param('tilt_servo_sign', -1).value
        self.tilt_offset = param('tilt_servo_offset_deg', -56).value
        self.pan_joint = param('pan_joint', 'head_pan_joint').value
        self.tilt_joint = param('tilt_joint', 'head_tilt_joint').value
        # Calm by default; settle times and latency are estimates until measured
        self.motion = HeadMotion(
            pan_limits, tilt_limits,
            speed=param('speed_deg_s', 30.0).value,
            accel=param('accel_deg_s2', 60.0).value,
            settle_base=param('settle_base_s', 0.2).value,
            settle_per_deg=param('settle_per_deg_s', 0.005).value)
        self.latency = param('servo_latency_s', 0.05).value
        control_rate = param('control_rate', 50.0).value
        idle_rate = param('joint_state_rate', 10.0).value
        resend = param('servo_resend_period', 1.0).value

        self.lock = threading.Lock()
        self.generation = 0      # bumped by every new target: replaces running goals
        self.sent = None         # last servo command (ints)
        self.moving = False
        self.dt = 1.0 / control_rate
        self.idle_every = max(1, int(round(control_rate / idle_rate)))
        self.ticks = 0
        # Commanded positions with their times, for the servo latency
        self.history = deque(maxlen=max(2, int(control_rate) * 2))

        self.pan_pub = self.create_publisher(Int32, 'servo_s1', 10)
        self.tilt_pub = self.create_publisher(Int32, 'servo_s2', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.moving_pub = self.create_publisher(Bool, '~/moving', LATCHED)

        group = ReentrantCallbackGroup()
        self.create_subscription(Int32, '/head/z_rotation', self.on_pan_deg, 10,
                                 callback_group=group)
        self.create_subscription(Int32, '/head/y_rotation', self.on_tilt_deg, 10,
                                 callback_group=group)
        self.create_subscription(HeadTarget, '/head/target', self.on_target, 10,
                                 callback_group=group)
        self.create_service(SetHeadPose, '~/set_pose', self.on_set_pose, callback_group=group)
        self.create_service(Trigger, '~/center', self.on_center, callback_group=group)
        self.move_server = ActionServer(
            self, MoveHead, '~/move', self.execute_move,
            cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=group)

        self.history.append((time.monotonic(), 0.0, 0.0))
        self.send_servos(force=True)
        self.moving_pub.publish(Bool(data=False))
        self.publish_joint_states()
        timers = MutuallyExclusiveCallbackGroup()
        self.create_timer(self.dt, self.tick, callback_group=timers)
        # Keep resending the pose: when the ESP resets or reconnects its servos go
        # to the firmware's default position, which would leave the head (and the
        # calibrated ToF) somewhere other than where /joint_states says it is.
        self.create_timer(resend, lambda: self.send_servos(force=True), callback_group=timers)

    # Targets (degrees)

    def set_target(self, pan_deg=None, tilt_deg=None, speed_deg_s=None):
        """Aim the head; returns (generation, clamped pan, clamped tilt)."""
        with self.lock:
            self.generation += 1
            pan, tilt = self.motion.set_target(pan_deg, tilt_deg, speed_deg_s)
            return self.generation, pan, tilt

    def stop_here(self):
        with self.lock:
            self.generation += 1
            return self.motion.stop()

    # Control loop

    def tick(self):
        now = time.monotonic()
        with self.lock:
            busy = self.motion.step(now, self.dt)
            pan, tilt = self.motion.position
        self.history.append((now, pan, tilt))
        self.send_servos()
        if busy != self.moving:
            self.moving = busy
            self.moving_pub.publish(Bool(data=busy))
        self.ticks += 1
        # Full rate while the head moves (the ToF and camera frames need the
        # right pose), the idle rate otherwise
        if busy or self.ticks % self.idle_every == 0:
            self.publish_joint_states(now)

    def servo_command(self):
        with self.lock:
            pan, tilt = self.motion.position
        return (int(round(self.pan_sign * pan + self.pan_offset)),
                int(round(self.tilt_sign * tilt + self.tilt_offset)))

    def send_servos(self, force=False):
        command = self.servo_command()
        if not force and command == self.sent:
            return
        self.sent = command
        self.pan_pub.publish(Int32(data=command[0]))
        self.tilt_pub.publish(Int32(data=command[1]))

    def lagged_position(self, now):
        """The commanded position servo_latency_s ago: where the servos are now."""
        when = now - self.latency
        pan, tilt = self.history[0][1:]
        for t, p, q in self.history:
            if t > when:
                break
            pan, tilt = p, q
        return pan, tilt

    def publish_joint_states(self, now=None):
        pan, tilt = self.lagged_position(time.monotonic() if now is None else now)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [self.pan_joint, self.tilt_joint]
        msg.position = [math.radians(pan), math.radians(tilt)]
        self.joint_pub.publish(msg)

    # Interfaces

    def on_pan_deg(self, msg):
        self.set_target(pan_deg=float(msg.data))

    def on_tilt_deg(self, msg):
        self.set_target(tilt_deg=float(msg.data))

    def on_target(self, msg):
        self.set_target(math.degrees(msg.pan), math.degrees(msg.tilt),
                        math.degrees(msg.speed))

    def on_set_pose(self, request, response):
        _, pan, tilt = self.set_target(math.degrees(request.pan), math.degrees(request.tilt))
        response.success = True
        response.pan = math.radians(pan)
        response.tilt = math.radians(tilt)
        response.message = f'moving to pan {pan:.0f} deg, tilt {tilt:.0f} deg'
        return response

    def on_center(self, request, response):
        self.set_target(0.0, 0.0)
        response.success = True
        response.message = 'centering'
        return response

    def execute_move(self, goal_handle):
        goal = goal_handle.request
        generation, pan, tilt = self.set_target(
            math.degrees(goal.pan), math.degrees(goal.tilt), math.degrees(goal.speed))
        result = MoveHead.Result(pan=math.radians(pan), tilt=math.radians(tilt))
        feedback = MoveHead.Feedback()
        while rclpy.ok():
            time.sleep(self.dt)
            with self.lock:
                replaced = self.generation != generation
                settled = self.motion.settled
                now_pan, now_tilt = self.motion.position
            result.pan, result.tilt = math.radians(now_pan), math.radians(now_tilt)
            if goal_handle.is_cancel_requested:
                self.stop_here()
                result.message = 'stopped'
                goal_handle.canceled()
                return result
            if replaced:
                result.message = 'replaced by a new head target'
                goal_handle.abort()
                return result
            if settled:
                result.success = True
                result.message = f'looking at pan {now_pan:.0f} deg, tilt {now_tilt:.0f} deg'
                goal_handle.succeed()
                return result
            feedback.pan, feedback.tilt = result.pan, result.tilt
            goal_handle.publish_feedback(feedback)
        result.message = 'shutting down'
        goal_handle.abort()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = HeadNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
