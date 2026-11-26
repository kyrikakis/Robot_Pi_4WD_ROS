"""Pan/tilt head.

Drives the two head servos on the ESP board and publishes the head's
/joint_states, so robot_state_publisher moves every frame on the head
(camera, ToF, eyes) with it.

The servos have no position feedback and jump at full speed to the angle they
are sent, so every move is planned here (rainbow_drivers.head_motion: limited
speed and acceleration, both axes arriving together). Our ESP firmware runs the
move itself: the node sends it one servo_goal per new target (angles and each
servo's speed and acceleration), and the firmware steps it once per 20 ms servo
frame, in fractions of a degree. The node runs the same model to know where the
head is, and /joint_states follows it while the head moves, a little late
(servo_latency_s) to match the servos. With esp_motion false (the stock
firmware) the node streams the model's position instead, in whole degrees on
servo_s1 / servo_s2 at control_rate. After each move the head counts as settled only after a
settle time that grows with the size of the move; photos should wait for it.

Interfaces:
  ~/move          rainbow_interfaces/action/MoveHead   move and wait until settled
  /head/target    rainbow_interfaces/msg/HeadTarget    continuous tracking (e.g.
                                                       faces): latest message wins
  /head/jog       geometry_msgs/Twist                  teleop style (Foxglove's
                                                       Teleop panel): angular.z
                                                       turns left/right, linear.x
                                                       up/down, in rad/s while held
  ~/set_pose      rainbow_interfaces/srv/SetHeadPose   start a move, no waiting
  ~/center        std_srvs/srv/Trigger                 move back to 0, 0
  /head/z_rotation, /head/y_rotation  std_msgs/Int32   pan/tilt in degrees
                                                       (kept from the old node)
  ~/moving        std_msgs/Bool (out, latched)         true while moving or settling
  servo_goal  rainbow_esp_msgs/HeadServoGoal (out)     the move for the ESP, on every
                                                       new target and every second
  servo_s1, servo_s2  std_msgs/Int32 (out)             with esp_motion false: servo
                                                       angles, on every change and
                                                       every second
  /joint_states       sensor_msgs/JointState (out)

Every new target (from any of these) replaces a running ~/move goal.
At start the servos' real position is unknown: the first command (centre) is
reached at the servos' own full speed.
"""

from collections import deque
import math
import threading
import time

from geometry_msgs.msg import Twist
from rainbow_drivers.head_motion import HeadMotion
from rainbow_esp_msgs.msg import HeadServoGoal
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
        # 60 deg/s for every move, tracking too (the owner, 2026-11-26, moves run on
        # the ESP): 10-30 deg/s jerky (the servos' ~1 deg deadband makes them move
        # in visible jumps), 45-60 better, 90 smoother but 60 preferred when
        # following a face, 150 not jerky but very fast. Settle times and latency
        # are estimates until measured
        self.motion = HeadMotion(
            pan_limits, tilt_limits,
            speed=param('speed_deg_s', 60.0).value,
            accel=param('accel_deg_s2', 4000.0).value,
            settle_base=param('settle_base_s', 0.2).value,
            settle_per_deg=param('settle_per_deg_s', 0.005).value)
        self.latency = param('servo_latency_s', 0.05).value
        control_rate = param('control_rate', 50.0).value
        idle_rate = param('joint_state_rate', 10.0).value
        resend = param('servo_resend_period', 1.0).value
        # True: our firmware runs the moves (servo_goal); false: stream whole
        # degrees on servo_s1 / servo_s2 (the stock firmware)
        self.esp_motion = param('esp_motion', True).value
        # A held jog button sends a message every 1/publish rate; the first one
        # (or one after a gap) moves the target this far ahead in time
        self.jog_step = param('jog_step_s', 0.2).value
        self.last_jog = 0.0

        self.lock = threading.Lock()
        self.generation = 0      # bumped by every new target: replaces running goals
        self.sent = None         # last servo command (ints)
        self.moving = False
        self.dt = 1.0 / control_rate
        self.last_tick = 0.0
        self.idle_every = max(1, int(round(control_rate / idle_rate)))
        self.ticks = 0
        # Commanded positions with their times, for the servo latency
        self.history = deque(maxlen=max(2, int(control_rate) * 2))

        if self.esp_motion:
            self.goal_pub = self.create_publisher(HeadServoGoal, 'servo_goal', 10)
        else:
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
        self.create_subscription(Twist, '/head/jog', self.on_jog, 10, callback_group=group)
        self.create_service(SetHeadPose, '~/set_pose', self.on_set_pose, callback_group=group)
        self.create_service(Trigger, '~/center', self.on_center, callback_group=group)
        self.move_server = ActionServer(
            self, MoveHead, '~/move', self.execute_move,
            cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=group)

        self.history.append((time.monotonic(), 0.0, 0.0))
        self.resend()
        self.moving_pub.publish(Bool(data=False))
        self.publish_joint_states()
        timers = MutuallyExclusiveCallbackGroup()
        self.create_timer(self.dt, self.tick, callback_group=timers)
        # Keep resending: after an ESP reset the servos get no pulse until the
        # first command (the stock firmware jumped to its default pose instead),
        # which would leave the head (and the calibrated ToF) somewhere other than
        # where /joint_states says it is. A resent goal does not change a move.
        self.create_timer(resend, self.resend, callback_group=timers)

    # Targets (degrees)

    def set_target(self, pan_deg=None, tilt_deg=None, speed_deg_s=None):
        """Aim the head; returns (generation, clamped pan, clamped tilt)."""
        with self.lock:
            self.generation += 1
            pan, tilt = self.motion.set_target(pan_deg, tilt_deg, speed_deg_s)
            generation = self.generation
        self.send_goal()
        return generation, pan, tilt

    def stop_here(self):
        with self.lock:
            self.generation += 1
            target = self.motion.stop()
        self.send_goal()
        return target

    # Control loop

    def tick(self):
        now = time.monotonic()
        # A clock jump makes the timer fire its missed ticks in a burst; each tick
        # moves the head one step, so a burst would jerk it at full speed
        if now - self.last_tick < 0.5 * self.dt:
            return
        self.last_tick = now
        with self.lock:
            busy = self.motion.step(now, self.dt)
            pan, tilt = self.motion.position
        self.history.append((now, pan, tilt))
        if not self.esp_motion:
            self.send_servos()
        if busy != self.moving:
            self.moving = busy
            self.moving_pub.publish(Bool(data=busy))
        self.ticks += 1
        # Full rate while the head moves (the ToF and camera frames need the
        # right pose), the idle rate otherwise
        if busy or self.ticks % self.idle_every == 0:
            self.publish_joint_states(now)

    def to_servo(self, pan, tilt):
        """Convert head degrees to servo degrees (S1, S2)."""
        return (self.pan_sign * pan + self.pan_offset, self.tilt_sign * tilt + self.tilt_offset)

    def send_goal(self):
        """Send the current move to the ESP (esp_motion)."""
        if not self.esp_motion:
            return
        with self.lock:
            target = self.motion.target
            (pan_speed, pan_accel), (tilt_speed, tilt_accel) = self.motion.limits()
        s1, s2 = self.to_servo(*target)
        self.goal_pub.publish(HeadServoGoal(
            s1_deg=float(s1), s2_deg=float(s2),
            s1_speed=float(pan_speed), s2_speed=float(tilt_speed),
            s1_accel=float(pan_accel), s2_accel=float(tilt_accel)))

    def resend(self):
        # Every second: after an ESP reset its servos get no pulse until a command
        if self.esp_motion:
            self.send_goal()
        else:
            self.send_servos(force=True)

    def servo_command(self):
        with self.lock:
            pan, tilt = self.motion.position
        s1, s2 = self.to_servo(pan, tilt)
        return int(round(s1)), int(round(s2))

    def send_servos(self, force=False):
        command = self.servo_command()
        if not force and command == self.sent:
            return
        self.sent = command
        self.pan_pub.publish(Int32(data=command[0]))
        self.tilt_pub.publish(Int32(data=command[1]))

    def lagged_position(self, now):
        """Return the commanded position servo_latency_s ago: where the servos are now."""
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

    def on_jog(self, msg):
        # The up button (linear.x > 0) raises the head: tilt + is down
        pan_rate = math.degrees(msg.angular.z)
        tilt_rate = -math.degrees(msg.linear.x)
        now = time.monotonic()
        dt = min(now - self.last_jog, self.jog_step)
        self.last_jog = now
        if pan_rate == 0.0 and tilt_rate == 0.0:
            return
        with self.lock:
            pan, tilt = self.motion.target
        # A little faster than the rate, so the head keeps up with the target
        speed = 1.5 * max(abs(pan_rate), abs(tilt_rate))
        self.set_target(pan + pan_rate * dt, tilt + tilt_rate * dt, speed)

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
