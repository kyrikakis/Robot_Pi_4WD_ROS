"""Eyes: two 8x8 LED matrices on an HT16K33 (I2C).

Interfaces:
  ~/set_expression  rainbow_interfaces/srv/SetExpression
  /eyes             std_msgs/String   expression name (kept from the old node)

Expressions: happy (smile with a blink), sad (frown with a blink; low battery), open,
closed, off.

The HT16K33 drops off the bus now and then while the robot moves (2026-11-25: Remote
I/O errors, then the chip back with its display off and the eyes dark until a reboot).
So I2C errors do not stop the node, and every `refresh_period` seconds it turns the
chip's oscillator and display on again and redraws the current frame.
"""

from rainbow_interfaces.srv import SetExpression
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

_ = False
X = True

SMILE = [
    [_, _, _, _, _, _, _, _],
    [_, _, _, _, _, _, _, _],
    [_, _, X, _, _, X, _, _],
    [_, _, _, X, X, _, _, _],
    [_, _, _, _, _, _, _, _],
    [_, _, _, _, _, _, _, _],
    [_, _, _, _, _, _, _, _],
    [_, _, _, _, _, _, _, _],
]
FROWN = [
    [_, _, _, _, _, _, _, _],
    [_, _, _, _, _, _, _, _],
    [_, _, _, X, X, _, _, _],
    [_, _, X, _, _, X, _, _],
    [_, _, _, _, _, _, _, _],
    [_, _, _, _, _, _, _, _],
    [_, _, _, _, _, _, _, _],
    [_, _, _, _, _, _, _, _],
]
SHUT = [
    [_, _, _, _, _, _, _, _],
    [_, _, _, _, _, _, _, _],
    [_, _, _, _, _, _, _, _],
    [_, _, X, X, X, X, _, _],
    [_, _, _, _, _, _, _, _],
    [_, _, _, _, _, _, _, _],
    [_, _, _, _, _, _, _, _],
    [_, _, _, _, _, _, _, _],
]
OPEN = [
    [_, _, _, _, _, _, _, _],
    [_, _, _, _, _, _, _, _],
    [_, _, _, X, X, _, _, _],
    [_, _, X, X, X, X, _, _],
    [_, _, _, X, X, _, _, _],
    [_, _, _, _, _, _, _, _],
    [_, _, _, _, _, _, _, _],
    [_, _, _, _, _, _, _, _],
]
BLANK = [[_] * 8 for _row in range(8)]

# Each expression is a list of (frame, seconds) shown in a loop.
EXPRESSIONS = {
    'happy': [(SMILE, 3.0), (SHUT, 1.0)],
    'sad': [(FROWN, 3.0), (SHUT, 1.0)],
    'open': [(OPEN, None)],
    'closed': [(SHUT, None)],
    'off': [(BLANK, None)],
}


class EyesNode(Node):

    def __init__(self):
        super().__init__('eyes')
        self.address = self.declare_parameter('i2c_address', 0x71).value
        expression = self.declare_parameter('expression', 'happy').value
        refresh_period = self.declare_parameter('refresh_period', 2.0).value

        # Imported here so the module can be loaded (and tested) off the robot
        import board
        import busio
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.matrix = None
        self.failing = False   # last I2C access failed (warn once per outage)
        self.frame = BLANK
        self.open_matrix()

        self.create_subscription(
            String, '/eyes', self.on_topic,
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       history=HistoryPolicy.KEEP_LAST, depth=1))
        self.create_service(SetExpression, '~/set_expression', self.on_service)

        self.timer = None
        self.set_expression(expression)
        self.create_timer(refresh_period, self.refresh)

    def open_matrix(self):
        from adafruit_ht16k33 import matrix
        try:
            # auto_write off: one bus write per frame, not one per pixel (128)
            self.matrix = matrix.Matrix16x8(self.i2c, address=self.address, auto_write=False)
        except OSError as e:
            self.i2c_failed(e)
            return False
        return True

    def refresh(self):
        """Turn the chip on again (it may have reset) and redraw the frame."""
        if self.matrix is None:
            if self.open_matrix():
                self.draw(self.frame)
            return
        try:
            # The same commands Matrix16x8 sends at start; harmless when repeated
            self.matrix._write_cmd(0x21)   # oscillator on
            self.matrix.blink_rate = 0     # display on, no blinking
            self.matrix.brightness = self.matrix.brightness
        except OSError as e:
            self.i2c_failed(e)
            return
        self.draw(self.frame)

    def i2c_failed(self, error):
        if not self.failing:
            self.get_logger().warning(f'eyes not answering on I2C ({error}); retrying')
        self.failing = True

    def set_expression(self, name):
        if name not in EXPRESSIONS:
            return False
        self.expression = name
        self.step = 0
        self.show_step()
        return True

    def show_step(self):
        if self.timer is not None:
            self.timer.cancel()
            self.destroy_timer(self.timer)
            self.timer = None
        frames = EXPRESSIONS[self.expression]
        frame, seconds = frames[self.step % len(frames)]
        self.draw(frame)
        if seconds is not None and len(frames) > 1:
            self.timer = self.create_timer(seconds, self.next_step)

    def next_step(self):
        self.step += 1
        self.show_step()

    def draw(self, frame):
        self.frame = frame
        if self.matrix is None:
            return   # the refresh timer draws it once the chip answers
        # The same 8x8 frame on both eyes; the matrix is addressed [column, row]
        for row in range(8):
            for col in range(16):
                self.matrix[col, row] = frame[row][col % 8]
        try:
            self.matrix.show()
        except OSError as e:
            self.i2c_failed(e)
            return
        if self.failing:
            self.get_logger().info('eyes answering on I2C again')
            self.failing = False

    def on_topic(self, msg):
        if not self.set_expression(msg.data):
            self.get_logger().warning(
                f"unknown expression '{msg.data}', expected one of {sorted(EXPRESSIONS)}")

    def on_service(self, request, response):
        response.success = self.set_expression(request.expression)
        response.message = ('ok' if response.success else
                            f'unknown expression, expected one of {sorted(EXPRESSIONS)}')
        return response

    def clear(self):
        self.draw(BLANK)


def main(args=None):
    rclpy.init(args=args)
    node = EyesNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.clear()
        node.destroy_node()
        rclpy.try_shutdown()
