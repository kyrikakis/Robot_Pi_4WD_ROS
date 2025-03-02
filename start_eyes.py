import board
import busio
from adafruit_ht16k33 import matrix
import time
import rclpy
from rclpy.node import Node
from threading import Thread
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class MATRIX_MODE:
    NONE = "1"
    DIZZY = "2"
    CRY = "3"
    HAPPY = "4"
    RIGHT = "5"
    LEFT = "6"
    BLINK = "7"
    OPEN = "8"
    RANDOM = "9"

eyeSmile = [
    [False, False, False, False, False, False, False, False],
    [False, False, False, False, False, False, False, False],
    [False, False, True, False, False, True, False, False],
    [False, False, False, True, True, False, False, False],
    [False, False, False, False, False, False, False, False],
    [False, False, False, False, False, False, False, False],
    [False, False, False, False, False, False, False, False],
    [False, False, False, False, False, False, False, False]
]
eyeShut = [
    [False, False, False, False, False, False, False, False],
    [False, False, False, False, False, False, False, False],
    [False, False, False, False, False, False, False, False],
    [False, False, True, True, True, True, False, False],
    [False, False, False, False, False, False, False, False],
    [False, False, False, False, False, False, False, False],
    [False, False, False, False, False, False, False, False],
    [False, False, False, False, False, False, False, False]
]
eyeOpen = [
    [False, False, False, False, False, False, False, False],
    [False, False, False, False, False, False, False, False],
    [False, False, False, True, True, False, False, False],
    [False, False, True, True, True, True, False, False],
    [False, False, False, True, True, False, False, False],
    [False, False, False, False, False, False, False, False],
    [False, False, False, False, False, False, False, False],
    [False, False, False, False, False, False, False, False]
]

eyeOpenFill = [
    [False, False, False, False, False, False, False, False],
    [False, False, True, True, True, True, False, False],
    [False, True, True, True, True, True, True, False],
    [False, True, True, True, True, True, True, False],
    [False, True, True, True, True, True, True, False],
    [False, False, True, True, True, True, False, False],
    [False, False, False, False, False, False, False, False],
    [False, False, False, False, False, False, False, False]
]

class CustomLEDMatrixController(Node):
    animation = MATRIX_MODE.NONE
    
    def __init__(self, i2c_address=0x71):
        super().__init__('eyes')
        self.subscription = self.create_subscription(
            String,
            '/eyes',
            self.callback,
            QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1))
        # Initialize I2C bus and HT16K33 LED matrix with the specified address
        i2c = busio.I2C(board.SCL, board.SDA)
        self.matrix = matrix.Matrix16x8(i2c, address=i2c_address)

    def eyes_blink(self):
        self.clear_display()
        while self.animation==MATRIX_MODE.BLINK:
            # Fill the display with the eyeSmile data for both left and right
            self.fill_display(eyeOpenFill)
            # Update the physical display
            self.show_display()
            # Fill the display with the eyeSmile data for both left and right
            self.fill_display(eyeOpen)
            # Update the physical display
            self.show_display()
    
    def eyes_smile(self):
        self.clear_display()
        while self.animation==MATRIX_MODE.HAPPY:
            # Fill the display with the eyeSmile data for both left and right
            self.fill_display(eyeSmile)
            # Update the physical display
            self.show_display()

            start_time = time.time()
            while time.time() - start_time < 3.0: #sec
                if self.animation!=MATRIX_MODE.HAPPY:
                    break
                time.sleep(0.1)
                
            # Fill the display with the eyeSmile data for both left and right
            if self.animation==MATRIX_MODE.HAPPY:
                self.fill_display(eyeShut)
            # Update the physical display
            if self.animation==MATRIX_MODE.HAPPY:
                self.show_display()
                
            start_time = time.time()
            while time.time() - start_time < 1.0: #sec
                if self.animation!=MATRIX_MODE.HAPPY:
                    break
                time.sleep(0.1)

    def eyes_open(self):
        self.clear_display()
        open = False
        while self.animation==MATRIX_MODE.OPEN:
            if open == False:
                self.fill_display(eyeOpen)
                self.show_display()
                open = True
            time.sleep(0.1)

    def eyes_movement(self):
        while rclpy.ok():
            match self.animation:
                case MATRIX_MODE.HAPPY:
                    self.eyes_smile()
                case MATRIX_MODE.OPEN:
                    self.eyes_open()
            
    def fill_display(self, data):
        """
        Fill the display with data (16x8 list of True/False values).
        """
        for row in range(8):
            for col in range(16):
                # Invert rows and columns here
                self.matrix[col, row] = data[row][col % 8]

    def clear_display(self):
        """
        Clear the display (turn off all LEDs).
        """
        self.fill_display([[False] * 16 for _ in range(8)])

    def show_display(self):
        """
        Update the physical display with the current data.
        """
        self.matrix.show()

    def callback(self, msg):
        if msg.data == 'happy':
          self.animation = MATRIX_MODE.HAPPY
        elif msg.data == 'open':
          self.animation = MATRIX_MODE.OPEN
          
def main(args=None):
    rclpy.init(args=args)

    display = CustomLEDMatrixController()
    display.animation = MATRIX_MODE.HAPPY
    thread = Thread(target = CustomLEDMatrixController.eyes_movement, args=(display,))
    thread.start()

    rclpy.spin(display)
    thread.join()
    display.clear_display()
    display.show_display()
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    display.destroy_node()
    rclpy.shutdown()

# Main program logic follows:
if __name__ == '__main__':
    main()