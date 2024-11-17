import board
import busio
from adafruit_ht16k33 import matrix
from MatrixMode import MATRIX_MODE
import time


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
    [False, False, True, True, True, True, False, False],
    [False, True, False, False, False, False, True, False],
    [False, True, False, False, False, False, True, False],
    [False, True, False, False, False, False, True, False],
    [False, False, True, True, True, True, True, False],
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

class CustomLEDMatrixController:
    animation = MATRIX_MODE.NONE
    
    def __init__(self, i2c_address=0x71):
        # Initialize I2C bus and HT16K33 LED matrix with the specified address
        i2c = busio.I2C(board.SCL, board.SDA)
        self.matrix = matrix.Matrix16x8(i2c, address=i2c_address)
    
    def eyes_smile(self):
        self.clear_display()
        while self.animation==MATRIX_MODE.HAPPY:
            # Fill the display with the eyeSmile data for both left and right
            self.fill_display(eyeSmile)
            # Update the physical display
            self.show_display()
            time.sleep(3)
            # Fill the display with the eyeSmile data for both left and right
            self.fill_display(eyeShut)
            # Update the physical display
            self.show_display()

    def eyes_blink(self):
        self.clear_display()
        while self.animation==MATRIX_MODE.BLINK:
            # Fill the display with the eyeSmile data for both left and right
            self.fill_display(eyeOpen)
            # Update the physical display
            self.show_display()
            # Fill the display with the eyeSmile data for both left and right
            self.fill_display(eyeOpenFill)
            # Update the physical display
            self.show_display()

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


# Main program logic follows:
if __name__ == '__main__':
    print("Running main. After that, you can press ctrl-C to end the program.")
    display=CustomLEDMatrixController()
    while True:
        try :
            display.eyes_smile()
        except KeyboardInterrupt:
            print ("\nEnd of program")
            break