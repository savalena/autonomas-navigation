#!/usr/bin/python
import math
import time
from pyiArduinoI2Cexpander import *


class Motor(pyiArduinoI2Cexpander):
    def __init__(self, pin_left=2, pin_right=3):
        super(pyiArduinoI2Cexpander, self).__init__(0x08)
        self.LEFT_CONST = 1520
        self.RIGHT_CONST = 1415
        self.PIN_LEFT = pin_left
        self.PIN_RIGHT = pin_right
        self.pinMode(self.PIN_LEFT, OUTPUT, SERVO)
        self.pinMode(self.PIN_RIGHT, OUTPUT, SERVO)
        self.servoAttach(self.PIN_LEFT, 1300, 1590, -100, 100)
        self.servoAttach(self.PIN_RIGHT, 1300, 1590, -100, 100)

    def set_speed(self, om_left, om_right):
        analog_om_left = self.LEFT_CONST + om_left * 4
        analog_om_right = self.RIGHT_CONST - om_right * 4
        self.servoWriteMicroseconds(self.PIN_LEFT, analog_om_left)
        self.servoWriteMicroseconds(self.PIN_RIGHT, analog_om_right)


if __name__ == '__main__':
    motors = Motor()
    time.sleep(1)
    motors.set_speed(0, 0)
    time.sleep(1)
