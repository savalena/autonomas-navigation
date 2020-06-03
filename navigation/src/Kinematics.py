#!/usr/bin/python
# -*-coding: utf-8 -*-
import time
from pyiArduinoI2Cexpander import *
import numpy as np


class MobilePlatform:

    def __init__(self, om_l, om_r, vel, ang_vel, length=0.085, rad=0.035, phi=0):
        self.L = length
        self.R = rad
        self.FULL_SPEED = 0.14 # defined experementally
        self.om_left = om_l
        self.om_right = om_r
        self.vel = vel
        self.ang_vel = ang_vel
        self.phi = phi
        self.om_left_max, self.om_right_max = self.inverted_kin(self.FULL_SPEED, 0)
        self.MIN_SPEED = self.direct_kin(self.om_left_max, 0)[0]

    def set_platform_speed(self, vel, ang_vel):
        """set wheel angular velocities in class attributes"""
        self.vel = vel
        self.ang_vel = ang_vel

    def set_wheel_speed(self, om_l, om_r):
        """"set linear angular velocities in class attributes"""
        self.om_left = om_l
        self.om_right = om_r

    def left_rotation(self, ang_vel):
        """calculate angular velocity of left wheel"""
        vel = self.om_right_max * self.R - ang_vel * self.L
        om_left = (vel - ang_vel * self.L) / self.R -1
        return vel, om_left

    def right_rotation(self, ang_vel):
        """calculate angular velocity of right wheel"""
        vel = self.om_left_max * self.R + ang_vel * self.L
        om_right = (vel + ang_vel * self.L) / self.R 
        return vel, om_right


    def direct_kin(self, om_l, om_r):
        """Kinematics equations"""
        vel = (om_l + om_r) * self.R / 2
        ang_vel = (om_r - om_l) * self.R / (2 * self.L)
        return vel, ang_vel

    def inverted_kin(self, vel, ang_vel):
        """inverse kinematics equations"""
        om_l = (vel - ang_vel * self.L) / self.R
        om_r = (vel + ang_vel * self.L) / self.R
        return om_l, om_r

    def convert_om_to10(self, om):
        """перевод из рад/c в условные 0...10"""
        om10 = 10 / (self.om_left_max) * om
        return np.ceil(om10)


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
        """set speed on motors and run it"""
        analog_om_left = self.LEFT_CONST + om_left*4
        analog_om_right = self.RIGHT_CONST - om_right*4
        self.servoWriteMicroseconds(self.PIN_LEFT, analog_om_left)
        self.servoWriteMicroseconds(self.PIN_RIGHT, analog_om_right)


if __name__ == '__main__':
    motors = Motor()
    motors.set_speed(10, 5)
    time.sleep(5)
    motors.set_speed(5, 10)
    time.sleep(5)