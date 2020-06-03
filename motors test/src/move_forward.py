#!/usr/bin/python
# -*- coding: utf-8 -*-
from kin import Motor
import math
import rospy
from std_msgs.msg import Float32


class ControlServo(Motor):
    def __init__(self):
        Motor.__init__(self)
        self.sub = rospy.Subscriber("/filter", Float32, self.callback)
        self.data = Float32()
        self.om_right = 10
        self.om_left = 10
        self.set_speed(self.om_left, self.om_right)

    def control_by_gyro(self):
        """control moving along straight line"""
        if math.fabs(self.data) > 0.6:
            if self.data > 0:
                self.om_right = self.om_right - 0.7
                self.om_left = self.om_left + 0.5
                self.set_speed(self.om_left, self.om_right)
            else:
                self.om_right = self.om_right + 0.3
                self.om_left = self.om_left - 0.5
                self.set_speed(self.om_left, self.om_right)
        else:
            self.om_right = 10
            self.om_left = 10

    def callback(self, data):
        self.data = data.data
        self.control_by_gyro()


if __name__ == '__main__':
    rospy.init_node('move_forward', anonymous=True)
    obc = ControlServo()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
