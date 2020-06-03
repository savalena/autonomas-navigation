#!/usr/bin/python
# -*-coding: utf-8 -*-
import rospy
from sensor_msgs.msg import LaserScan
from Kinematics import Motor
import time


class ListenScan(Motor):
    def __init__(self):
        Motor.__init__(self)
        self.listen_scan = rospy.Subscriber("/scan", LaserScan, self.callback)
        self.sc1 = LaserScan()
        self.sc2 = LaserScan()
        self.it = 0
        self.twait = 2
        self.f = "/home/ubuntu/catkin_ws/src/tech_vision/data/"

    def callback(self, data):
        """defines initial position; takes two scans"""
        if self.it == 0 and data.ranges != []:
            self.sc1 = data
            f = open(self.f + str(self.it) + ".txt", "w")
            f.write(str(data))
            print("get1")
            f.close()
            self.set_speed(10, 10)
            self.set_speed(10, 10)
            time.sleep(self.twait)
            self.set_speed(0, 0)
            self.it = self.it + 1
        if self.it == 1 and data.ranges != []:
            self.sc2 = data
            f = open(self.f + str(self.it) + ".txt", "w")
            f.write(str(data))
            f.close()
            print("get2")
            self.it = self.it + 1
        if self.it == 2:
            self.it = 2


if __name__ == '__main__':
    rospy.init_node('initpos', anonymous=True)
    ln = ListenScan()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
