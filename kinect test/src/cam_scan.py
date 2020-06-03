#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
import time


def callback(data):
    global i, files, t0
    r = data.ranges
    i = i + 1
    t = rospy.Time.now().to_sec()
    if i < 2:
        f = open(files + str(i) + "s" + ".txt", "w")
        t0 = rospy.Time.now().to_sec()
        f.write(str(data))
        f.close()


if __name__ == '__main__':
    files = "/home/ubuntu/catkin_ws/src/tech_vision/data/"
    i = 0
    rospy.init_node('cam_scan', anonymous=True)
    t0 = rospy.Time.now().to_sec()
    rospy.Subscriber("scan", LaserScan, callback)
    rospy.spin()
