#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import String, Float32
import matplotlib.pyplot as plt
import time


def callback(data):
    global om_comp, aplha, time_last, ax, fig#, f
    gx = data.angular_velocity.x

    om_comp = om_comp * alpha + gx * (1 - alpha)

    time_now = rospy.Time.now().to_sec()
    dt = time_now - time_last
    data_pub = str(gx) + "   " + str(om_comp) + "    " + str(dt)
    #f.write(data_pub + "\n")
    pub_data.publish(data_pub)
    pub_filter.publish(om_comp)


if __name__ == '__main__':
    om_comp = 0
    alpha = 0.7  # 0.6...1
    #file_name = "/home/ubuntu/catkin_ws/src/tech_vision/data/"
    #f = open(file_name + "filter" + ".txt", "w")
    fig = plt.figure()
    ax = fig.add_subplot(111)
    pub_data = rospy.Publisher('all_data_filtered', String, queue_size=10)
    pub_filter = rospy.Publisher('/filter', Float32, queue_size=10)
    rospy.init_node('filter', anonymous=True)
    time_last = rospy.Time.now().to_sec()
    rate = rospy.Rate(10)
    rospy.Subscriber("imu", Imu, callback)
    rospy.spin()
   # f.close()
