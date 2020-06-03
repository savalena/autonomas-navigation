#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import cv2
import time

global bridge


def callback(data):
    global img, k, file
    img = bridge.imgmsg_to_cv2(data, "bgr8")  # convert ROS image to CV image
    print(k)
    if k < 2:
        cv2.imwrite(file + "d" + str(k) + ".png", img)
        print(k)
        k = k + 1
        time.sleep(1)


def get_image():
    k = 0
    rospy.init_node('depth', anonymous=True)
    rospy.Subscriber("/camera/depth/image_raw", Image, callback)
    rospy.spin()
