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
    img = bridge.imgmsg_to_cv2(data, "bgr8")
    if k < 4:
        cv2.imwrite(file + "im" + str(k) + ".png", img)
        print(k)
        k = k + 1
        time.sleep(1)


def get_image():
    rospy.init_node('cam_im', anonymous=True)
    rospy.Subscriber("/camera/rgb/image_raw", Image, callback)
    rospy.spin()


if __name__ == '__main__':
    file = "/home/ubuntu/catkin_ws/src/tech_vision/data/"
    k = 0
    bridge = CvBridge()
    get_image()
