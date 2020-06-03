#!/usr/bin/python
# -*-coding: utf-8 -*-
import math
import numpy as np
import rospy
import time
from initpos import ListenScan
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from Kinematics import Motor, MobilePlatform

from stucture import initiate_pos, regular_iterating, RotoTranslationalM, return_traject_data
from control_signal import return_data, trajectori_length, prepare_om_t

from find_segments import plot_line


class ControlServo(Motor):
    def __init__(self):
        Motor.__init__(self)
        self.listen_gyro = rospy.Subscriber("/filter", Float32, self.gyro_callback)
        self.listen_scan = rospy.Subscriber("/scan", LaserScan, self.avoid_callback)

        self.gyro = Float32()
        self.scan = LaserScan()
        self.om_right = 10
        self.om_left = 10
        self.flag = True
        self.iter = 0
        self.time1 = time.time()
        self.time11 = 0
        self.T = RotoTranslationalM(0.14, 0, ttt)
        self.linedata = initiate_pos(scan1, scan2, self.T)

    def move_forward(self, val):
        """move along straight line"""
        val = val * 180 / math.pi
        print("gyro diff", self.gyro - val)
        print("gyrof", self.gyro)
        if math.fabs(self.gyro - val) > 0.6:
            if self.gyro - val > 0:
                self.om_right = self.om_right - 0.7
                self.om_left = self.om_left + 0.5
                self.set_speed(self.om_left, self.om_right)
                print("om_l", self.om_left)
                print("om_r", self.om_right)
            else:
                self.om_right = self.om_right + 0.3
                self.om_left = self.om_left - 0.5
                self.set_speed(self.om_left, self.om_right)
                print("om_l", self.om_left)
                print("om_r", self.om_right)
        else:
            self.om_right = 10
            self.om_left = 10

    def control_by_gyro(self, val):
        if self.flag:
            print("in gyro")
            self.move_forward(val)

    def gyro_callback(self, data):
        self.gyro = data.data  # here is an angular velocity around gyro X axis
        self.control_by_gyro(0)

    def avoid_callback(self, data):
        self.scan = data
        self.avoid_obs_control()

    def control_wheel(self, om_w, time_for_move, side):
        """wheel speed control for obstacles avoidance"""
        for i in range(1, len(time_for_move)):
            if side == 'left':
                print("OMEGA LEFT CONTROL")
                self.set_speed(om_w[i - 1], 10)
                rospy.sleep(time_for_move[i] - time_for_move[i - 1])
            else:
                print("OMEGA RIGHT CONTROL")
                self.set_speed(10, om_w[i - 1])
                rospy.sleep(time_for_move[i] - time_for_move[i - 1] - 0.5)
        self.flag = True
        self.set_speed(10, 10)

    def compare_OM_w_gyro(self, om):
        """compare calculated angular velocity and real one
            and form the control signal for wheels"""
        om = om * 180 / math.pi
        g = self.gyro
        if math.fabs(math.fabs(g) - math.fabs(om)) > 1:
            if g > 0 and om < 0:
                self.om_left = 10
                self.om_right -= 2
            elif g < 0 and om > 0:
                self.om_left -= 2
                self.om_right = 10
            elif g < 0 and om < 0:
                if g < om:
                    self.om_left -= 2
                    self.om_right = 10
                else:
                    self.om_left = 10
                    self.om_right -= 2
            elif g > 0 and om > 0:
                if g > om:
                    self.om_left = 10
                    self.om_right -= 2
                else:
                    self.om_left -= 2
                    self.om_right = 10
        else:
            self.om_left = 10
            self.om_right = 10
        self.set_speed(self.om_left, self.om_right)

    def control_OM(self, ang_vel, dt, t1):
        """obstacles avoidance based on angular velocity control"""
        print("OM CONTROL")
        for i in range(ang_vel.shape[0]):
            t1 = time.time()
            t2 = time.time()
            while t2 - t1 < dt:
                self.compare_OM_w_gyro(ang_vel[i])
                self.move_forward(ang_vel[i])
                t2 = time.time()

    def avoid_obs_control(self):
        """obstacles avoidance algorythm"""
        time2 = time.time()
        if time2 - self.time1 < 1:
            # collecting data and considering static obstacles
            self.set_speed(10, 10)
            if time2 - self.time11 < 0.01:
                self.T = RotoTranslationalM(0.14, 0, time2 - self.time11)
                self.linedata = regular_iterating(self.scan, self.linedata, self.T)
                self.time11 = time.time()
                self.iter += 1
        else:
            self.flag = False
            side, trajectory_params, y, time_coll = return_traject_data(self.linedata)
            rospy.sleep(time_coll)
            l = trajectori_length(trajectory_params, y)
            full_time = round(l / 0.1)
            n = int((max(y) - min(y)) / 0.05)
            y_add = np.linspace(min(y), max(y), n) # devision y on 5 sm
            alpha = -np.arctan(
                3 * trajectory_params[0] * y_add ** 2 + 2 * trajectory_params[1] * y_add + trajectory_params[
                    2])
            al = -np.arctan(
                3 * trajectory_params[0] * y_add ** 2 + 2 * trajectory_params[1] * y_add + trajectory_params[
                    2]) * 180 / math.pi
            OM = np.array([])
            dt = full_time / n
            for i in range(1, len(alpha)):
                OM = np.append(OM, np.array([(alpha[i] - alpha[i - 1]) / dt]))
            ind = []
            for i in range(1, OM.shape[0]):
                if OM[i] > 0 and OM[i - 1] < 0 or OM[i] < 0 and OM[i - 1] > 0:
                    ind.append(i - 1)
            ind.append(len(OM) - 1)
            om = OM
            t1 = time.time()
            # self.control_OM(om, dt, t1)             # for controlling by gyro
            # for controlling by wheel angular velocities
            for i in ind:
                if om[i] > 0:
                    vel, om_l = robot.left_rotation(OM[:i])
                    l = trajectori_length(trajectory_params, y_add[:i])
                    t = l / vel
                    om10 = robot.convert_om_to10(om_l)
                    OM = OM[i + 1:]
                    om_l, time_for_move, dt = prepare_om_t(t, om10)
                    y_add = y_add[i + 1:]
                    self.control_wheel(om_l, time_for_move, 'left')
                elif om[i] < 0:
                    vel, om_r = robot.right_rotation(OM[:i])
                    l = trajectori_length(trajectory_params, y_add[:i])
                    t = l / vel
                    om10 = robot.convert_om_to10(om_r)
                    om_r, time_for_move, dt = prepare_om_t(t, om10)
                    OM = OM[i + 1:]
                    y_add = y_add[i + 1:]
                    self.control_wheel(om_r, time_for_move, 'right')
                print("END OF MOVE")
                rospy.sleep(10)


if __name__ == '__main__':
    rospy.init_node('avoid_obstacles', anonymous=True)
    # files generated by initpos.py
    f1 = "/home/ubuntu/catkin_ws/src/tech_vision/data/" + str(0) + ".txt"
    f2 = "/home/ubuntu/catkin_ws/src/tech_vision/data/" + str(1) + ".txt"
    global scan1, scan2, ttt
    ttt = 2  # time from initpos.py
    with open(f1, 'r') as f1:
        scan1 = f1.read()
    with open(f2, 'r') as f2:
        scan2 = f2.read()
    print("go")
    robot = MobilePlatform(0, 0, 0, 0)
    obc = ControlServo()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
