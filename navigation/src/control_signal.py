#!/usr/bin/python
# -*-coding: utf-8 -*-
import math
import numpy as np
from Kinematics import MobilePlatform


def trajectory_length(params, coord):
    if params.shape[0] == 3:
        poly = np.zeros((3,))
        poly[0] = (2 * params[0]) ** 2
        poly[1] = 4 * params[0] * params[1]
        poly[2] = params[1] ** 2 + 1
    elif params.shape[0] > 3:
        poly = np.zeros((5,))
        poly[0] = (3 * params[0]) ** 2
        poly[1] = 12 * params[0] * params[1]
        poly[2] = (2 * params[1]) ** 2 + 6 * params[0] * params[2]
        poly[3] = 4 * params[1] * params[2]
        poly[4] = params[2] ** 2 + 1
    int = np.polyint(poly)
    lenght = np.polyval(int, np.max(coord)) - np.polyval(int, np.min(coord))
    return round(lenght, 2)


def prepare_om_t(t, om):
    OM = []
    dt = np.sum(t) / t.shape[0] ** 2
    t = 0
    T = [0]
    for i in range(1, om.shape[0]):
        t += dt
        if om[i - 1] != om[i]:
            OM.append(om[i - 1])
            T.append(t)
    OM.append(om[-1])
    T.append(t)
    return OM, T, dt


def params3(robot, params, y):
    l = trajectory_length(params, y)
    full_time = round(l / 0.11)  # division on average velocity defined experimentally
    n = int((max(y) - min(y)) / 0.05)  # devision y on 5 sm
    y_add = np.linspace(min(y), max(y), n)
    alpha = -np.arctan(2 * params[0] * y_add + params[1])
    OM = np.array([])
    dt = full_time / n
    for i in range(1, len(alpha)):
        OM = np.append(OM, np.array([(alpha[i] - alpha[i - 1]) / dt]))
    print("angular vel ", OM)
    if OM[1] > 0:
        vel, om_l = robot.left_rotation(OM)
        l = trajectory_length(params, y)
        t = l / vel
        om10 = robot.convert_om_to10(om_l)
        om_wheel, time, dt = prepare_om_t(np.array([t]).T, om10)
    elif OM[1] < 0:
        vel, om_r = robot.right_rotation(OM)
        l = trajectory_length(params, y)
        t = l / vel
        om10 = robot.convert_om_to10(om_r)
        om_wheel, time, dt = prepare_om_t(np.array([t]).T, om10)

    return om_wheel, time, OM, [dt]


def params3plus(robot, params, y):
    l = trajectory_length(params, y)
    full_time = round(l / robot.FULL_SPEED)
    n = int((max(y) - min(y)) / 0.05)  # devision y on 5 sm
    y_add = np.linspace(min(y), max(y), n)
    alpha = -np.arctan(3 * params[0] * y_add ** 2 + 2 * params[1] * y_add + params[
        2])
    al = -np.arctan(3 * params[0] * y_add ** 2 + 2 * params[1] * y_add + params[
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
    om_wheel = []
    t_general = []
    dt_general = []
    for i in ind:
        if om[i] > 0:
            vel, om_l = robot.left_rotation(OM[:i])
            l = trajectori_length(params, y_add[:i])
            t = l / vel
            om10 = robot.convert_om_to10(om_l)
            OM = OM[i + 1:]
            om_l, time, dt = prepare_om_t(t, om10)
            y_add = y_add[i + 1:]
            om_wheel.append(om_l)

        elif om[i] < 0:
            vel, om_r = robot.right_rotation(OM[:i])
            l = trajectori_length(params, y_add[:i])
            t = l / vel
            om10 = robot.convert_om_to10(om_r)
            om_l, time, dt = prepare_om_t(t, om10)
            OM = OM[i + 1:]
            y_add = y_add[i + 1:]

    return om_wheel, t_general, om, dt_general


def return_data(robot, params, y):
    if params.shape[0] == 3:
        om_wheel, time, OM, dt = params3(robot, params, y)
    elif params.shape[0] > 3:
        om_wheel, time, OM, dt = params3plus(robot, params, y)
    return om_wheel, time, OM, dt
