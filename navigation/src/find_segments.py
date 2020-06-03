#!/usr/bin/python
# -*-coding: utf-8 -*-
import math
import matplotlib.pyplot as plt
import numpy as np


def create_numpy_data(data, angle_min, angle_step):
    """"
        Convert raw data to numpy structure shaped (n,2) like
        [phi0][r0]
        [phi1][r1]
        [...][..]
        """
    work_data = np.zeros((len(data), 2))
    angle = angle_min
    mask = []
    for i in range(len(data)):
        if data[i] != 'inf' and data[i] != 'nan':
            work_data[i][0] = angle
            work_data[i][1] = float(data[i])
        angle += angle_step
    for i in range(work_data.shape[0]):
        if work_data[i][0] != 0 and work_data[i][1] != 0:
            mask.append(i)
    return work_data[mask]


def parse_data(f):
    """unpacking LaserScan package"""
    data = []
    # for forking with txt data scans
    #with open(f, "r") as f:
    f = str(f)
    origin_data = f.split()
    i = 0
    while i in range(len(origin_data)):
        if origin_data[i] == 'angle_min:':
            # rotate on 90 degree
            ang_min = float(origin_data[i + 1]) - math.pi / 2
            i = i + 1
        elif origin_data[i] == 'angle_max:':
            ang_max = float(origin_data[i + 1])
            i = i + 1
        elif origin_data[i] == 'angle_increment:':
            angle_step = float(origin_data[i + 1])
            i = i + 1
        elif origin_data[i] == 'ranges:':
            origin_data[i + 1] = origin_data[i + 1][1:]
            while origin_data[i + 1] != 'intensities:':
                data.append(origin_data[i + 1][:-1])
                i = i + 1
            break
        i = i + 1
    work_data = create_numpy_data(data, ang_min, angle_step)
    return work_data, angle_step


def find_segments(data, NumPoints):
    """segments recognition based on Dietmayer"""
    rad = data[:, 1]
    angle = data[:, 0]
    segments = []
    previousI = -1
    for i in range(0, rad.shape[0] - 1):
        D = math.sqrt(rad[i] ** 2 + rad[i + 1] ** 2 - 2 * rad[i] * rad[i + 1] * math.cos(
            math.fabs(angle[i] - angle[i + 1])))
        C0 = 0.05  # Dietmayer constant for noise reduction
        C1 = math.sqrt(2 - 2 * math.cos(math.fabs(angle[i] - angle[i + 1])))
        tr = C0 + C1 * min(rad[i], rad[i + 1])
        if D > tr:
            if data[previousI + 1:i].size >= NumPoints - 1:
                segments.append(data[previousI + 1:i])
            previousI = i
    return segments


def plot_line(m, q, x, col, ax):
    """plot lines of segments, usage in for with plt.show() in the end
     for seg in segments:
        color = list(np.random.choice(range(256), size=3) / 255)
        x,y = polar_to_decart(seg)
        a,b,c, m, q = line_param(x,y)
        plot_line(a,b,c,x,y, color)
    plt.show()
    """
    minX = min(x)
    maxX = max(x)
    xc = np.linspace(minX, maxX, 10)
    yc = m * xc + q
    ax.plot(xc, yc, color=col)


def plot_segments(segments, ax):
    for element in segments:
        if element.shape[0] >= 2:
            col = list(np.random.choice(range(256), size=3) / 255)
            ax.plot(element[:, 1] * np.cos(element[:, 0]),
                    element[:, 1] * np.sin(element[:, 0]), 'o', color=col, markersize=1)
    # plt.show()


def polar_to_decart(data):
    """convert polar coordinate to Descartes one """
    x = data[:, 1] * np.cos(data[:, 0]) + 0.2
    y = data[:, 1] * np.sin(data[:, 0])
    x = np.reshape(x, (x.size, 1))
    y = np.reshape(y, (y.size, 1))
    return x, y


def line_param(x, y, representation):
    """find line params considering segment points"""
    B = np.concatenate((x.T, y.T, (np.ones((x.size, 1))).T))
    u, s, vh = np.linalg.svd(B.T)
    vh = vh.transpose()
    last_col = vh.shape[1] - 1
    # general lines params
    a, b, c = vh[:, last_col]
    # common lines params
    m = -a / b
    q = -c / b
    if representation == "canonical":
        return a, b, c
    elif representation == "parametric":
        return m, q


def apply_find_seg(f, ax):
    """quick application of lines and segments recognition"""
    data, angle_step = parse_data(f)
    plot_segments([data], ax)
    segments = find_segments(data, 2)
    plot_segments(segments, ax)
    for seg in segments:
        color = list(np.random.choice(range(256), size=3) / 255)
        x, y = polar_to_decart(seg)
        m, q = line_param(x, y, "parametric")
        plot_line(m, q, x, color, ax)
    plt.show()



if __name__ == '__main__':
    fig, ax = plt.subplots()
    #apply_find_seg('200.txt', ax)