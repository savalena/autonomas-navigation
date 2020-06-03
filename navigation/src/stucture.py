#!/usr/bin/python
# -*-coding: utf-8 -*-
import numpy as np
import find_segments
import math


class Lines:
    def __init__(self, a, b, xmax, xmin):
        self.xmin = xmin
        self.xmax = xmax
        self.a = a
        self.b = b
        self.x, self.y = self.create_coords()
        self.homo_xy = self.homo_coords()

    def homo_coords(self):
        """Create homo coordinates from Descartes one"""
        x = self.x
        y = self.y
        zeros = np.zeros(x.shape)
        ones = np.ones(x.shape)
        homo = np.concatenate((x, y, zeros, ones), axis=1)
        return homo.T

    def create_coords(self, step=5):
        """create coordinates from line equation"""
        xmax = 100 * self.xmax
        xmin = 100 * self.xmin
        x = np.linspace(xmax, xmin, step)
        x = np.reshape(x, (x.size, 1))
        x = x / 100
        y = self.a * x + self.b
        return x, y


class LinesData:
    def __init__(self, lines, old_lines, lines_predicted):
        self.lines = self.lines_to_np(lines)
        self.old_lines = self.lines_to_np(old_lines)
        self.current_lines = np.zeros((self.old_lines.shape[0], 4))
        self.lines_predicted = self.lines_to_np(lines_predicted)
        self.archived = self.old_lines
        self.error = []
        self.ind_changed = set()

    def lines_to_np(self, lines):
        """convert input list in np.array
                  output stucture: [a0, b0, xmin0, xmax0]
                                   [.., ..,  ...,   ...]
                                   [an, bn, xminn, xmaxn]"""
        nplines = np.array([lines[0].a, lines[0].b, float(lines[0].xmin), float(lines[0].xmax)]).reshape(1, 4)
        for i in range(1, len(lines)):
            tmp = np.array([lines[i].a, lines[i].b, float(lines[i].xmin), float(lines[i].xmax)]).reshape(1, 4)
            nplines = np.concatenate((nplines, tmp), axis=0)
        return nplines

    def compare_lines(self):
        """compare array of old_lines with lines
           add tracked and new lines to current_lines"""
        const = 2
        indi = []
        for line in self.lines:
            MIN = 10
            min_old = 10
            for i in range(self.lines_predicted.shape[0]):
                min = np.linalg.norm(line - self.lines_predicted[i], ord=1)
                min_old = np.linalg.norm(line - self.old_lines[i], ord=1)
                if MIN > min or MIN > min_old:
                    MIN = min
                    ind = i
                    indi.append(ind)
            if MIN < const:
                self._replace_line(ind, line)
                self.error.append(MIN)
            else:
                #
                self._add_new_line(line)
        self._delete_line(indi)

    def _delete_line(self, indi):
        """delete a row considering index"""
        for i in range(self.current_lines.shape[0]):
            if i not in indi:
                self.current_lines[i] = np.zeros((1, 4))
            else:
                self.ind_changed.add(i)

    def _replace_line(self, old_lines_ind, line):
        """Replace old line position on new one"""
        if line.reshape((1, 4)) not in self.old_lines:  # возможно не нужно
            self.current_lines[old_lines_ind] = line

    def _add_new_line(self, line):
        """Add new lines which were not found in old_lines"""
        if line.reshape((1, 4)) not in self.old_lines:
            self.current_lines = np.concatenate((self.current_lines, line.reshape(1, 4)), axis=0)

    def add_lines(self, lines):
        """Add new measurement"""
        self.lines = self.lines_to_np(lines)

    def add_predicted_lines(self, predicted_lines):
        """Add predicted lines"""
        self.lines_predicted = self.lines_to_np(predicted_lines)

    def archiev_lines(self):
        """LOGS for lines"""
        self.archived = np.concatenate((self.archived, self.current_lines), axis=0)
        self.old_lines = self.current_lines


def RotoTranslationalM(V, alpha, dt):
    """Roto translational matrix creation"""
    al = alpha * math.pi / 180
    T = np.array([[math.cos(al), -math.sin(al), 0, 0],
                  [math.sin(al), math.cos(al), 0, -V * dt],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])
    return T


def initiate_pos(f1, f2, T):
    """parse inital data"""
    lines = []
    data, angle_step = find_segments.parse_data(f2)
    segments = find_segments.find_segments(data, 3)
    for seg in segments:
        x, y = find_segments.polar_to_decart(seg)
        m, q = find_segments.line_param(x, y, "parametric")
        if m > 100:  # for vertical lines
            m = (max(y) - min(y)) / (max(x) - min(x))
            q = max(y) - m * max(x)
        lines.append(Lines(m, q, max(x), min(x)))

    old_lines = []
    data, angle_step = find_segments.parse_data(f1)
    segments = find_segments.find_segments(data, 3)
    for seg in segments:
        x, y = find_segments.polar_to_decart(seg)
        m, q = find_segments.line_param(x, y, "parametric")
        if m > 100:  # for vertical lines
            m = (max(y) - min(y)) / (max(x) - min(x))
            q = max(y) - m * max(x)
        old_lines.append(Lines(m, q, max(x), min(x)))

    predicted_lines = []

    # homogeneous transformation
    for i in range(len(old_lines)):
        homo = old_lines[i].homo_xy
        predicted_line = np.matmul(T, homo)
        x = predicted_line[0, :]
        x = np.reshape(x, (x.size, 1))
        y = predicted_line[1, :]
        y = np.reshape(y, (x.size, 1))
        a, b = find_segments.line_param(x, y, "parametric")
        predicted_lines.append(Lines(a, b, max(x), min(x)))

    linedata = LinesData(lines, old_lines, predicted_lines)
    linedata.compare_lines()

    return linedata


def collect_data():
    T = RotoTranslationalM(0.135, -0.35, 1)
    LD = initiate_pos('30.txt', '60.txt', T)
    return LD

def regular_iterating(scan, LD, T):
    """collecting data"""
    LD.archiev_lines()
    predicted_lines = []
    for l in LD.old_lines:
        line = Lines(l[0], l[1], l[2], l[3])
        homo = line.homo_xy
        predicted_line = np.matmul(T, homo)
        x = predicted_line[0, :]
        x = np.reshape(x, (x.size, 1))
        y = predicted_line[1, :]
        y = np.reshape(y, (x.size, 1))
        a, b = find_segments.line_param(x, y, "parametric")
        predicted_lines.append(Lines(a, b, max(x), min(x)))

    lines = []
    data, angle_step = find_segments.parse_data(scan)
    segments = find_segments.find_segments(data, 20)
    for seg in segments:
        x, y = find_segments.polar_to_decart(seg)
        m, q = find_segments.line_param(x, y, "parametric")
        if m > 50:
            m = (max(y) - min(y)) / (max(x) - min(x))
            q = max(y) - m * max(x)
        lines.append(Lines(m, q, max(x), min(x)))

    LD.add_lines(lines)
    LD.add_predicted_lines(predicted_lines)

    LD.compare_lines()

    return LD


def static_obs(lines, ind):
    """find static considering index of unchanged LinesData elements"""
    ind = list(ind)
    lines = lines[ind]
    current_lines = np.array([[0, 0, 0, 0]])
    for l in lines:
        if np.linalg.norm(l - [[0, 0, 0, 0]], ord=1) != 0:
            current_lines = np.append(current_lines, l.reshape(1, 4), axis=0)
    current_lines = np.delete(current_lines, 0, axis=0)
    return current_lines


def form_extreme_points(first_lines):
    """Add max у and min y to line params"""
    current_lines = first_lines
    Y = np.array([[0, 0]])
    for i in range(current_lines.shape[0]):
        ymax = current_lines[i][0] * current_lines[i][3] + current_lines[i][1]
        ymin = current_lines[i][0] * current_lines[i][2] + current_lines[i][1]
        ya = np.array([ymax, ymin]).reshape((1, 2))
        Y = np.append(Y, ya, axis=0)
    current_lines = np.concatenate((current_lines, Y[1:]), axis=1)
    current_lines = current_lines[current_lines[:, 4].argsort()]
    return current_lines


def group_obs(ungrouped):
    """groupe obstacles considering if they could be avoide"""
    ind = []
    grouped = []
    for i in range(ungrouped.shape[0]):
        if ungrouped[i][4] < 2.0 and ungrouped[i][2] < 2.0 and ungrouped[i][3] > -2.0:
            ind.append(i)
    ws_ungrouped = ungrouped[ind]
    mins_previous = []
    for line in ws_ungrouped:
        mins = []
        for i in range(ws_ungrouped.shape[0]):
            min = np.linalg.norm(line[2:5] - ws_ungrouped[i][2:5], ord=1)
            if min < 3.0:
                mins.append(i)
        if mins not in mins_previous:
            grouped.append(ws_ungrouped[mins])
            mins_previous = [mins]
    return grouped


def choose_side(group):
    """choose side of avoiding"""
    sides = []
    for gr in group:
        x_posmax = np.amax(gr[:, 3])
        x_posmin = np.amin(gr[:, 3])
        x_negmax = np.amax(gr[:, 2])
        x_negmin = np.amin(gr[:, 2])
        x_pos = max(x_posmax, x_posmin)
        x_neg = max(x_negmax, x_negmin)
        if math.fabs(x_pos) - math.fabs(x_neg) > 0:
            sides.append("left")
        else:
            sides.append("right")
    return sides


def create_route(group, side):
    """Find points od route considering extreme points for avoiding"""
    points = np.array([[0, 0]])
    for i in range(len(group)):
        if side[i] == 'left':
            edge_points_x = group[i][:, 2]
            edge_points_y = group[i][:, 4]
            f = 1
            for k in range(len(edge_points_x)):
                if math.fabs(edge_points_x[k]) < 0.3:
                    f = 0
                    points = np.append(points, np.array([[edge_points_x[k] - 0.2, edge_points_y[k] - 0.4]]), axis=0)
                    points = np.append(points, np.array([[edge_points_x[k] - 0.2, edge_points_y[k] + 0.2]]), axis=0)
            if f == 1:
                points = np.append(points, np.array([[min(edge_points_x) - 0.2, edge_points_y[0]]]), axis=0)
        elif side[i] == 'right':
            edge_points_x = group[i][:, 3]
            edge_points_y = group[i][:, 5]
            f = 1
            for k in range(len(edge_points_x)):
                if math.fabs(edge_points_x[k]) < 0.3:
                    f = 0
                    points = np.append(points, np.array([[edge_points_x[k], edge_points_y[k] - 0.2]]), axis=0)
                    points = np.append(points, np.array([[edge_points_x[k] + 0.2, edge_points_y[k] + 0.2]]), axis=0)
            if f == 1:
                points = np.append(points, np.array([[max(edge_points_x) + [0.2], edge_points_y[0]]]), axis=0)
    return points


def calculate_explosion(extreme_points, t, V=0.14):
    """predict time of collision"""
    dist = extreme_points[0, 5]
    time = dist / V
    return time - t


def prediction(lines, T, linedata):
    """predict lines position in t"""
    predicted_lines = []
    for l in lines:
        line = Lines(l[0], l[1], l[2], l[3])
        homo = line.homo_xy
        predicted_line = np.matmul(T, homo)
        x = predicted_line[0, :]
        x = np.reshape(x, (x.size, 1))
        y = predicted_line[1, :]
        y = np.reshape(y, (x.size, 1))
        a, b = find_segments.line_param(x, y, "parametric")
        predicted_lines.append(Lines(a, b, max(x), min(x)))
    return linedata.lines_to_np(predicted_lines)


def interpolation(points, n):
    """interpolate on route points"""
    deg = 2
    if n > 1:
        deg = 3
    x = points[:, 0]
    y = points[:, 1]
    params = np.polyfit(y, x, deg)
    return params


def plot_interpolation(params, points):
    """plot interpolation line and route points"""
    y_inter = np.linspace(min(points[:, 1]), max(points[:, 1]),
                          10)
    x_inter = np.polyval(params, y_inter)
    plt.plot(x_inter, y_inter, 'grey')
    for p in points:
        plt.plot(p[0], p[1], '-x')
    return y_inter


def return_traject_data(LD):
    """quick implementation"""
    cur_l = static_obs(LD.lines, LD.ind_changed)
    extreme_points = form_extreme_points(cur_l)
    print("EP", extreme_points)
    time = calculate_explosion(extreme_points, 2)
    T = RotoTranslationalM(0.14, 0, time)
    predicts = prediction(cur_l, T, LD)

    extreme_points = form_extreme_points(predicts)
    group = group_obs(extreme_points)
    side = choose_side(group)
    points_of_route = create_route(group, side)
    params = interpolation(points_of_route, len(group))
    y = np.linspace(min(points_of_route[:, 1]), max(points_of_route[:, 1]),
                    10)
    return side, params, y, time
