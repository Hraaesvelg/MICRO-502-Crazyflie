import numpy as np
import math


def chang_yaw_range(angle):
    return angle + np.pi


def angle_between_points(point1, point2):
    x1, y1 = point1
    x2, y2 = point2
    dx = x2 - x1
    dy = y2 - y1
    return math.atan2(dy, dx)

def distance_between_points(point1, point2):
    x1, y1 = point1
    x2, y2 = point2
    dx = x2 - x1
    dy = y2 - y1
    return math.sqrt(dx ** 2 + dy ** 2)


def direction(x, y, x_obj, y_obj):
    return math.atan2(y_obj - y, x_obj - x)


def distance(x, y, x_obj, y_obj):
    return math.sqrt((y_obj - y) ** 2 + (x_obj - x) ** 2)


def reach_yaw(desired_yaw, yaw):
    yaw_command = (desired_yaw - yaw)
    if yaw_command > math.pi:
        yaw_command = yaw_command - 2 * math.pi
    elif yaw_command < -math.pi:
        yaw_command = yaw_command + 2 * math.pi
    return yaw_command * 0.5



