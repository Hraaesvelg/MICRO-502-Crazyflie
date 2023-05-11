# You can change anything in this file except the file name of 'my_control.py',
# the class name of 'MyController', and the method name of 'step_control'.

# Available sensor data includes data['t'], data['x_global'], data['y_global'],
# data['roll'], data['pitch'], data['yaw'], data['v_forward'], data['v_left'],
# data['range_front'], data['range_left'], data['range_back'],
# data['range_right'], data['range_down'], data['yaw_rate'].

from locale import locale_encoding_alias
# from symbol import return_stmt
import numpy as np

import math
import helpers as hlp

pos_x = [3.60, 4.80, 4.80, 3.60, 3.60, 4.80, 4.80, 3.60, 3.60, 4.80, 4.80, 3.60, 3.60, 4.80, 4.80, 3.60, 3.60, 4.80, 4.80,
      3.60]
pos_x = [0.10, 0.10, 0.40, 0.40, 0.70, 0.70, 1.00, 1.00, 1.30, 1.30, 1.60, 1.60, 1.90, 1.90, 2.20, 2.20, 2.50, 2.50, 2.80,
      2.80]
x_end = 0
y_end = 0

min_x, max_x = 0, 5.0  # meter
min_y, max_y = 0, 3.0  # meter
start_lim, stop_lim = 1.5, 3.5  # meter

range_max = 2.0  # meter, maximum range of distance sensor
res_pos = 0.  # meter

# Don't change the class name of 'MyController'
class MyController():
    def __init__(self):
        self.on_ground = True
        self.height_desired = 0.5
        self.state = 'explore'
        self.index = 0
        self.return_point = None

    # Don't change the method name of 'step_control'
    def step_control(self, sensor_data):
        global pos_x, pos_y, x_end, y_end
        # Take off
        if self.on_ground and sensor_data['range_down'] < 0.49:  # and
            return self.take_off(sensor_data)
        # Land
        else:
            self.on_ground = False
            zone = self.determine_area(sensor_data['x_global'], sensor_data['y_global'])
            if (self.state == 'explore'):
                if (self.determine_area(pos_x[19], pos_y[19]) == zone):
                    precision_yaw = 0.01
                    precision_dist = 0.01
                    speed = 0.1
                    if self.height_desired > 0.252:
                        self.height_desired = self.height_desired - 0.002
                    if self.height_desired < 0.148:
                        self.height_desired = self.height_desired + 0.002
                else:
                    precision_yaw = 0.1
                    precision_dist = 0.4
                    speed = 0.3
                    self.height_desired = 0.5
                objx = pos_x[self.index]
                objy = pos_y[self.index]
                # bordure en traversée, recentrage
                if (zone == 'BORDER') or ((zone == 'OUT') and (sensor_data['x_global'] < stop_lim) and (
                        sensor_data['x_global'] > start_lim)):
                    if pos_y[0] != 1.5:
                        pos_x.insert(0, stop_lim)
                        pos_y.insert(0, 1.5)
                # detection sol (base)
                if (sensor_data['range_down'] < self.height_desired - 0.075) and (
                        self.determine_area(pos_x[19], pos_y[19]) == zone) and (
                        (math.cos(sensor_data['yaw']) > 0.9) or (math.cos(sensor_data['yaw']) < -0.9) or (
                        math.sin(sensor_data['yaw']) > 0.9) or (math.sin(sensor_data['yaw']) < -0.9)):
                    # points from base saved
                    x_end = sensor_data['x_global']
                    y_end = sensor_data['y_global']
                    self.height_desired = self.height_desired - 0.1
                    print('xe1 =', x_end)
                yaw_obj = hlp.direction(sensor_data['x_global'], sensor_data['y_global'], objx, objy)
                yaw_command = 2 * hlp.reach_yaw(yaw_obj, sensor_data['yaw'])
                control_command = [0., 0.0, yaw_command, self.height_desired]
                if yaw_command ** 2 <= precision_yaw ** 2:
                    advance = speed
                    control_command[0] = advance

                # When the drone reaches the goal setpoint, next or stop
                if hlp.distance(sensor_data['x_global'], sensor_data['y_global'], pos_x[self.index], pos_y[self.index]) < precision_dist:
                    self.index += 1
                elif hlp.distance(sensor_data['x_global'], sensor_data['y_global'], x_end, y_end) < precision_dist:
                    self.state = 'landing'

            elif (self.state == 'landing'):
                control_command = self.landing_procedure(sensor_data)

            elif (self.state == 'goes_back'):
                if ('START' == zone):
                    precision_yaw = 0.01
                    precision_dist = 0.01
                    speed = 0.1
                    if self.height_desired > 0.252:
                        self.height_desired = self.height_desired - 0.002
                else:
                    precision_yaw = 0.1
                    precision_dist = 0.4
                    speed = 0.3
                    self.height_desired = 0.5

                control_command = [0., 0., 0, self.height_desired]
                if sensor_data['range_down'] < 0.49:
                    self.height_desired += 0.05
                else:
                    if (zone == 'START'):
                        precision_yaw = 0.005
                        precision_dist = 0.01
                        speed = 0.1
                        if self.height_desired > 0.251:
                            self.height_desired = self.height_desired - 0.002
                    else:
                        precision_yaw = 0.2
                        precision_dist = 0.3
                        speed = 0.3
                        self.height_desired = 0.5

                # follow set_point
                yaw_obj = hlp.direction(sensor_data['x_global'], sensor_data['y_global'], self.return_point[0], self.return_point[1])
                yaw_command = 2 * hlp.reach_yaw(yaw_obj, sensor_data['yaw'])
                control_command = [0., 0.0, yaw_command, self.height_desired]
                # if aligned, go
                if yaw_command ** 2 <= precision_yaw ** 2:
                    advance = speed
                    control_command[0] = advance

                # When the drone reaches the goal setpoint, next or stop
                if hlp.distance(sensor_data['x_global'], sensor_data['y_global'], self.return_point[0],
                            self.return_point[1]) < precision_dist:
                    self.state = 'landing'

            elif (self.state == 'STOP'):
                control_command = [0., 0., 0, self.height_desired]

            # Obstacle avoidance with distance sensors
            if sensor_data['range_front'] < 0.2 or sensor_data['range_left'] < 0.1 or sensor_data[
                'range_right'] < 0.1 or sensor_data['range_back'] < 0.1:
                if (self.determine_area(pos_x[self.index], pos_y[self.index]) == zone):
                    pos_x[self.index] = pos_x[self.index - 1]
                    pos_y[self.index] = pos_y[self.index - 1]
                else:
                    control_command = self.obstacle_avoidance(sensor_data)

        return control_command

    def determine_area(self, x, y):
        zone = 'OUT'
        if y < max_y - res_pos and y > min_y + res_pos:
            if x < max_x - res_pos and x > min_x + res_pos:
                zone = 'IN'
                if x > stop_lim + res_pos:
                    zone = 'LANDING'
                elif x < start_lim + res_pos:
                    zone = 'START'
                elif y < 0.30 or y > 2.70:
                    zone = 'BORDER'
        return zone

    def take_off(self, sensor_data):
        self.height_desired = 0.5
        control_command = [0.0, 0.0, 0., self.height_desired]
        if (self.determine_area(sensor_data['x_global'], sensor_data['y_global']) == 'START'):
            self.return_point = [sensor_data['x_global'], sensor_data['y_global']]
            self.state = 'explore'
        else:
            self.state = 'goes_back'
        return control_command

    def landing_procedure(self, sensor_data):
        print('start landing')
        self.height_desired -= 0.001
        control_command = [0., 0., 0, self.height_desired]
        if self.height_desired < 0.002:
            print('landed')
            self.state = 'goes_back'
            # self.on_ground=True
        return control_command

    def obstacle_avoidance(self, sensor_data):
        if sensor_data['range_left'] > 0.1:
            left_bounded = 0.1
        else:
            left_bounded = sensor_data['range_left']
        if sensor_data['range_right'] > 0.1:
            right_bounded = 0.1
        else:
            right_bounded = sensor_data['range_right']
        if sensor_data['range_front'] > 0.2:
            front_bounded = 0.2
        else:
            front_bounded = sensor_data['range_front']
        if sensor_data['range_back'] > 0.1:
            back_bounded = 0.1
        else:
            back_bounded = sensor_data['range_back']
        gain_front = 1.2 * (front_bounded + back_bounded)
        gain_lat = 1.2 * (left_bounded - right_bounded)
        if sensor_data['range_left'] > sensor_data['range_right'] + 0.05:
            control_command = [-gain_front, gain_lat + 0.5 * gain_front, -1. * gain_front,
                               self.height_desired]
        else:
            control_command = [-gain_front, gain_lat - 0.5 * gain_front, 1. * gain_front,
                               self.height_desired]
        return control_command