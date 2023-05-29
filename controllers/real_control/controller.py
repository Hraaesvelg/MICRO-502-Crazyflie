import numpy as np
import math
import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.utils import uri_helper


class Conroller():
    def __init__(self):
        # Grid points for grid search
        self.px = 0
        self.py = 0
        self.index = 0

        # Initialize the landing zone
        self.landing = np.array([[0., 0.], [0., 0.]])
        self.height_desired = 0.2
        self.previous = 0

        # Initialize the speed of motors
        self.speedx = 0.
        self.speedy = 0.

        # Initialize the zones and states of the FSM
        self.zone = None
        self.state = None
        self.target_found = None

        # Initialize the Logger
        self.lg_stab = None

        # Initiate all the modules at the correct value
        self.init_fsm()

    def init_logger(self):
        # LOG ENTRIES
        lg_stab = LogConfig(name='Stabilizer', period_in_ms=500)
        lg_stab.add_variable('stateEstimate.x', 'float')
        lg_stab.add_variable('stateEstimate.y', 'float')
        lg_stab.add_variable('range.front')
        lg_stab.add_variable('range.back')
        lg_stab.add_variable('range.left')
        lg_stab.add_variable('range.right')
        lg_stab.add_variable('kalman.stateZ')
        self.lg_stab = lg_stab
        return lg_stab
    def get_grid_point(self):
        """
        :return: The list of check point and the index of the table initialized at 0
        """
        ## Grid points: These points are used to make the grid search for the landing pad
        point_x = [1.85, 1.85, 2.35, 2.35, 2.8, 2.8, 3.2, 3.2, 0.0]
        point_y = [-0.4, 1.0, 1.0, -0.4, -0.4, 1.0, 1.0, -0.4, 0.0]
        idx = 0
        self.px = point_x
        self.py = point_y
        self.index = idx
        return True

    def init_fsm(self):
        """
        State of the Finite state machine
        Different States
        #state = ['TAKE_OFF', 'LAND', 'SEARCH', 'GO_BACK', 'STOP', 'AVOID', 'ARRIVED']
        Different Zones
        #zone = ['START', 'EXPLO', 'FINAL', 'OUT', 'GOAL']
        """
        self.state = 'TAKE_OFF'  # Initially we want  to take off
        self.zone = 'START'  # Initially we are at start
        self.target_found = False
        return True

    def check_zone(self, x, y):
        """
        Check in which zone is the drone between starting, landing and exploration zone.
        Return 'BORDER' if outside the delimitation
        :param x: The x position of the drone from estimator
        :param y: The y position of the drone from estimator
        :return: The zone in which is the drone
        """
        zone = 'OUT'
        if y < 3 and y > -1:  # beware of int_float comparison ?
            if x < 5 and x > 0:
                zone = 'IN'
                if x > 3.5:
                    zone = 'LANDING'
                elif x < 1.5:
                    zone = 'START'
                elif y < 0.30 or y > 2.70:
                    zone = 'BORDER'
        return zone