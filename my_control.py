# You can change anything in this file except the file name of 'my_control.py',
# the class name of 'MyController', and the method name of 'step_control'.

# Available sensor data includes data['t'], data['x_global'], data['y_global'],
# data['roll'], data['pitch'], data['yaw'], data['v_forward'], data['v_left'],
# data['range_front'], data['range_left'], data['range_back'],
# data['range_right'], data['range_down'], data['yaw_rate'].

from locale import locale_encoding_alias
import numpy as np
import math

posx=[3.65, 4.85,4.85,3.65, 3.65, 4.85,4.85,3.65,3.65, 4.85,4.85,3.65,3.65, 4.85,4.85,3.65,3.65, 4.85,4.85,3.65,]
posy=[0.15, 0.15, 0.45, 0.45, 0.75, 0.75, 1.05, 1.05, 1.35, 1.35, 1.65, 1.65, 1.95, 1.95, 2.25, 2.25, 2.55, 2.55, 2.85, 2.85]
x_end = 0
y_end = 0

min_x, max_x = 0, 5.0 # meters
min_y, max_y = 0, 3.0 # meters
start_l, stop_l = 1.5, 3.5 #meters

range_max = 2.0 # meters, max range of distance sensor
res_pos = 0. # meters
conf = 0.2 
t = 0 


# Don't change the class name of 'MyController'
class MyController():
    def __init__(self):
        global state
        self.on_ground = True
        self.height_desired = 0.5
        state ='search'
        global idx
        idx=0
        

    # Don't change the method name of 'step_control'
    def step_control(self, sensor_data):
        global posx, posy, x_end, y_end
        global idx, state, return_point
        # Take off
        if self.on_ground and sensor_data['range_down'] < 0.5:
            self.height_desired=0.5
            control_command = [0.0, 0.0, 0.0, self.height_desired]
            if (region_look(sensor_data['x_global'],sensor_data['y_global'])=='Starting'):
                return_point=[sensor_data['x_global'],sensor_data['y_global']]
                state='search'
            else:
                state='back'
            return control_command
        else:
            self.on_ground = False
            region=region_look(sensor_data['x_global'],sensor_data['y_global'])
            if(state=='search'):
                # Drone tries to reach the base 
                if (region_look(posx[19],posy[19])==region):
                    precision_yaw=0.01
                    precision_dist=0.01
                    speed=0.1
                    if self.height_desired>0.25:
                        self.height_desired=self.height_desired-0.002
                    if self.height_desired<0.15:
                        self.height_desired=self.height_desired+0.002
                else:
                    precision_yaw=0.1
                    precision_dist=0.4
                    speed=0.3
                    self.height_desired=0.5

                #Sequence of points to reach to find the landing pad
                goalx=posx[idx]
                goaly=posy[idx]
                if(region=='Edges')or((region=='Out')and(sensor_data['x_global']<stop_l)and(sensor_data['x_global']>start_l)):
                    if posy[0]!=1.5 :
                        posx.insert(0, stop_l)
                        posy.insert(0, 1.5)
                # Detection of landing pad
                if (sensor_data['range_down']<self.height_desired-0.075)and(region_look(posx[19],posy[19])==region)and((math.cos(sensor_data['yaw'])>0.9)or(math.cos(sensor_data['yaw'])<-0.9)or(math.sin(sensor_data['yaw'])>0.9)or(math.sin(sensor_data['yaw'])<-0.9)):
                    # Edge of the pad
                    x_end = sensor_data['x_global']
                    y_end = sensor_data['y_global']
                    self.height_desired=self.height_desired-0.1
                yaw_obj=direction(sensor_data['x_global'],sensor_data['y_global'],goalx,goaly)
                yaw_cmd=2*reach_yaw(yaw_obj,sensor_data['yaw'])
                control_command = [0., 0.0, yaw_cmd, self.height_desired]
                if yaw_cmd**2<=precision_yaw**2:
                    advance=speed
                    control_command[0]=advance

                # Drone reaches an intermeidate goal, goes to the next
                if distance(sensor_data['x_global'],sensor_data['y_global'],posx[idx],posy[idx]) < precision_dist:
                        idx += 1
                # Drone reaches the landing pad
                elif distance(sensor_data['x_global'],sensor_data['y_global'],x_end, y_end) < precision_dist:
                    state = 'land'

            elif(state=='land'):
                self.height_desired -= 0.001
                control_command = [0., 0., 0, self.height_desired]
                if self.height_desired<0.002:
                    state='stop'

            elif(state=='back'):
                state = 'stop'

            elif(state=='stop'):
                control_command = [0., 0., 0, self.height_desired]
            

            # Obstacle avoidance
            if sensor_data['range_front'] < 0.2 or sensor_data['range_left']<0.1 or sensor_data['range_right']<0.1 or sensor_data['range_back']<0.1:
                if (region_look(posx[idx],posy[idx])==region):
                    posx[idx]=posx[idx-1]
                    posy[idx]=posy[idx-1]
                else:
                    if sensor_data['range_right']>0.1:
                        right=0.1
                    else:
                        right=sensor_data['range_right']
                    if sensor_data['range_left']>0.1:
                        left=0.1
                    else:
                        left=sensor_data['range_left']
                    if sensor_data['range_front']>0.2:
                        front=0.2
                    else:
                        front=sensor_data['range_front']
                    if sensor_data['range_back']>0.1:
                        back=0.1
                    else:
                        back=sensor_data['range_back']
                
                    front = 1.2*(front+back)
                    lat = 1.2*(left-right)
                    if sensor_data['range_left'] > sensor_data['range_right']+0.05:
                        control_command = [-front, lat+0.5*front, -1.*front, self.height_desired]
                    else:
                        control_command = [-front, lat-0.5*front, front, self.height_desired]
        return control_command

def direction(x,y,x_obj,y_obj):
    return math.atan2(y_obj-y,x_obj-x)

def distance(x,y,x_obj,y_obj):
    return math.sqrt((y_obj-y)**2+(x_obj-x)**2)

def reach_yaw(desired_yaw,yaw):
    yaw_cmd = (desired_yaw-yaw)
    if yaw_cmd>math.pi:
        yaw_cmd-=2*math.pi
    elif yaw_cmd<-math.pi:
        yaw_cmd+=2*math.pi
    return yaw_cmd*0.5

def region_look(x,y):
    region='Out'
    if y<max_y-res_pos and y>min_y+res_pos :
        if x<max_x-res_pos and x>min_x+res_pos :
            region='In'
            if x>stop_l+res_pos :
                region='Landing'
            elif x<start_l+res_pos :
                region='Starting'        
            elif y<0.30 or y>2.70:
                region='Edges'
    return region