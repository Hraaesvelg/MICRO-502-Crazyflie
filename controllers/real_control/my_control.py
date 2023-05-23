# You can change anything in this file except the file name of 'my_control.py',
# the class name of 'MyController', and the method name of 'step_control'.

# Available sensor data includes data['t'], data['x_global'], data['y_global'],
# data['roll'], data['pitch'], data['yaw'], data['v_forward'], data['v_left'],
# data['range_front'], data['range_left'], data['range_back'],
# data['range_right'], data['range_down'], data['yaw_rate'].

from locale import locale_encoding_alias
#from symbol import return_stmt
import numpy as np
import math

px=[3.65, 4.85,4.85,3.65, 3.65, 4.85,4.85,3.65,3.65, 4.85,4.85,3.65,3.65, 4.85,4.85,3.65,3.65, 4.85,4.85,3.65,]#[4.25, 4.25]#
py=[0.15, 0.15, 0.45, 0.45, 0.75, 0.75, 1.05, 1.05, 1.35, 1.35, 1.65, 1.65, 1.95, 1.95, 2.25, 2.25, 2.55, 2.55, 2.85, 2.85]#[0.5, 2.5]#

min_x, max_x = 0, 5.0 # meter
min_y, max_y = 0, 3.0 # meter
start_lim, stop_lim = 1.5, 3.5 #meter

range_max = 2.0 # meter, maximum range of distance sensor
res_pos = 0. # meter
conf = 0.2 # certainty given by each measurement
t = 0 # only for plotting

# Don't change the class name of 'MyController'
class MyController():
    def __init__(self):
        global state
        self.on_ground = True
        self.height_desired = 0.5
        state='aller'
        global index
        index=0
        global landing 
        landing=np.array([[0,0]])
        

    # Don't change the method name of 'step_control'
    def step_control(self, sensor_data):
        global px, py
        global index
        global state
        global landing
        global return_point
        global index
        # Take off
        if self.on_ground and sensor_data['range_down'] < 0.49:#and
            self.height_desired=0.5
            control_command = [0.0, 0.0, 0., self.height_desired]
            if (check_zone(sensor_data['x_global'],sensor_data['y_global'])=='START'):
                return_point=[sensor_data['x_global'],sensor_data['y_global']]
                state='aller'
            else:
                state='retour'
            return control_command
        # Land
        else:
            
            self.on_ground = False
            zone=check_zone(sensor_data['x_global'],sensor_data['y_global'])
            #[pitch, roll, yaw, height]
            #self.height_desired -= 0.005
            #control_command = [0.5, 0.0, 0.0, self.height_desired]
            
        
            if(state=='aller'):
                #reach base and call approach
                #be precise only in landing zone
                if (check_zone(px[19],py[19])==zone):
                    precision_yaw=0.005
                    precision_dist=0.01
                    speed=0.2
                    if self.height_desired>0.252:
                        self.height_desired=self.height_desired-0.002
                    if self.height_desired<0.148:
                        self.height_desired=self.height_desired+0.002
                else:
                    #si proche du bord insérer px py milieu
                    precision_yaw=0.1
                    precision_dist=0.5
                    speed=0.3
                    self.height_desired=0.5

                #predetermined reach points to scan landing zone
                objx=px[index]
                objy=py[index]
                if (sensor_data['range_down']<self.height_desired-0.075)and(check_zone(px[19],py[19])==zone)and((math.cos(sensor_data['yaw'])>0.9)or(math.cos(sensor_data['yaw'])<-0.9)or(math.sin(sensor_data['yaw'])>0.9)or(math.sin(sensor_data['yaw'])<-0.9)):
                    #points from base saved
                    print('trigger up')
                    landing=np.concatenate((landing,[[sensor_data['x_global'],sensor_data['y_global']]]),axis=0)
                    self.height_desired=self.height_desired-0.1
                    print('landing : ',landing)
                    #change objective to go straight (X cm forward)
                    forward_x=0
                    forward_y=0
                    if math.cos(sensor_data['yaw'])>0.9:
                        forward_x=1
                    elif math.cos(sensor_data['yaw'])<-0.9:
                        forward_x=-1
                    elif math.sin(sensor_data['yaw'])>0.9:
                        forward_y=1
                    elif math.sin(sensor_data['yaw'])<-0.9:
                        forward_y=-1
                    px[index]=sensor_data['x_global']+forward_x
                    py[index]=sensor_data['y_global']+forward_y
                    # yaw_obj = 
                elif (sensor_data['range_down']>self.height_desired+0.05)and(check_zone(px[19],py[19])==zone)and(len(landing)>1)and((math.cos(sensor_data['yaw'])>0.9)or(math.cos(sensor_data['yaw'])<-0.9)or(math.sin(sensor_data['yaw'])>0.9)or(math.sin(sensor_data['yaw'])<-0.9)):
                    print('trigger down')
                    state='approach'
                    self.height_desired+=0.1
                    landing=np.concatenate((landing,[[sensor_data['x_global'],sensor_data['y_global']]]),axis=0)
                    index=0
                    print('landing : ',landing)
                    #si data et pas capteur : obj = mean y data, mean x data + offset et yaw adapter, vitesse reduite ==>losange, moyenne centre
                    px=[(landing[1][0]+landing[2][0])/2, (landing[1][0]+landing[2][0])/2+0.35*math.sin(sensor_data['yaw'])]#[(landing[1][0]+landing[2][0])/2, landing[2][0]-0.30, (landing[1][0]+landing[2][0])/2, landing[2][0]+0.30, (landing[1][0]+landing[2][0])/2]
                    py=[(landing[1][1]+landing[2][1])/2, (landing[1][1]+landing[2][1])/2+0.35*math.cos(sensor_data['yaw'])]#[landing[2][1]+0.06,(landing[1][1]+landing[2][1])/2, landing[1][1]-0.06,(landing[1][1]+landing[2][1])/2, (landing[1][1]+landing[2][1])/2]

                    print('px approach : ',px)
                    print('py approach : ',py)

                    #SI OBSTACLE SE POSER DES QUE POSSIBLE SUR LA BASE CONNUE
                
                #follow set_point
                yaw_obj=direction(sensor_data['x_global'],sensor_data['y_global'],objx,objy)
                yaw_command=reach_yaw(yaw_obj,sensor_data['yaw'])
                control_command = [0., 0.0, yaw_command, self.height_desired]
                #if aligned, go
                if yaw_command**2<=precision_yaw**2:
                    advance=speed#/(1+yaw_command**2)
                    control_command[0]=advance

                # When the drone reaches the goal setpoint, next or stop
                if distance(sensor_data['x_global'],sensor_data['y_global'],px[index],py[index]) < precision_dist:
                    # Hover at the final setpoint
                    #if (index >= len(px)-1):
                        #state='stop'
                        #control_command = [0.0, 0.0, 0.0, self.height_desired]
                    #    px[index]=np.rand()*1.5+3.5
                    #    py[index]=np.rand()*3
                    #    return control_command
                    #else :# # Select the next setpoint as the goal position
                        index += 1
            elif (state=='approach'):
                #on a survolé la base
                precision_yaw=0.02
                precision_dist=0.02
                speed=0.1
                #keep altitude and save step points
                if (sensor_data['range_down']<self.height_desired-0.05):
                    print('trigger up')
                    landing=np.concatenate((landing,[[sensor_data['x_global'],sensor_data['y_global']]]),axis=0)
                    self.height_desired=self.height_desired-0.1
                elif (sensor_data['range_down']>self.height_desired+0.05):
                    print('trigger down')
                    landing=np.concatenate((landing,[[sensor_data['x_global'],sensor_data['y_global']]]),axis=0)
                    self.height_desired=self.height_desired+0.1
                    print('landing : ',landing)
                
                #follow set_point
                yaw_obj=direction(sensor_data['x_global'],sensor_data['y_global'],px[index],py[index])
                yaw_command=reach_yaw(yaw_obj,sensor_data['yaw'])
                control_command = [0., 0.0, yaw_command, self.height_desired]
                #if aligned, go
                if yaw_command**2<=precision_yaw**2:
                    advance=speed#/(1+yaw_command**2)
                    control_command[0]=advance
                #print(distance(sensor_data['x_global'],sensor_data['y_global'],px[index],py[index]))
                # When the drone reaches the goal setpoint, next or stop
                ##print('index',index,'len(landing)',len(landing))
                if distance(sensor_data['x_global'],sensor_data['y_global'],px[index],py[index]) < precision_dist:
                    # Hover at the final setpoint
                    #if (index > 1):
                        state='land'
                        print('land')
                        control_command = [0.0, 0.0, 0.0, self.height_desired]
                    #    return control_command
                    #elif(index==2)or(len(landing)>=5):#
                    #    print('centering')
                        #print(float(landing[4][0]))*math.sin(sensor_data['yaw'])
                    #    px.append((landing[1][0]+0.5*landing[2][0]+0.5*landing[3][0])*0.5+(float(landing[4][0]-0.15))*math.sin(sensor_data['yaw']))#-(landing[1][0]+landing[2][0])/2)
                    #    py.append((landing[1][1]+0.5*landing[2][1]+0.5*landing[3][1])*0.5+(float(landing[4][1]-0.15))*math.cos(sensor_data['yaw']))#-(landing[1][1]+landing[2][1])/2)
                    #    index=len(px-1)
                    #else :# # Select the next setpoint as the goal position
                    #    index += 1
                    #    print('index : ',index)
            elif(state=='land'):
                #land and pass to retour
                self.height_desired -= 0.001
                control_command = [0., 0., 0, self.height_desired]
                if self.height_desired<0.002:
                    print('landed')
                    state='retour'
                    #self.on_ground=True


            elif(state=='retour'):
                #self.on_ground=True
                #self.height_desired+=0.05

                if False :#zone!='START' and sensor_data['range_down'] < 0.49:
                    self.height_desired+=0.02
                    return [0, 0, 0, self.height_desired]
                else:
                    if (zone=='START'):
                        precision_yaw=0.005
                        precision_dist=0.01
                        speed=0.1
                        if self.height_desired>0.251:
                            self.height_desired=self.height_desired-0.002
                    else:
                        #si proche du bord insérer px py milieu
                        precision_yaw=0.2
                        precision_dist=0.3
                        speed=0.3
                        if sensor_data['range_down'] < 0.49:
                            self.height_desired+=0.02

                #follow set_point
                yaw_obj=direction(sensor_data['x_global'],sensor_data['y_global'],return_point[0],return_point[1])
                yaw_command=reach_yaw(yaw_obj,sensor_data['yaw'])
                control_command = [0., 0.0, yaw_command, self.height_desired]
                #if aligned, go
                if yaw_command**2<=precision_yaw**2:
                    advance=speed#/(1+yaw_command**2)
                    control_command[0]=advance

                # When the drone reaches the goal setpoint, next or stop
                if distance(sensor_data['x_global'],sensor_data['y_global'],return_point[0],return_point[1]) < precision_dist:
                    state='land'
            elif(state=='stop'):
                control_command = [0., 0., 0, self.height_desired]
            

            # Obstacle avoidance with distance sensors
            if sensor_data['range_front'] < 0.4 or sensor_data['range_left']<0.3 or sensor_data['range_right']<0.3 or sensor_data['range_back']<0.2:
                #utile??
                #if (len(landing)>2)and((state=='aller')or(state=='approach')):
                #    state='approach'
                #    index=2
                #    print(landing)
                #    px[2]=np.mean(landing[:][0])
                #    py[2]=np.mean(landing[:][1])
                #random.randint
                
                if (check_zone(px[index],py[index])==zone):
                    if(sensor_data['range_front']<0.3 or sensor_data['range_left']<0.1 or sensor_data['range_right']<0.1):
                        if (index==0) and (px[index]!=px[index+1]or py[index]!=py[index+1]):
                            print('go noeud suivant')
                            px[index]=px[index+1]
                            py[index]=py[index+1]
                        elif (px[index]!=px[index-1]or py[index]!=py[index-1]):
                            print('go noeud precedent')
                            px[index]=px[index-1]
                            py[index]=py[index-1]
                        print('px',px)
                        print('py',py)
                elif zone!='LANDING':
                    if sensor_data['range_left']>0.3:
                        left_bounded=0.3
                    else:
                        left_bounded=sensor_data['range_left']
                    if sensor_data['range_right']>0.3:
                        right_bounded=0.3
                    else:
                        right_bounded=sensor_data['range_right']
                    if sensor_data['range_front']>0.4:
                        front_bounded=0.4
                    else:
                        front_bounded=sensor_data['range_front']
                    if sensor_data['range_back']>0.2:
                        back_bounded=0.2
                    else:
                        back_bounded=sensor_data['range_back']
                
                    gain_front=1*(front_bounded+back_bounded)
                    gain_lat=1*(left_bounded-right_bounded)
                    print('gain_front',gain_front)
                    print('gain_lat ',gain_lat)
                    
                    if sensor_data['range_front'] < 0.2 or sensor_data['range_back']<0.1:
                        if (zone=='BORDER' or gain_lat<0.1)and(state!='retour'):
                            if sensor_data['y_global']<1.5:
                                control_command[0:2] = [-0.3*gain_front, 0.3*gain_front]
                            else:
                                control_command[0:2] = [-0.3*gain_front, -0.3*gain_front]
                        elif(state!='retour'):
                            if left_bounded > right_bounded:
                                control_command[0:2] = [-0.3*gain_front, 0.3*gain_front]
                            else:
                                control_command[0:2] = [-0.3*gain_front, -0.3*gain_front]
                        else:
                            if (zone=='BORDER' or gain_lat<0.1):
                                if sensor_data['y_global']<1.5:
                                    control_command[0:2] = [-0.3*gain_front, -0.3*gain_front]
                                else:
                                    control_command[0:2] = [-0.3*gain_front, 0.3*gain_front]
                            else:
                                if left_bounded > right_bounded:
                                    control_command[0:2] = [-0.3*gain_front, 0.3*gain_front]
                                else:
                                    control_command[0:2] = [-0.3*gain_front, -0.3*gain_front]
                    else :
                        control_command[1]=gain_lat
        return control_command

def direction(x,y,x_obj,y_obj):
    return math.atan2(y_obj-y,x_obj-x)

def distance(x,y,x_obj,y_obj):
    return math.sqrt((y_obj-y)**2+(x_obj-x)**2)

def reach_yaw(desired_yaw,yaw):
    #mieux? PID?
    yaw_command = (desired_yaw-yaw)#+(yaw_rate-yaw_rate_obj)*0.5
    if yaw_command>math.pi:
        yaw_command=yaw_command-2*math.pi
    elif yaw_command<-math.pi:
        yaw_command=yaw_command+2*math.pi
    return yaw_command

def check_zone(x,y):
    zone='OUT'
    if y<max_y-res_pos and y>min_y+res_pos :
        if x<max_x-res_pos and x>min_x+res_pos :
            zone='IN'
            if x>stop_lim+res_pos :
                zone='LANDING'
            elif x<start_lim+res_pos :
                zone='START'        
            elif y<0.30 or y>2.70:
                zone='BORDER'
    return zone