# You can change anything in this file except the file name of 'my_control.py',
# the class name of 'MyController', and the method name of 'step_control'.

# Available sensor data includes data['t'], data['x_global'], data['y_global'],
# data['roll'], data['pitch'], data['yaw'], data['v_forward'], data['v_left'],
# data['range_front'], data['range_left'], data['range_back'],
# data['range_right'], data['range_down'], data['yaw_rate'].

import numpy as np
#ozgun ekrem 301295

 
# Don't change the class name of 'MyController'
class MyController():
    def __init__(self):
        self.on_ground = True
        self.height_desired = 0.5
        self.mode='taking_off'
        self.obstacle = False
        
        self.i_x = 0 #between 0 and 6
        self.i_y = 0 #between 0 and 12
        self.search_way = 'up'
        
        self.start = True #just used to have the starting coordinates
        self.x_start = 0.0
        self.y_start = 0.0
        
        self.path = [[0.0,0.0],[0.0,0.0],[0.0,0.0]]
        self.path_counter = 0 
        self.avoidance = False 

    # Don't change the method name of 'step_control'
    def step_control(self, sensor_data):
        x_drone, y_drone = sensor_data['x_global'], sensor_data['y_global']
        yaw = sensor_data['yaw']
        
        
        if abs(sensor_data['v_left'])>=0.5 or abs(sensor_data['v_forward'])>=0.5:
            print("oke")
            #control_command = [0.0, 0.0, 0.0, self.height_desired]
            #return control_command
        
        if abs(sensor_data['roll'])>= 0.3  or  abs(sensor_data['pitch'])>=0.3:
            print("oke roll or pitch")
            #control_command = [0.0, 0.0, 0.0, self.height_desired]
            #return control_command
            
            
        
        
        
        #---------- end ------------------------------------
       
        
          
        #--------------------------------------------------------------------
        #Obstacle detection and avoidance
        
        if self.mode!='travelling_back' and (self.mode!='searching_landing') and (self.mode!='first_landing'):
            #we detect an obstacle on the front 
            if sensor_data['range_front'] < 0.2: #0.55
                if sensor_data['range_left']>sensor_data['range_right']:
                    print("front1")
                    control_command = [-0.07, -0.25, 0.0, self.height_desired] #b 0 on x #c 0.05
                if sensor_data['range_left']<sensor_data['range_right']:
                    control_command = [-0.07, 0.25, 0.0, self.height_desired] #b 0 on x 
                    print("front 2")
                else : 
                    control_command = [-0.07, 0.25, 0.0, self.height_desired]
                    print("front 3")
                return control_command
            #we detect an obstacle on the left 
            if sensor_data['range_left'] < 0.2:  #0.4
                print("Left")
                control_command = [0.2, -0.2, 0.0, self.height_desired]
                return control_command

            #we detect an obstacle on the right 
            if sensor_data['range_right'] < 0.2: #0.4
                control_command = [0.2, 0.2, 0.0, self.height_desired] 
                return control_command

            if sensor_data['range_back']< 0.2: 
                control_command = [0.15, 0.15, 0.0, self.height_desired] 
                return control_command
        
        #inversed scheme for travelling back    
        if self.mode =='travelling_back':
            if sensor_data['range_back'] < 0.2: #0.55
                if sensor_data['range_left']>sensor_data['range_right']:
                    control_command = [0.07, -0.25, 0.0, self.height_desired] #b 0.0 on x #c 0.05
                if sensor_data['range_left']<sensor_data['range_right']:
                    control_command = [0.07, 0.25, 0.0, self.height_desired] #b 0.0 on x
                else : 
                    control_command = [0.07, 0.25, 0.0, self.height_desired]
                return control_command
            #we detect an obstacle on the left 
            if sensor_data['range_left'] < 0.2:  #0.4
                control_command = [-0.1, -0.2, 0.0, self.height_desired] #A -0.2 for x 
                return control_command

            #we detect an obstacle on the right 
            if sensor_data['range_right'] < 0.2: #0.4
                control_command = [-0.1, 0.2, 0.0, self.height_desired] #A -0.2 for x
                return control_command

            if sensor_data['range_front']< 0.2: 
                control_command = [-0.15, 0.15, 0.0, self.height_desired] 
                return control_command
        
             
        #............................................................
        if abs(yaw)>=0+0.1 : 
            if yaw>0:
                control_command=[0.0, 0.0, -0.4, self.height_desired]
            else: 
                control_command=[0.0, 0.0, 0.4, self.height_desired]
            return control_command
        
        
        #---------------------------------
        #Zone testing : 
        
        #make sure that we stay inside the arena : 
        if x_drone <-0.3 :
            control_command = [0.1, 0.0, 0.0, self.height_desired]
            return control_command
        
        if x_drone >=5.3 :
            control_command = [-0.1, 0.0, 0.0, self.height_desired]
            return control_command
        
        if y_drone <-0.3:
            control_command = [0.20, 0.1, 0.0, self.height_desired]
            return control_command
        
        if y_drone >3.3:
            control_command = [0.20, -0.1, 0.0, self.height_desired]
            return control_command
            
           
            
       
        
        
        if x_drone>3.5 and self.mode=='travelling_to_landing': 
            self.mode='searching_landing'
            print("We are now in the landing region and looking for the landing pad") 
            control_command=[0.0, 0.0, 0.2, self.height_desired]
            return control_command
        
        if self.mode=='searching_landing' and sensor_data['range_down']<0.42:
            print("here")
            if (abs(sensor_data['roll']) < 0.15) and (abs(sensor_data['pitch'])<0.15): ### TBT
                self.mode='first_landing'
                print("Landing pad detected. The drone is now going to land")
                control_command=[0.0, 0.0, 0.0, self.height_desired]
                return control_command
          
        if self.mode=='first_landing' and sensor_data['range_down']<0.12 and self.height_desired<=0:
            print("Ok we have our first landing")
            self.mode='second_taking_off'
            print("The mode is now : " , self.mode)
            control_command=[0.0, 0.0, 0.0, 0]
            self.height_desired = 0.5
            
            return control_command
        
        
        
        #fall detection   (on its back)  (doesn't work)
        if sensor_data['range_down'] >= 1.5 :
            print("ON ITS BACK" , sensor_data['range_down'])
            #control_command=[0.0, 0.0, 0.0, -self.height_desired]
            #return control_command
        
        
        
        
        
        #--------------------------------------------------------------------
        # Take off
        if self.on_ground and sensor_data['range_down'] < 0.49:
            control_command = [0.0, 0.0, 0.0, self.height_desired]
            return control_command
        
        if self.mode=='second_taking_off' and sensor_data['range_down'] < 0.39:
            control_command = [0.0, 0.0, 0.0, self.height_desired]
            self.mode=='second_taking_off'
            return control_command
            
       
        #detect the fact that the drone has taken off for the first time and change the mode to travelling to  
        #the landing region
        if self.on_ground and sensor_data['range_down'] >= 0.49:
            self.on_ground = False
            control_command = [0.0, 0.0, 0.0, self.height_desired]
            self.mode='travelling_to_landing'
            if self.start==True: 
                self.start=False
                self.x_start = sensor_data['x_global']
                self.y_start= sensor_data['y_global']
                print(self.x_start , "x start")
                print(self.y_start, "y start")
            return control_command
        
        if self.mode=='second_taking_off' and sensor_data['range_down'] >= 0.39:
            self.mode = 'travelling_back'
            control_command = [0.0, 0.0, 0.0, self.height_desired]
            return control_command
        
        #------------------------------------------
        if self.mode=='travelling_to_landing' and abs(yaw)<0+0.1:
            control_command = [0.2, 0.0, 0.0, self.height_desired]  #b 0.2 on x  #0.18
            return control_command
        
        if self.mode =='travelling_back':
            
            x_current = sensor_data['x_global']
            y_current = sensor_data['y_global']
            
            #print("ok we are going to : " , self.x_start , self.y_start)
            v_x, v_y = self.x_start - x_current, self.y_start - y_current
            
            distance_drone_to_start = np.linalg.norm([self.x_start - x_current, self.y_start- y_current])
            v_x , v_y = v_x/(distance_drone_to_start*6.3), v_y/(distance_drone_to_start*6.3) #A 5  b 5.7 #c5.9
            
            if distance_drone_to_start <=0.05 : #and sensor_data['range_down']<=0.42
                self.mode='last_landing'
                print("We found the starting pad")
                control_command = [0,0,0,self.height_desired]
                return control_command
            control_command = [v_x, v_y, 0.0, self.height_desired]
            return control_command
            
        
        
            
            
        #-----------------------------------
        
        
        if self.mode=='searching_landing': 
            #print("Searching of the landing pad")
            x_goal = self.i_x*0.25 + 3.5
            y_goal = self.i_y*0.25
            x_current = sensor_data['x_global']
            y_current = sensor_data['y_global']
            v_x, v_y = x_goal - x_current, y_goal - y_current
            
            distance_drone_to_goal = np.linalg.norm([x_goal - x_current, y_goal- y_current])
            v_x , v_y = v_x/(distance_drone_to_goal*9), v_y/(distance_drone_to_goal*9)#A5.5 #b6 #c6.4 d 7.5
            
            threshold = 0.07
            
            control_command = [v_x, v_y, 0.0, self.height_desired]
            blocked_1 = sensor_data['range_left'] < 0.2 or sensor_data['range_right']<0.2
            blocked =  blocked_1 or sensor_data['range_front']<0.2
            if distance_drone_to_goal <= 0.1 or (distance_drone_to_goal <= 0.4 and blocked):
                if self.i_y==12 and self.search_way =='up':
                    self.i_x +=1
                    self.search_way = 'down'
                    control_command = [0,0,0,self.height_desired]
                    return control_command
                if self.i_y==0 and self.search_way == 'down': 
                    self.i_x += 1
                    self.search_way = 'up'
                    control_command = [0,0,0,self.height_desired]
                    return control_command

                if self.search_way == 'up':
                    self.i_y +=1
                    
                elif self.search_way == 'down':
                    self.i_y -=1
                    
                    
            if (self.i_y==12 or self.i_y ==0) and self.i_x==6:
                self.i_y=0 
                self.i_x=0
                self.search_way = 'up'
                control_command = [0,0,0,self.height_desired]
                return control_command
                
            #---------obstacle avoidance module ------------------------
           
            x_d, y_d = x_goal - x_current, y_goal - y_current
            tol = 0.2  #tolerance 
            d_x = 0.45 #displacement
            d_y = 0.67  #0.60  displacement #A 0.65 #b 0.67
            
            path = self.path
            
            if self.avoidance == True: 
        
                if abs(yaw)>=0+0.05: #a b c +0.1
                    if yaw>0:
                        control_command=[0.0, 0.0, -0.2, self.height_desired] # abc 0.4
                    else: 
                        control_command=[0.0, 0.0, 0.2, self.height_desired]
                    return control_command
                
                 
                #print("In avoidance mode")
                v_x_av = path[self.path_counter][0] - x_current
                v_y_av = path[self.path_counter][1] - y_current
            
                distance = np.linalg.norm([v_x_av, v_y_av])
                v_x_av , v_y_av = v_x_av/(distance*9), v_y_av/(distance*9)  #A 5.1 #b 6 #c 6.4 #d 7.5
                
                if abs(distance)<0.1: 
                    self.path_counter +=1
                
                if self.path_counter>2:
                    print("Avoidance phase is finished")
                    self.path_counter = 0
                    self.avoidance = False 
                    control_command = [0.0, 0.0, 0.0, self.height_desired]
                    return control_command
            
                control_command = [v_x_av, v_y_av, 0.0, self.height_desired]
                return control_command
                
            #-------------------------------     
            
            if self.avoidance ==False :
                
                if ((abs(x_d)<tol) and (y_d < -tol)) or (self.i_y == 0 and self.i_x==0):
                    if sensor_data['range_right']< 0.2: #1
                        print("MODE NUMBER 1")
                        self.path[0]=[x_current +d_x, y_current]
                        self.path[1]=[x_current +d_x, y_current - d_y]
                        self.path[2]=[x_current, y_current - d_y]
                        self.avoidance = True 
                        self.path_counter = 0
                        control_command = [0.0, 0.0, 0.0, self.height_desired]
                        
                        if  self.i_y==0 and self.search_way == 'down':  #1.5
                            
                            print("MODE NUMBER 1.5")
                            self.path[0]=[x_current +d_x, y_current]
                            self.path[1]=[x_current +d_x, y_current - d_y/4]
                            self.path[2]=[x_current +d_x, y_current - d_y/2]
                            self.avoidance = True 
                            self.path_counter = 0
                            control_command = [0.0, 0.0, 0.0, self.height_desired]
                        
                        return control_command
                    
                if x_d>= tol and abs(y_d)<tol and self.i_y==0 : 
                    if sensor_data['range_front']<0.2: #2
                        print("MODE NUMBER 2")
                        self.path[0]=[x_current, y_current +d_y]
                        self.path[1]=[x_current + d_x/4, y_current +d_y]
                        self.path[2]=[x_current + d_x/2, y_current +d_y]
                        self.avoidance = True 
                        self.path_counter = 0
                        control_command = [0.0, 0.0, 0.0, self.height_desired]
                        return control_command
                    
                if abs(x_d)<tol and y_d>=tol:
                    if sensor_data['range_left']<0.2: #3 
                        print("MODE NUMBER 3")
                        self.path[0]=[x_current +d_x, y_current ]
                        self.path[1]=[x_current + d_x, y_current+d_y]
                        self.path[2]=[x_current, y_current+d_y]
                        self.avoidance = True 
                        self.path_counter = 0
                        control_command = [0.0, 0.0, 0.0, self.height_desired]
                        
                        if self.i_y==12 and self.search_way =='up': #3.5
                            print("MODE NUMBER 3.5")
                            self.path[0]=[x_current +d_x, y_current]
                            self.path[1]=[x_current + d_x + 0.05, y_current+d_y/4]
                            self.path[2]=[x_current +d_x +0.05, y_current+d_y/2]
                            self.avoidance = True 
                            self.path_counter = 0
                            control_command = [0.0, 0.0, 0.0, self.height_desired]
                        return control_command
                
                if x_d>=tol and abs(y_d)<tol and self.i_y == 12 :
                    if sensor_data['range_front']<0.2: #4
                        print("MODE NUMBER 4")
                        self.path[0]=[x_current, y_current + d_y]
                        self.path[1]=[x_current + d_x/2, y_current + d_y]
                        self.path[2]=[x_current +d_x/2, y_current+d_y]
                        self.avoidance = True 
                        self.path_counter = 0
                        control_command = [0.0, 0.0, 0.0, self.height_desired]
                        
                        return control_command
                               
            #-------------------end of module ---------------------------
            
            return control_command
         
                
        if self.mode=='first_landing':
            
            print("Landing," ,self.height_desired )
            self.height_desired -= 0.005
            control_command = [0.0, 0.0, 0.0, self.height_desired]
            return control_command
        
        if self.mode=='last_landing':
            print("Starting the final landing")
            self.height_desired -= 0.01
            control_command = [0.0, 0.0, 0.0, self.height_desired]
            if self.height_desired < -0.02 :
                control_command = [0.0, 0.0, 0.0, 0.0]
                print("We have completed the whole task")
            return control_command
            

        
        




      