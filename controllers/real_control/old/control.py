#faire meilleur mix capteurs, ATTENTION ca change tout
#esquive coté intérieur
#comprendre positions

## Problèmes estimator (tester stabilité sans déplacement (print values)) 

## Recap of the code :
# Different possible states : TAKE_OFF, land, GO_BACK, approach
# Different possible zones : START, STOP, IN, OUT, BORDER, FINAL

## 28/05 : augmenter un peu la range d'atterissage
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

uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E708')

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

## What happens OUT and BORDER


def check_zone(x,y):
    zone='OUT'
    if y<3 and y>-1 : # beware of int_float comparison ? 
        if x<5 and x>0 :
            zone='IN'
            if x>3.5:
                zone='LANDING'
            elif x<1.5:
                zone='START'        
            elif y<0.30 or y>2.70:
                zone='BORDER'
    return zone



if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    #LOG ENTRIES
    lg_stab = LogConfig(name='Stabilizer', period_in_ms=500)
    #lg_stab.add_variable('stabilizer.roll', 'float')
    #lg_stab.add_variable('stabilizer.pitch', 'float')
    #lg_stab.add_variable('stabilizer.yaw', 'float')
    lg_stab.add_variable('stateEstimate.x', 'float')
    lg_stab.add_variable('stateEstimate.y', 'float')
    #lg_stab.add_variable('stateEstimate.z', 'float')
    lg_stab.add_variable('range.front')
    lg_stab.add_variable('range.back')
    lg_stab.add_variable('range.left')
    lg_stab.add_variable('range.right')
    #lg_stab.add_variable('kalman.stateX')#, 'float'
    #lg_stab.add_variable('kalman.stateY')
    lg_stab.add_variable('kalman.stateZ')
    #lg_stab.add_variable('kalman.stateX','float'),'float'
    #lg_stab.add_variable('range.zrange')
    #lg_stab.add_variable('range.up')
    #kal = LogConfig(name='Position', period_in_ms=500)
    #kal.add_variable('kalman.stateZ', 'float')

    #state = ['TAKE_OFF', 'LAND', 'SEARCH', 'GO_BACK', 'STOP', 'AVOID', 'ARRIVED']
    state = 'TAKE_OFF' # Initially we want  to take off
    #zone = ['START', 'EXPLO', 'FINAL', 'OUT', 'GOAL']
    zone = 'START' # Initially we are at start
    target_found = False

    ## Grid points
    #px=[1.85,0.0]#[3.65, 4.85,4.85,3.65, 3.65, 4.85,4.85,3.65,3.65, 4.85,4.85,3.65,3.65, 4.85,4.85,3.65,3.65, 4.85,4.85,3.65,]# 
    #py=[-0.6,0.0]#[0.15, 0.15, 0.45, 0.45, 0.75, 0.75, 1.05, 1.05, 1.35, 1.35, 1.65, 1.65, 1.95, 1.95, 2.25, 2.25, 2.55, 2.55, 2.85, 2.85]#
    px=[1.85, 1.85, 2.35, 2.35,  2.8, 2.8, 3.2,  3.2, 0.0]
    py=[-0.4,  1.0,  1.0, -0.4, -0.4, 1.0, 1.0, -0.4, 0.0]
    
    #px=[1.85, 1.85, 1.85, 1.85, 2.35, 2.35, 2.35, 2.35,  2.8,  2.8, 2.8, 2.8, 0.0]
    #py=[-0.6, -0.2,  0.2,  0.6,  0.6,  0.2, -0.2, -0.6, -0.6, -0.2, 0.2, 0.6, 0.0]
    index=0

    landing=np.array([[0.,0.],[0.,0.]]) ## coordinates of the landing pad (one for first landing and one for second landing ? )
    height_desired=0.2
    previous = 0
    
    speedx=0.
    speedy=0.
    
    #state='go'
    print('GO')

    cf = Crazyflie(rw_cache='./cache')
    with SyncCrazyflie(uri, cf=cf) as scf:
        
        # Note: it is possible to add more than one log config using an
        # array.
        # with SyncLogger(scf, [lg_stab, other_conf]) as logger:
        with SyncLogger(scf,lg_stab) as logger: 
        
            endTime = time.time() + 120
            startTime = endTime - 120
            landing_time = endTime - 120

            print('before') ## what is here ?
            cf.param.set_value('kalman.resetEstimation', '1')
            time.sleep(0.1)
            cf.param.set_value('kalman.resetEstimation', '0')
            time.sleep(2)
            print('after')

            #BASE SAVING

            for log_entry in logger:
                timestamp = log_entry[0]
                data = log_entry[1]
                logconf_name = log_entry[2]
                #print(data['stateEstimate.x'],data['kalman.stateX'])
                # print('x',data['stateEstimate.x'])
                # print('y',data['stateEstimate.y'])
                zone= check_zone(data['stateEstimate.x'],data['stateEstimate.y'])
                zone='START'
                
                if state!='land':#time.time()-startTime<7.5:
                    #if time.time()-startTime<5:#data['kalman.stateZ']<0.6
                    if endTime-time.time()<2: # Land state triggered when 2 seconds left
                        state='land'
                    elif height_desired<0.6: # Keep going up until we reach 0.6
                        height_desired+=0.05
                        cf.commander.send_hover_setpoint(0., 0., 0, height_desired)
                        #cf.commander.send_hover_setpoint(0, 0., 0, 0)
                    elif state=='approach': ## If near the landing pad
                        if abs(landing[0][0]-data['stateEstimate.x'])<0.1 and abs(landing[0][1]-data['stateEstimate.y'])<0.1: ## BASED ON THE LANDING COORDINATES THAT WE HAVE (COMPUTED LATER IN THE CODE)
                            height_desired-=0.02
                        if height_desired<0.05:
                            state='land' # initally 'landing'
                        kp=0.5
                        cf.commander.send_hover_setpoint(kp*(landing[0][0]-data['stateEstimate.x']), kp*(landing[0][1]-data['stateEstimate.y']), 0, height_desired) # PD Control
                        #cf.commander.send_hover_setpoint(0, 0., 0, 0)
                    else:
                        #print(data['range.zrange'], data['range.up'], data['kalman.stateZ'])
                        #print(data['kalman.stateZ'])
                        speed_max=0.2
                        print(zone)
                        if zone=='FINAL'or zone=='START':
                            speed_max=0.2
                            print('difference', data['kalman.stateZ']-height_desired, landing[0][0], time.time()-startTime)
                            ### GET THE DIFFERENCE BETWEEN TWO CONSECUTIVE POINTS
                            print('écart', data['kalman.stateZ'] - previous)             
                            ###
                            print('TIME SINCE LANDING', time.time()-landing_time)
                            if data['kalman.stateZ'] - previous > 0.022 and landing[0][0]==0 and time.time()-landing_time > 9.: # SAVE PAD LOCATION  # (data['kalman.stateZ']-height_desired) < -0.1
                                height_desired-=0.1
                                landing[0][:]=[data['stateEstimate.x'],data['stateEstimate.y']] # keep starting point in memory
                                print('point1 :',landing[0][:])
                                state = 'land'
                                print('LAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAANDING PAD')
                                #KEEP TRAJECTORY
                                if abs(px[index]-data['stateEstimate.x'])>abs(py[index]-data['stateEstimate.y']):
                                    px[index]+=0.5
                                else:                                                     
                                    py[index]+=0.5
                            elif height_desired-data['kalman.stateZ']<-0.2 and landing[0][0]!=0:
                                height_desired+=0.1
                                landing[1][:]=[data['stateEstimate.x'],data['stateEstimate.y']]
                                print('point2 :',landing[1][:])
                                #prepare landing
                                state='approach'
                                landing[0][:]=(landing[0]+landing[1])/2
                                print('approach obj : ',landing[0][:])
                            if abs(px[index]-data['stateEstimate.x'])>abs(py[index]-data['stateEstimate.y']):
                                speedx=(px[index]-data['stateEstimate.x'])*0.5
                            else:
                                speedy=(py[index]-data['stateEstimate.y'])*0.5

                        #REACH X Y PT
                        else:
                            print('coucou')
                            speedx=(px[index]-data['stateEstimate.x'])*0.5
                            speedy=(py[index]-data['stateEstimate.y'])*0.5
                            if speedx < abs(0.05):
                                speedx = 0.
                            if speedy < abs(0.05):
                                speedy = 0.

                        if math.sqrt((px[index]-data['stateEstimate.x'])**2+(py[index]-data['stateEstimate.y'])**2)<0.1:
                            print('Point ', index, ' Reached') 
                            if index>len(px)-1:
                                print('first',index)
                                state='land'#'STOP'?
                                print('ok')
                            else:
                                index+=1
                                print('index second',index)
                        #SPEED LIMITATION
                        if speedx>speed_max:
                            speedx=speed_max
                        elif speedx<-speed_max:
                            speedx=-speed_max
                        if speedy>speed_max:
                            speedy=speed_max
                        elif speedy<-speed_max:
                            speedy=-speed_max
                        cf.commander.send_hover_setpoint(speedx, speedy, 0, height_desired)
                        #cf.commander.send_hover_setpoint(0, 0., 0, 0)
                else: # if state == land
                    print("IN THE ELSE")
                    height_desired-=0.2 ## go down
                    cf.commander.send_hover_setpoint(-speedx/4, -speedy/4, 0, height_desired)
                    #cf.commander.send_hover_setpoint(0, 0., 0, 0)
                    if zone=='START' and data['kalman.stateZ']<0.1: ## Once we have landed, go back to start point 
                        state='GO_BACK'
                        #BASE NEW OBJ
                        index=0
                        px=[0, 0.3, -0.3, 0., 0.]#px=[startx] 
                        py=[0,  0., 0., 0.3, -0.3]#py=[starty] 
                        speedx=0
                        speedy=0
                        landing[0][0]=0.
                        time.sleep(2)
                        landing_time=time.time()
                    elif zone=='START' and data['kalman.stateZ']<0.1:
                        break

                
                print('end_state', state) 
                print('end_zone', zone) 
                print('end_speeds', speedx, speedy)
                if time.time()-startTime>3:#height_desired>0.2 # if we are after the 3 first seconds, obstacle avoidance
                    if data['range.front']<300:
                        cf.commander.send_hover_setpoint(-0.2, -0.2, 0, height_desired)
                        print('OBSTACLE_FRONT')
                    if data['range.left']<100:
                        cf.commander.send_hover_setpoint(0, -0.2, 0, height_desired)
                        print('PBSTACLE_LEFT')
                    if data['range.right']<100:
                        cf.commander.send_hover_setpoint(0, 0.2, 0, height_desired)
                        print('OBSTACLE_RIGHT')
                    if data['range.back']<100: 
                        cf.commander.send_hover_setpoint(0.2, 0.2, 0, height_desired)
                        print('OBSTACLE_BACK')
                
                previous = data['kalman.stateZ']
                time.sleep(0.05)

                #if data['stateEstimate.z']>0.3:
                #cf.commander.send_hover_setpoint(0, 0, 0.8, 0.4)
                #time.sleep(0.1)
                #print('[%d][%s]: %s' % (timestamp, logconf_name, data))

                if time.time() > endTime: ## finish program
                    break





# while(1):
#     assign_zone()
#     # take_off
#     if state == 'STOP':
#         if zone == 'START':
#             ctrl_takeoff()
#         elif zone == 'FINAL':
#             ctrl_takeoff()

#     # Check obstacle
#     if obstacle:
#         obstacle_avoidance()

#     if state == 'SEARCH' and zone == 'EXPLO':
#         ctrl_avance()

#     if zone == 'FINAL' and target_found==False:
#         quadrillage()
#         if height < desired_height:
#             state = 'LAND'
#             zone = 'GOAL'
#             return

#     if state = 'LAND' and zone 'GOAL':
#         ctrl_land()

#     if state == 'GO_BACK':
#         if pos =! base
#             ctrl_go_home()
#         else:
#             ctrl_land()


# def ctrl_takeoff(height=None):
#     if height is correct:
#         if target_found == False:
#             state = 'SEARCH'
#             zone = 'EXPLO'
#             return command
#         else:
#             state = 'GO_BACK'
#             return command
#     else:
#         return command

# def obstacle_avoidance():
#     if zone == 'EXPLO' and target_found == False:
#         return command
#     elif zone == 'FINAL' and target_found == False:
#         return command
#     elif target_found == True:
#         return command


# def assign_zone():
#     # check if final zone limit is crossed
#     if state is not 'STOP':
#         if not target_found:
#             if x < limit_explo:
#                 zone = 'EXPLO'
#             else:
#                 zone = 'FINAL'
#         # Check if in zone
#         if not (0<x<5 and 0<y<3):
#             zone = 'OUT'
#         return zone

# def quadrillage()

# def ctrl_land():
#     if target_found==False:
#         save pos
#         compute distance
#         if distance reached:
#             go_down
#             if height < seuil
#                 stop motors
#                 state = 'STOP'
#                 target_found = True
#         else:
#             avance
#     else:
#         go_down
#         if height < seuil
#             stop motors
#             state = 'ARRIVED'
#             target_found = False


# def ctrl_go_home():
#     orientation
#     avancer

# def ctrl_avance():
#     avance
