#faire meilleur mix capteurs, ATTENTION ca change tout
#esquive coté intérieur
#comprendre positions


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


def check_zone(x,y):
    zone='OUT'
    if y<3 and y>0 :
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
    state = 'TAKE_OFF'
    #zone = ['START', 'EXPLO', 'FINAL', 'OUT', 'GOAL']
    zone = 'START'
    target_found = False

    px=[3.65]#[3.65, 4.85,4.85,3.65, 3.65, 4.85,4.85,3.65,3.65, 4.85,4.85,3.65,3.65, 4.85,4.85,3.65,3.65, 4.85,4.85,3.65,]#
    py=[0.15]#[0.15, 0.15, 0.45, 0.45, 0.75, 0.75, 1.05, 1.05, 1.35, 1.35, 1.65, 1.65, 1.95, 1.95, 2.25, 2.25, 2.55, 2.55, 2.85, 2.85]#
    index=0

    landing=np.array([[0.,0.],[0.,0.]])
    height_desired=0.2
    #state='go'
    print('GO')

    cf = Crazyflie(rw_cache='./cache')
    with SyncCrazyflie(uri, cf=cf) as scf:
        
        # Note: it is possible to add more than one log config using an
        # array.
        # with SyncLogger(scf, [lg_stab, other_conf]) as logger:
        with SyncLogger(scf,lg_stab) as logger: 
        
            endTime = time.time() + 10
            startTime = endTime - 10

            print('before')
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

                if state!='land':#time.time()-startTime<7.5:
                    #if time.time()-startTime<5:#data['kalman.stateZ']<0.6
                    if height_desired<0.6:
                        height_desired+=0.05
                        cf.commander.send_hover_setpoint(0., 0., 0, height_desired)
                        #cf.commander.send_hover_setpoint(0, 0., 0, 0)
                    elif endTime-time.time()<2:
                        state='land'
                        print('LAND')
                    elif state=='approach':
                        if abs(landing[0][0]-data['stateEstimate.x'])<0.1 and abs(landing[0][1]-data['stateEstimate.y'])<0.1:
                            height_desired-=0.02
                        if height_desired<0.05:
                            state='landing'
                        kp=0.5
                        cf.commander.send_hover_setpoint(kp*(landing[0][0]-data['stateEstimate.x']), kp*(landing[0][1]-data['stateEstimate.y']), 0, height_desired)
                        #cf.commander.send_hover_setpoint(0, 0., 0, 0)
                    else:
                        #print(data['range.zrange'], data['range.up'], data['kalman.stateZ'])
                        #print(data['kalman.stateZ'])
                        speed_max=0.2
                        if zone=='FINAL'or zone=='START':
                            speed_max=0.05
                            if data['kalman.stateZ']-height_desired<-0.2 and landing[0][0]==0:#
                                height_desired-=0.1
                                landing[0][:]=[data['stateEstimate.x'],data['stateEstimate.y']]
                                print('point1 :',landing[0][:])
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
                            speedx=(px[index]-data['stateEstimate.x'])*0.5
                            speedy=(py[index]-data['stateEstimate.y'])*0.5

                        if math.sqrt((px[index]-data['stateEstimate.x'])**2+(py[index]-data['stateEstimate.y'])**2)<0.1:
                            if index<len(px)-1:
                                state='land'#'STOP'?
                            else:
                                index+=1
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
                else:#LANDING
                    height_desired-=0.03
                    cf.commander.send_hover_setpoint(0, 0, 0, height_desired)
                    #cf.commander.send_hover_setpoint(0, 0., 0, 0)
                    if zone=='FINAL' and data['kalman.stateZ']<0.1:
                        state='GO_BACK'
                        #BASE NEW OBJ
                        index=0
                        px=[0]#px=[startx]
                        py=[0]#py=[starty]
                    elif zone=='START' and data['kalman.stateZ']<0.1:
                        break

                

                if time.time()-startTime>3:#height_desired>0.2
                    if data['range.front']<1000:
                        cf.commander.send_hover_setpoint(-0.2, -0.1, 0, height_desired)
                    if data['range.left']<800:
                        cf.commander.send_hover_setpoint(0, -0.2, 0, height_desired)
                    if data['range.right']<800:
                        cf.commander.send_hover_setpoint(0, 0.2, 0, height_desired)
                    if data['range.back']<800:
                        cf.commander.send_hover_setpoint(0.2, 0.1, 0, height_desired)
                
                time.sleep(0.1)

                #if data['stateEstimate.z']>0.3:
                #cf.commander.send_hover_setpoint(0, 0, 0.8, 0.4)
                #time.sleep(0.1)
                #print('[%d][%s]: %s' % (timestamp, logconf_name, data))

                if time.time() > endTime:
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