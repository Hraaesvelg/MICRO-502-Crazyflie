

state = ['TAKE_OFF', 'LAND', 'SEARCH', 'GO_BACK', 'STOP', 'AVOID', 'ARRIVED']
zone = ['START', 'EXPLO', 'FINAL', 'OUT', 'GOAL']
target_found = False

while(1):
    assign_zone()
    # take_off
    if state == 'STOP':
        if zone == 'START':
            ctrl_takeoff()
        elif zone == 'FINAL':
            ctrl_takeoff()

    # Check obstacle
    if obstacle:
        obstacle_avoidance()

    if state == 'SEARCH' and zone == 'EXPLO':
        ctrl_avance()

    if zone == 'FINAL' and target_found==False:
        quadrillage()
        if height < desired_height:
            state = 'LAND'
            zone = 'GOAL'
            return

    if state = 'LAND' and zone 'GOAL':
        ctrl_land()

    if state == 'GO_BACK':
        if pos =! base
            ctrl_go_home()
        else:
            ctrl_land()


def ctrl_takeoff(height=None):
    if height is correct:
        if target_found == False:
            state = 'SEARCH'
            zone = 'EXPLO'
            return command
        else:
            state = 'GO_BACK'
            return command
    else:
        return command

def obstacle_avoidance():
    if zone == 'EXPLO' and target_found == False:
        return command
    elif zone == 'FINAL' and target_found == False:
        return command
    elif target_found == True:
        return command


def assign_zone():
    # check if final zone limit is crossed
    if state is not 'STOP':
        if not target_found:
            if x < limit_explo:
                zone = 'EXPLO'
            else:
                zone = 'FINAL'
        # Check if in zone
        if not (0<x<5 and 0<y<3):
            zone = 'OUT'
        return zone

def quadrillage()

def ctrl_land():
    if target_found==False:
        save pos
        compute distance
        if distance reached:
            go_down
            if height < seuil
                stop motors
                state = 'STOP'
                target_found = True
        else:
            avance
    else:
        go_down
        if height < seuil
            stop motors
            state = 'ARRIVED'
            target_found = False


def ctrl_go_home():
    orientation
    avancer

def ctrl_avance():
    avance
