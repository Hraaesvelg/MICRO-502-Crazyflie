state = ['TAKE_OFF', 'LAND', 'SEARCH', 'GO_BACK', 'STOP', 'AVOID', 'QUADRL']
zone = ['START', 'EXPLO', 'FINAL']
target_found = False

while (1):
    # take_off
    if state == 'TAKE_OFF':
        if zone == 'START' and
            --> takeoff and save
            data
            if height corect:
                state = 'SEARCH'
                zone = 'EXPLO'
        elif zone = 'FINAL':
            procedure
            takoff
            different
            if height corect:
                state = 'GO_BACK'

    if obstacle detected:
        state = 'AVOID'
        if zone = 'EXPLO'

        elif zone = 'FINAL'

        avoid
        obstacle

    if position in 'FINAL' and zone = 'EXPLO':
        zone = 'FINAL'
        state = 'QUADRL'

    if test land height and zone == 'FINAL':
        state = 'LAND'

    if on ground and zone == 'FINAL':
        state = 'STOP'
        target_found = True

    if on_ground and target_found = True and state = 'STOP':
        state = 'TAKE_OFF'

    if position = base and height check and state = 'GO_BACK':
        state = 'LAND'
        zone = 'START'

    if on ground and zone == 'START' and target_found = True:
        state = 'STOP'
        lumieres
