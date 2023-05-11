class Parameters:
    def __init__(self):
        # General parameters
        self.debug = True

        # takeoff parameters
        self.epsilon_z_direction = 0.01
        self.height_desired_take_off = 0.5
        self.z_increment = 0.005
        self.height_stand_take_off = 0.2
        self.epsilon_rotation = 0.05
        self.tof_rotation_speed = 0.6

        # Exploration parameters
        self.landing_zone = "LD"
        self.take_off_zone = "TK"
        self.explo_zone = "EX"
        self.x_limit = [0, 1.5, 3.5, 5]
        self.y_limit = [0, 3]
        self.explo_speed = 0.5
        self.explo_desired_altitude = 0.5
