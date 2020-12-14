class UserCode:
    def __init__(self): # define parameters for max overshooting 1m
        # achieve stability in 10s
        self.Kp = 5
        self.Kd = 5
        self.last = 0

    def compute_control_command(self, t, dt, x_measured, x_desired):

        # param t: time since simulation start
        # param dt: time since last call to compute_control_command
        # param x_measured: measured position (scalar)
        # param x_desired: desired position (scalar)


        vel = (x_measured - self.last) / dt # compute velocity
        u = self.Kp * (x_desired - x_measured) + self.Kd * (0 - vel) #compute control command of PD controller

        self.last = x_measured;

        return u
