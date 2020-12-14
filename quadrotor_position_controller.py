import numpy as np
# implements the 3D position control of the quadrotor.

# Its input is the current state of the quadrotor (state) and the desired state (state_desired).
# Both states are instances of the State class, which comprises the position and the velocity
# of the quadrotor. The ouput of the method are three linear velocities commands for the X, Y, and Z axis
# respectively.



class State:
    def __init__(self):
        self.position = np.zeros((3, 1))
        self.velocity = np.zeros((3, 1))


class UserCode:
    def __init__(self): # tune the controller gains
        Kp_xy = 2
        Kp_z = 1
        Kd_xy = 1
        Kd_z = 0

        self.Kp = np.array([[Kp_xy, Kp_xy, Kp_z]]).T
        self.Kd = np.array([[Kd_xy, Kd_xy, Kd_z]]).T

    def compute_control_command(self, t, dt, state, state_desired):
        self.plot(state.position, state_desired.position)
        # compute u command proportionnal term + derivative term
        u = self.Kp * (state_desired.position - state.position) + self.Kd * (state_desired.velocity - state.velocity)

        return u

    def plot(self, position, position_desired):
        from plot import plot
        plot("x", position[0])
        plot("x_des", position_desired[0])
        plot("y", position[1])
        plot("y_des", position_desired[1])
        plot("z", position[2])
        plot("z_des", position_desired[2])