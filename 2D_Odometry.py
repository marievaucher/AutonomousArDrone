import numpy as np
from plot import plot_trajectory


# here we compute new state vector with data measurements

class UserCode:
    def __init__(self):
        self.position = np.array([[0], [0]])

    def rotation_to_world(self, yaw):
        # compute rotation matrix
        from math import cos, sin
        return np.array([[cos(yaw), -sin(yaw)], [sin(yaw), cos(yaw)]])

    def measurement_callback(self, t, dt, navdata):
        # t: time since simulation start
        # dt : time since last call to measurement_callback
        # navdata : measurements of the quadrotor

        self.position = self.position + dt * np.dot(self.rotation_to_world(navdata.rotZ),
                                                    np.array([[navdata.vx], [navdata.vy]]))
        plot_trajectory("odometry", self.position)
