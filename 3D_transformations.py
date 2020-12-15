import numpy as np


class Pose3D:
    def __init__(self, rotation, translation): # define state matrix parameters
        self.rotation = rotation
        self.translation = translation

    def inv(self):
        inv_rotation = self.rotation.transpose()
        inv_translation = -np.dot(inv_rotation, self.translation)

        return Pose3D(inv_rotation, inv_translation)

    def mul(self, other):
        return Pose3D(np.dot(self.rotation, other.rotation),
                      np.dot(self.rotation, other.translation) + self.translation)

    def __str__(self):
        return "rotation:\n" + str(self.rotation) + "\ntranslation:\n" + str(self.translation.transpose())


def compute_quadrotor_pose(global_marker_pose, observed_marker_pose): # transformation from observed marker
    # and global marker pose

    global_quadrotor_pose = global_marker_pose.mul(observed_marker_pose.inv())

    return global_quadrotor_pose
