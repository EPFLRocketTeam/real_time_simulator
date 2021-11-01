# Author : Michaël Tasev
# Last update : 16 October 2020
# EPFL Rocket Team, 1015 Lausanne, Switzerland

import numpy as np


def rot2anglemat(c):
    pitch = np.arctan(c[1, 2] / c[2, 2]) * 180 / np.pi
    yaw = - np.arcsin(c[0, 2]) * 180 / np.pi
    roll = np.arctan(c[0, 1] / c[0, 0]) * 180 / np.pi

    return [pitch, yaw, roll]
