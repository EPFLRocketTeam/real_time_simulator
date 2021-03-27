# Author : Michaël Tasev
# Last update : 1 December 2020
# EPFL Rocket Team, 1015 Lausanne, Switzerland

import numpy as np

"""
ROT2QUAT computes the quaternion representation of the attitude based on
the rotation matrix rotating the earth coordinate system to the rocket
coordinate system.
"""


def rot2quat(C):
    q = np.zeros(4).transpose()
    T = np.trace(C)
    qsq = np.array([1 + 2 * C[0][0] - T, 1 + 2 * C[1][1] - T, 1 + 2 * C[2][2] - T, 1 + T]) / 4

    x = np.max(qsq)
    if qsq[0] == x:
        i = 0
    elif qsq[1] == x:
        i = 1
    elif qsq[2] == x:
        i = 2
    elif qsq[3] == x:
        i = 3

    if i == 3:
        q[3] = np.sqrt(x)
        q[0] = (C[1][2] - C[2][1]) / (4 * q[3])
        q[1] = (C[2][0] - C[0][2]) / (4 * q[3])
        q[2] = (C[0][1] - C[1][0]) / (4 * q[3])

    elif i == 2:
        q[2] = np.sqrt(x)
        q[0] = (C[0][2] - C[2][0]) / (4 * q[2])
        q[1] = (C[2][1] - C[1][2]) / (4 * q[2])
        q[3] = (C[0][1] - C[1][0]) / (4 * q[2])

    elif i == 1:
        q[1] = np.sqrt(x)
        q[0] = (C[0][1] - C[1][0]) / (4 * q[1])
        q[2] = (C[2][1] - C[1][2]) / (4 * q[1])
        q[3] = (C[2][1] - C[1][2]) / (4 * q[1])

    elif i == 0:
        q[0] = np.sqrt(x)
        q[1] = (C[0][1] - C[1][0]) / (4 * q[0])
        q[2] = (C[0][2] - C[2][0]) / (4 * q[0])
        q[3] = (C[1][2] - C[2][1]) / (4 * q[0])

    return q
