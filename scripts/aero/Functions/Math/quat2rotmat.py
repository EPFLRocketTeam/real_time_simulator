# Author : Michaël Tasev
# Last update : 16 October 2020
# EPFL Rocket Team, 1015 Lausanne, Switzerland

import numpy as np


def quat2rotmat(q):
    """q1 = q[0,:]
    q2 = q[1,:]
    q3 = q[2,:]
    q4 = q[3,:]
    return np.reshape([1-2*q2**2-2*q3**2,
    2*(q1*q2 + q3*q4),
    2*(q1*q3 - q2*q4),
    2*(q1*q2 - q3*q4),
    1-2*q1**2-2*q3**2,
    2*(q2*q3 + q1*q4),
    2*(q1*q3 + q2*q4),
    2*(q2*q3 - q1*q4),
    1-2*q1**2 - 2*q2**2]
    , 3, 3, [])"""

    return np.reshape([
        1 - 2 * q[1] ** 2 - 2 * q[2] ** 2,
        2 * (q[0] * q[1] + q[2] * q[3]),
        2 * (q[0] * q[2] - q[1] * q[3]),
        2 * (q[0] * q[1] - q[2] * q[3]),
        1 - 2 * q[0] ** 2 - 2 * q[2] ** 2,
        2 * (q[1] * q[2] + q[0] * q[3]),
        2 * (q[0] * q[2] + q[1] * q[3]),
        2 * (q[1] * q[2] - q[0] * q[3]),
        1 - 2 * q[0] ** 2 - 2 * q[1] ** 2], [3, 3], order='F')
