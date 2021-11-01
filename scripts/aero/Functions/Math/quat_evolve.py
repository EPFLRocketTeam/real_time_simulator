# Author : Michaël Tasev
# Last update : 16 October 2020
# EPFL Rocket Team, 1015 Lausanne, Switzerland

import numpy as np


def quat_evolve(q, w):
    # quat_evolve returns the time derivative of the quaternion attitude vector
    # as a function of the current attitude q and the rotation w expressed in
    # the same frame as q.

    # correction is to correct for integration errors
    # (c.f. Modeling and Simulation of aerospace
    # vehicle dynamics, second edition p.126, Peter H. Zipfel)

    omega = np.array([[  0  , w[2] , -w[1], -w[0]],
                      [-w[2],  0   , w[0] , -w[1]],
                      [w[1] , -w[0],  0   , -w[2]],
                      [w[0] , w[1] ,  w[2],   0 ]])
    correction = (1 - np.linalg.norm(q)) * q
                      
    q = np.array([q])
    q_dot = 0.5*np.dot(q,omega)
                        
    return (correction  + q_dot[0] )
