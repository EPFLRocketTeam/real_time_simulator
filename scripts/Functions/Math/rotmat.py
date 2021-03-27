# Author : Sayid Derder
# Last update : 1 December 2020
# EPFL Rocket Team, 1015 Lausanne, Switzerland

import numpy as np
import math


def rotmat(a, ax):

    a = a*math.pi/180
    if ax == 1 :
        C = np.array([[1, 0        , 0         ],
                      [0, np.cos(a), -np.sin(a)],
                      [0, np.sin(a), np.cos(a)]])

    elif ax == 2 :
        C = np.array([[np.cos(a), 0        , np.sin(a)],
                      [0, 1, 0],
                      [-np.sin(a), 0, np.cos(a)]])

    elif ax == 3 :
        C = np.array([[np.cos(a), -np.sin(a), 0],
                      [np.sin(a), np.cos(a), 0],
                      [0, 0, 1]])
    else:
        print("Error: In ROTMAT, Axes number must be between 1 and 3")

    return C