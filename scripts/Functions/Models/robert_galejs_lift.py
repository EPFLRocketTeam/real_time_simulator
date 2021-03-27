# Author : Michaël Tasev
# Last update : 16 October 2020
# EPFL Rocket Team, 1015 Lausanne, Switzerland

from Rocket.Rocket import Rocket
import numpy as np
import math


def robert_galejs_lift(rocket, alpha, k):
    # Cone
    if rocket.cone_mode == 'on':
        Ap_cone = 0.5 * rocket.diameters_position[1] * rocket.diameters[1]
        Xp_cone = 2 / 3 * rocket.diameters_position[1]

    # Stages
    Ap_stage = np.zeros(len(rocket.stages))
    Xp_stage = np.zeros(len(rocket.stages))
    """for i in range(len(rocket.stages)):
        Ap_stage[i] = ((rocket.diameters[i + 1] + rocket.diameters[i + 2]) / 2
                       * (rocket.diameters_position[i + 2] - rocket.diameters_position[i + 1]))
        Xp_stage[i] = (rocket.diameters_position[i + 1] + 1 / 3 *
                       (rocket.diameters_position[i + 2] - rocket.diameters_position[i + 1]) *
                       (rocket.diameters[i + 1] + 2 * rocket.diameters[i + 2]) / (rocket.diameters[i + 1] +
                       rocket.diameters[i + 2]))"""
    Ap_stage[0] = ((rocket.diameters[1] + rocket.diameters[2]) / 2
                       * (rocket.diameters_position[2] - rocket.diameters_position[1]))
    Ap_stage[1] = ((rocket.diameters[2] + rocket.diameters[3]) / 2
                       * (rocket.diameters_position[3] - rocket.diameters_position[2]))
    Xp_stage[0] = (rocket.diameters_position[1] + 1 / 3 *
                   (rocket.diameters_position[2] - rocket.diameters_position[1]) *
                   (rocket.diameters[1] + 2 * rocket.diameters[2]) / (rocket.diameters[1] +
                                                                              rocket.diameters[2]))
    Xp_stage[1] = (rocket.diameters_position[2] + 1 / 3 *
                   (rocket.diameters_position[3] - rocket.diameters_position[2]) *
                   (rocket.diameters[2] + 2 * rocket.diameters[3]) / (rocket.diameters[2] +
                                                                      rocket.diameters[3]))

    # Output
    Ap = Ap_stage
    Xp = Xp_stage
    if rocket.cone_mode == 'on':
        Ap = np.append(Ap_cone, Ap)
        Xp = np.append(Xp_cone, Xp)

    Calpha2 = 4 / math.pi / rocket.diameters[1]**2 * k * Ap * alpha

    return Calpha2, Xp
