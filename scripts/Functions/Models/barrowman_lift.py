# Author : Michaël Tasev
# Last update : 16 October 2020
# EPFL Rocket Team, 1015 Lausanne, Switzerland

from Rocket.Rocket import Rocket
import math
import numpy as np


def barrowman_lift(rocket: Rocket, alpha, m, theta):

    a_ref = math.pi * rocket.diameters[1] ** 2 / 4

    # Cone
    if rocket.cone_mode == 'on':
        if alpha == 0:
            CNa_cone = 2
        else:
            CNa_cone = 2 * math.sin(alpha) / alpha
    CP_cone = 2 / 3 * rocket.diameters_position[1]

    # Body
    CNa_stage = np.zeros(len(rocket.stages))
    CP_stage = np.zeros(len(rocket.stages))

    for i in range(len(rocket.stages)):
        if alpha == 0:
            CNa_stage[i] = (rocket.diameters[i + 2] ** 2 - rocket.diameters[i + 1] ** 2) * math.pi / a_ref / 2
        else:
            CNa_stage[i] = ((rocket.diameters[i + 2] ** 2 - rocket.diameters[i + 1] ** 2)
                            * math.pi / a_ref / 2 * math.sin(alpha) / alpha)
        if rocket.diameters[i+1] != rocket.diameters[i+2]:
            CP_stage[i] = (rocket.diameters_position[i + 1] + 1 / 3 *
                           (rocket.diameters_position[i + 2] - rocket.diameters_position[i + 1]) *
                           (1 + (1 - rocket.diameters[i + 1] / rocket.diameters[i + 2]) / (
                                       1 - (rocket.diameters[i + 1] / rocket.diameters[i + 2]) ** 2)))
        else:
            CP_stage[i] = None
    """if alpha == 0:
        CNa_stage[0] = (rocket.diameters[3] ** 2 - rocket.diameters[2] ** 2) * math.pi / a_ref / 2
        CNa_stage[1] = (rocket.diameters[4] ** 2 - rocket.diameters[3] ** 2) * math.pi / a_ref / 2
    else:
        CNa_stage[0] = ((rocket.diameters[3] ** 2 - rocket.diameters[2] ** 2)
                        * math.pi / a_ref / 2 * math.sin(alpha) / alpha)
        CNa_stage[1] = ((rocket.diameters[4] ** 2 - rocket.diameters[3] ** 2)
                        * math.pi / a_ref / 2 * math.sin(alpha) / alpha)

    if rocket.diameters[2] == rocket.diameters[3]:
        CP_stage[0] = 0
    else:
        CP_stage[0] = (rocket.diameters_position[2] + 1 / 3 *
                   (rocket.diameters_position[3] - rocket.diameters_position[2]) *
                   (1 + (1 - rocket.diameters[2] / rocket.diameters[3]) / (
                           1 - (rocket.diameters[2] / rocket.diameters[3]) ** 2)))
    if rocket.diameters[3] == rocket.diameters[4]:
        CP_stage[1] = 0
    else:
        CP_stage[1] = (rocket.diameters_position[3] + 1 / 3 *
                   (rocket.diameters_position[4] - rocket.diameters_position[3]) *
                   (1 + (1 - rocket.diameters[3] / rocket.diameters[4]) / (
                           1 - (rocket.diameters[3] / rocket.diameters[4]) ** 2)))"""

    # Fins
    if m < 1:
        beta = math.sqrt(1 - m ** 2)
    else:
        print("Warning : in Barrowman calculations Mach number > 1")
        beta = math.sqrt(m ** 2 - 1)

    fin_n = rocket.get_fin_number
    fin_xs = rocket.get_fin_xs
    fin_xt = rocket.get_fin_xt
    fin_ct = rocket.get_fin_ct
    fin_cr = rocket.get_fin_cr
    fin_span = rocket.get_fin_span

    gamma_c = math.atan(((fin_xs+fin_ct)/2 - fin_cr/2)/fin_span)
    a = 0.5*(fin_ct + fin_cr)*fin_span
    idx=0
    for i, pos in enumerate(rocket.diameters_position):
        if pos < fin_xt:
            idx=i
    r = rocket.diameters[idx]/2
    ktb = 1 + r/(r + fin_span)
    CNa1 = ktb*2*math.pi*fin_span**2 / a_ref / (1 + math.sqrt(1+(beta*fin_span**2 / a / np.cos(gamma_c))**2))
    CNa_fins = CNa1*sum(np.sin(theta+2*math.pi/fin_n*(np.arange(fin_n)))**2)
    CP_fins = fin_xt + fin_xs/3*(fin_cr+2*fin_ct)/(fin_cr+fin_ct) + 1/6*((fin_cr+fin_ct)-(fin_cr*fin_ct)/(fin_cr+fin_ct))

    # Output
    Calpha = np.append(CNa_stage, CNa_fins)
    CP = np.append(CP_stage, CP_fins)
    if rocket.cone_mode == 'on':
        Calpha = np.append(CNa_cone, Calpha)
        CP = np.append(CP_cone, CP)

    for i, cp in enumerate(CP):
        if math.isnan(cp):
            CP[i] = 0

    return Calpha, CP
