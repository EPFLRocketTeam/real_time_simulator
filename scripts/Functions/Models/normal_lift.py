# Author : Michaël Tasev
# Last update : 16 October 2020
# EPFL Rocket Team, 1015 Lausanne, Switzerland

from Rocket.Rocket import Rocket
from Functions.Models.barrowman_lift import barrowman_lift
from Functions.Models.robert_galejs_lift import robert_galejs_lift
import numpy as np


def normal_lift(rocket: Rocket, alpha, k, m, theta, Galejs):
    """
        NORMALLIFT computes the normal force intensity applied to the center of
        pressure according to Barrowman's theory and corrections for extreme
        aspect ratio bodies proposed by robert Galejs.
        INPUTS:
        - Rocket      : Rocket object
        - alpha       : angle of attack [rad]
        - k           : Robert Galejs' correction factor
        - m           : Mach number
        - theta       : Roll angle [rad]
        - Galejs      : Flag indicating use of Galejs' correction or not [1 or 0]
        OUTPUTS:
        - CNa        : Normal lift derivative versus delta coefficient derivative [1/rad]
        - Xp          : Center of pressure
        - CNa_barrowman: Normal lift coefficient derivatives of rocket components
        according to barrowman theory [1/rad]
        - Xp_barrowman: Center of pressure of rocket components
        according to barrowman theory [1/rad]

    """

    CNa_barrowman, Xp_barrowman = barrowman_lift(rocket, alpha, m, theta)
    # Factor for montecarlo simulation
    # useful ?
    Xp_barrowman = Xp_barrowman*rocket.cp_fac
    CNa_barrowman = CNa_barrowman*rocket.CNa_fac

    if Galejs:
        CNa_galejs, Xp_galejs = robert_galejs_lift(rocket, alpha, k)
        CNa = np.sum(CNa_barrowman) + np.sum(CNa_galejs)
        Xp = (np.sum(CNa_barrowman*Xp_barrowman) + np.sum(CNa_galejs*Xp_galejs)) / CNa
    else:
        CNa = np.sum(CNa_barrowman)
        Xp = np.sum(CNa_barrowman*Xp_barrowman) / CNa

    return CNa, Xp, CNa_barrowman, Xp_barrowman
