# Author : Michaël Tasev
# Last update : 16 October 2020
# EPFL Rocket Team, 1015 Lausanne, Switzerland

from Rocket.Rocket import Rocket
import numpy as np


def pitch_damping_moment(rocket: Rocket, rho, calpha, CP, dMdt, cg, w, v):
    """
    pitch_damping_moment computes the pitch damping moment coefficient of the
    rocket. It also applies to yaw damping, but not to roll!
    Damping is based on the rocket's geometry i.e the air resistance opposing
    its rotational movement and the mass change rate during the thrust phase.
    """
    cdm = 0

    if v != 0:
        cdm_thrust = dMdt * (rocket.get_length - cg) ** 2 * w * 2 / (v**2 * rho * rocket.get_max_cross_section_surface)
        CNa_Total = np.sum(calpha*(CP-cg)**2)
        cdm_aero = CNa_Total*w/v

        cdm = cdm_aero + cdm_thrust

    return cdm
