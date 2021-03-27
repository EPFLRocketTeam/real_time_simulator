# Author : Henri Faure
# Date : 31 March 2019
# EPFL Rocket Team, 1015 Lausanne, Switzerland

def drag_shuriken(Rocket, theta, alpha, Uinf, nu):
    # DRAG_SHURIKEN estimates the drag coefficient normalized to the Rocket's
    # reference area for the shuriken airbrake design.
    # INPUTS :
    # - Rocket  : Rocket object
    # - theta   : Airbrakes command input, -190.5 = closed, 1.165 = open [deg]
    # - alpha   : wind angle of attack [rad]
    # - Uinf    : Air free stream velocity [m/s]
    # - nu      : dynamic viscosity coefficient [m2/s]

    import numpy as np
    import math
    from scipy.interpolate import interp1d

    # parameters
    angle_tab = np.array([0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 66, 73])

    surface_tab = np.array([0, 1.77, 2.85, 4.038, 5.27, 6.534, 7.825, 9.139, 10.47, 11.83, 13.19, 14.57, 15.95, 17.62, 19.56]) * 10**(-3)
    h_tab = np.array([0, 3.91, 7.758, 11.517, 15.16, 18.658, 21.982, 25.104, 27.995, 30.628, 32.979, 35.023, 36.741, 38.564, 39.561]) * 10**(-3)

    angle = (theta + 232)*73./232.9

    S = interp1d(angle_tab, surface_tab)(angle)
    h = interp1d(angle_tab, h_tab)(angle)

    CD0 = 1.17
    U = abs(Uinf * math.cos(alpha))
    Rex = Rocket.ab_x * U / nu
    delta = 0.37 * Rocket.ab_x / Rex ** 0.2

    # drag coefficient
    if h < delta:
        qr = 49 / 72 * (h / delta) ** (2 / 7)
    else:
        qr = 1 - 4 / 9 * delta / h + 1 / 8 * (delta / h) ** 2

    CD = Rocket.ab_n * CD0 * qr * S / Rocket.get_max_cross_section_surface

    return CD
