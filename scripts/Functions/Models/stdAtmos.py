from math import exp, sqrt
from scipy.interpolate import interp1d


def stdAtmos(alt, Env):

    """
    Input:  - alt: altitude [m]
            - Env: Environment structure

    Output: - T: Local standard temperature[K]
            - a: local speed of sound[m/s]
            - p: local standard pressure[Pa]
            - rho: local standard density [kg/m^3]
            - Nu: local kinematic viscosity of air [m^2/s]

    ASSUMPTIONS:
     - hydrostatic approximation of atmosphere
     - linear temperature variation with altitude with a slope of -9.5°C/km comes from result of radiosonde data
     - homogenous composition

    LIMITATIONS:
    - troposphere: 10km

     CHECK ALTITUDE RANGE
    """
    if alt > 1e4:
        raise Exception("stdAtmos:outOfRange, The altitude is out of range: max 10km", )

    R = 287.04
    gamma = 1.4
    p0 = 101325
    T0 = 288.15
    a0 = 340.294
    g0 = 9.80665

    # TEMPERATURE MODEL
    T = Env.ground_temperature + Env.dTdH * (alt - Env.ground_altitude) / 1000 # en [K]
    p = p0 * (1 + Env.dTdH / 1000 * alt / T0) ** (-g0 / R / Env.dTdH * 1000)
    x = Env.saturation_vapor_ratio * Env.ground_humidity
    rho = p / R / T * (1 + x) / (1 + 1.609 * x)
    a = sqrt(gamma * R * T)

    Nu = Env.get_viscosity(alt)

    return T, a, p, rho, Nu