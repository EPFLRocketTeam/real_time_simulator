from Rocket import Rocket
import numpy as np
from scipy.interpolate import interp1d

def Thrust(tt, rocket: Rocket):
    thrust = []
    if type(tt) == float or type(tt) == np.float64:
        if tt > rocket.get_burn_time():
            T = 0
        elif tt < 0:
            T = 0
        else:
            interp = interp1d(rocket.get_thrust_time(), rocket.get_thrust_force())
            T = interp(tt)
        return T
    else:
        for t in tt:
            if t > rocket.get_burn_time():
                T = 0
            elif t<0:
                T = 0
            else:
                interp = interp1d(rocket.get_thrust_time(), rocket.get_thrust_force())
                T = interp(t)
            thrust.append(T)
        return thrust