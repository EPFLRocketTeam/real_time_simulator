# Author : Michaël Tasev
# Last update : 16 October 2020
# EPFL Rocket Team, 1015 Lausanne, Switzerland

import numpy as np
from scipy import interpolate


def wind_model(t, I, V_inf, Model, h_alt):
    t_wind_ = []
    wind_ = []

    if Model == 'None':
        U = V_inf

    else:
        if not t_wind_:
            t_wind_.append(0)
            wind_.append(V_inf)

        if Model == 'Gaussian':  # TODO : add VonKarman model (needs to be complete on MatLab too) and Logarithmic model
            if t > t_wind_[-1]:
                t_wind_.append(t)
                turb_std = I * V_inf
                U = np.random.normal(V_inf, turb_std)
                wind_.append(U)
            else:
                U = interpolate.interp1d(t_wind_, wind_, t, 'linear')

        elif Model == "VonKarman":
            if t > t_wind_[-1]:
                t_wind_.append(t)
                turb_std = I * V_inf
                z0 = 0.001
                zi = 1000*z0**0.18

                # TODO: complete model
                wind_.append(U)
            else:
                U = interpolate.interp1d(t_wind_, wind_, t, 'linear')

        elif Model == "Logarithmic":
            z0 = 0.0024
            h_ground = 1.5
            U = V_inf*(np.log(h_alt/z0)/np.log(h_ground/z0))
            wind_.append(U)



        else:
            raise Exception("wind_model ", Model, " is unknowkn")

    return U
