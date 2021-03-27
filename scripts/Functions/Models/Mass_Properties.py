from Rocket import Rocket
from Functions.Models.Thrust import Thrust
import numpy as np


def Mass_Properties(t: float, rocket: Rocket, Opt):

    rocket_m = rocket.get_empty_mass()
    burn_time = rocket.get_burn_time()
    propel_mass = rocket.get_propel_mass()
    motor_mass = rocket.get_motor_mass()
    thrust_to_mass = rocket.get_thrust_to_mass()
    casing_mass = rocket.get_motor_casing_mass()
    motor_length = rocket.get_motor_length()


    if rocket.isHybrid == 0:
        if Opt == "Linear":
            if t == 0:
                dMdt = propel_mass/burn_time
                M = rocket_m

            elif t > burn_time:
                M = rocket_m + rocket.casing_mass
                dMdt = 0
            else:
                dMdt = propel_mass/burn_time
                M = rocket_m + motor_mass - t * dMdt
        elif Opt == "NonLinear":
            if t == 0:
                dMdt = thrust_to_mass*Thrust(t, rocket)
                M = rocket_m
            elif t > burn_time:
                M = rocket_m + motor_mass - propel_mass
                dMdt = 0
            else:
                tt = np.linspace(0, t, 500)
                current_impulse = np.trapz(Thrust(tt, rocket), tt)
                M = rocket_m + motor_mass - thrust_to_mass * current_impulse
                dMdt = thrust_to_mass * Thrust(t, rocket)
        else:
            print("ERROR: Opt parameter should be Linear or NonLinear")

        # Center of Mass

        Cm = (rocket.cg * rocket_m + (M - rocket_m) * (rocket.L - motor_length / 2)) / M
        dCmdt = (dMdt * (rocket.L - motor_length / 2) - dMdt * Cm) / M

        # Moment of intertia

        R_i = 0.005
        R_e = rocket.get_motor_dia() / 2

        I_L_Casing = casing_mass * (motor_length ** 2 / 12 + R_e ** 2 / 2)

        Grain_Mass = M - rocket_m - casing_mass
        I_L_Grain = Grain_Mass * (motor_length ** 2 / 12 + (R_e ** 2 + R_i ** 2) / 4)

        I_L = rocket.rocket_I + I_L_Casing + I_L_Grain + (Grain_Mass + casing_mass) * (
                rocket.L - Cm - motor_length / 2) ** 2

        dI_L_Grain = dMdt * (motor_length ** 2 / 12 + (R_e ** 2 + R_i ** 2) / 4)

        dI_Ldt = dI_L_Grain + dMdt * (rocket.L - Cm - motor_length / 2) ** 2 + 2 * (
                Grain_Mass + casing_mass) * (rocket.L - Cm - motor_length / 2) * dCmdt

        I_R = 1e6

        dI_Rdt = 0

    else:
        if Opt == "Linear":
            if t == 0:
                dMdt = propel_mass / burn_time
                M = rocket_m

            elif t > burn_time:
                M = rocket_m + casing_mass
                dMdt = 0
            else:
                dMdt = propel_mass / burn_time
                M = rocket_m + motor_mass - t * dMdt
        elif Opt == "NonLinear":
            if t == 0:
                dMdt = thrust_to_mass * Thrust(t, rocket)
                M = rocket_m
            elif t > burn_time:
                M = rocket_m + motor_mass - propel_mass
                dMdt = 0
            else:
                tt = np.linspace(0, t, 500)
                current_impulse = np.trapz(tt, Thrust(tt, Rocket))
                M = rocket_m + motor_mass - thrust_to_mass * current_impulse
                dMdt = thrust_to_mass * Thrust(t, Rocket)
        else:
            print("Opt parameter should be Linear or NonLinear")


        # TODO: Hybrid case

        # center of mass
        motor_cm = (rocket.L - rocket.motor_lengthP/2)*(rocket.motor_massP - t*(rocket.propel_massP/burn_time))
        motor_cmF = (rocket.L - rocket.motor_lengthF/2)

    return M, dMdt, Cm, dCmdt, I_L, dI_Ldt, I_R, dI_Rdt