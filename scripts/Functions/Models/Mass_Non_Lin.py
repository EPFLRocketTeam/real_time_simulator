from Rocket import Rocket
import numpy as np
from Functions.Models.Thrust import Thrust

def Mass_Non_Lin(t: float, Rocket: Rocket):

    if t>Rocket.get_burn_time():
        mass = Rocket.get_empty_mass() + Rocket.get_motor_mass() - Rocket.get_propel_mass()
        dmassdt = 0
    else:
        tt = np.linspace(0, t, 500)
        current_impulse = np.trapz(tt, Thrust(tt, Rocket))
        mass = Rocket.get_empty_mass() + Rocket.get_motor_mass() - Rocket.get_thrust_to_mass()*current_impulse
        dmassdt = Rocket.get_thrust_to_mass()*Thrust(t, Rocket)
    return mass, dmassdt
