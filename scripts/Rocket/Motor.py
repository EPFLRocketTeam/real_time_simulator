# Motor class file
# Author : Jules Triomphe
# Date : 5 May 2019
# EPFL Rocket Team, 1015 Lausanne, Switzerland

from scipy.interpolate import interp1d
from scipy.integrate import simps
import numpy as np
import bisect


class Motor:
    """
    Motor object
    ============

    Motor object defined by it's file path and the source .eng document found on www.thrustcurve.org.

    Attributes
    ----------

    diameter : float
        Motor diameter (with casing), in [m].

    length : float
        Motor length (with casing), in [m].

    delay_type : str
        Time after burnout when the ejection charge ignites.
        Classifications :
            - Number : Time in seconds
            - T : Tiny
            - M : Medium (~10 s)
            - P : Plugged ; no ejection charge
            - N/A or not listed : not adjustable

    propellant_mass : float
        Propellant mass, in [kg].
        Value is constant.
        To get the actual mass of the motor, use the get_mass() method.

    total_mass : float
        Total mass, in [kg].

    casing_mass : float
        Casing mass, in [kg].
        Value is constant.
        Equal to the difference between the total mass and the propellant mass.

    thrust_pairs : list of str
        Table of the sampled thrust at the corresponding time after ignition.

    thrust_time : list of float
        First column of thrust_pairs listing the time of the provided thrust samples.
        A 0 value is added at the beginning to provide a range from ignition to burnout.

    thrust_force : list of float
        Second column of thrust_pairs listing the thrust at the provided sampling times.
        A 0 value is added at the beginning to provide a data set from ignition to burnout.

    thrust_function : interp1d function
        Handle to calculate the thrust at a given time t [s] between ignition at t=0 and burnout at t=burn_time.
        Should be used as thrust_function(t).

    burn_time : float
        Motor burn time given by the last sampling time of the motor data sheet, in [s].

    total_impulse : float64
        Total impulse of the motor given by a Simpson interpolation of the samples, in [N.s].

    thrust_to_mass : float64
        Thrust to mass ratio of the motor, in [m.s^-2].
        Used to calculate the mass during burn.


    Constructor
    -----------

    __init__(motor_file_path)
        Initializes a motor with its physical and mathematical characteristics from a given motor data sheet path.
        The motor file can be a .txt or a .eng file.


    Methods
    -------

    get_thrust(t) : float
        Returns the thrust of the motor at time t [s], in [N].

    get_propellant_mass(t) : float
        Returns the propellant mass at time t [s], in [kg].

    get_total_mass(t) : float
        Returns the total mass of the motor, casing included, in [kg].

    TODO: Add get_dmass_dt description

    get_cg() : float
        Returns the center of mass of the motor, in [m].
        TODO: Rethink the output of the method.

    get_propellant_inertia(t) : float
        Returns the propellant's inertia at time t [s], in [kg.m^2].

    get_casing_inertia() : float
        Returns the casing's inertia, in [kg.m^2].

    get_motor_inertia(t, d) : float
        Returns the motor's inertia with respect to a point at a distance d [m], in [kg.m^2].
        d is in [m].

    get_total_inertia(t, d) : float
        Returns the total inertia of the motor with respect to a point at a distance d at time t [s], in [kg.m^2].
        d is in [m].

    """

    # --------------------
    # CONSTRUCTOR
    # --------------------

    def __init__(self, motor_file_path: str):
        with open(motor_file_path, 'r') as motor_data:
            motor_data.readline()
            general_data = motor_data.readline().split()

            self.diameter = float(general_data[1]) / 1000
            self.length = float(general_data[2]) / 1000
            self.delay_type = general_data[3]

            self.propellant_mass = float(general_data[4])
            self.total_mass = float(general_data[5])
            self.casing_mass = self.total_mass - self.propellant_mass

            # First value is thrust time
            # Second value is thrust force
            self.thrust_pairs = [line.split() for line in motor_data]
            self.thrust_time = [float(thrust[0]) for thrust in self.thrust_pairs]
            self.thrust_force = [float(thrust[1]) for thrust in self.thrust_pairs]
            # Correct to add (0,0) point
            self.thrust_time.insert(0, 0)
            self.thrust_force.insert(0, 0)

        # Linear interpolation of the thrust force samples
        self.thrust_function = interp1d(self.thrust_time, self.thrust_force)

        self.burn_time = self.thrust_time[-1]

        # Simpson integration of the thrust curve
        self.total_impulse = np.trapz(self.thrust_force, self.thrust_time)
        # self.total_impulse = simps(sample_thrust, sample_time, even='avg')

        self.thrust_to_mass = self.propellant_mass / self.total_impulse

        self.motor_fac = 1

    # --------------------
    # METHODS
    # --------------------

    def set_motor_fac(self, motor_fac: float):
        self.motor_fac = motor_fac

    def get_motor_fac(self):
        return self.motor_fac

    def get_thrust(self, t: float) -> float:
        """
        Computes the current thrust, in [N].
        It is 0 if outside of burn time.

        :param t: time, in [s]
        :return: current thrust force, in [N]
        """
        if 0 <= t <= self.burn_time:
            return self.thrust_function(t)
        else:
            return 0

    def get_burnt_propellant_mass(self, t: float) -> float:
        """
        Computes the current burnt propellant mass, in [kg].

        :param t: time, in [s]
        :return: current burnt propellant mass, in [kg]
        """
        if t < 0:
            return 0
        elif 0 <= t <= self.burn_time:
            thrust_t = self.thrust_time[:bisect.bisect_right(self.thrust_time, t)]
            thrust_f = self.thrust_force[:len(thrust_t)]
            if t not in self.thrust_time:
                thrust_t.append(t)
                thrust_f.append(self.thrust_function(t).tolist())
            current_impulse = np.trapz(thrust_f, thrust_t)
            # Test to simulate exact Matlab code in 1D
            # Yields higher results
            time = np.linspace(0, t, 500)
            current_impulse = np.trapz(self.thrust_function(time), time)
            # current_impulse = simps(thrust_f, thrust_t, even='avg')
            return self.thrust_to_mass * current_impulse
        else:
            return self.propellant_mass

    def get_propellant_mass(self, t: float) -> float:
        """
        Computes the current propellant mass, in [kg].

        :param t: time, in [s]
        :return: current propellant mass, in [kg]
        """
        if t < 0:
            return self.propellant_mass
        elif 0 <= t <= self.burn_time:
            thrust_t = self.thrust_time[:bisect.bisect_right(self.thrust_time, t)]
            thrust_f = self.thrust_force[:len(thrust_t)]
            if 0:  # t not in self.thrust_time: # Yields a higher altitude than 0
                thrust_t.append(t)
                thrust_f.append(self.thrust_function(t).tolist())
            current_impulse = np.trapz(thrust_f, thrust_t)
            # current_impulse = simps(thrust_f, thrust_t, even='avg')
            return self.propellant_mass - self.thrust_to_mass * current_impulse
        else:
            return 0

    def get_total_mass(self, t: float) -> float:
        """
        Computes the current mass of the motor (casing included), in [kg].

        :param t: time, in [s]
        :return: current motor mass, in [kg]
        """
        return self.total_mass - self.get_burnt_propellant_mass(t)

    def get_dmass_dt(self, t: float) -> float:
        """
        Computes the current change in mass of the motor over time, in [kg.s^-1].

        :param t: time, in [s]
        :return: current mass change over time, in [kg.s^-1]
        """
        return self.thrust_to_mass * self.get_thrust(t)

    @property
    def get_cg(self) -> float:
        """
        Computes the motor's center of mass (CG), in [m].

        :return: distance of the CG to the top of the motor, in [m]
        """
        return self.length / 2

    def get_propellant_inertia(self, t: float) -> float:
        """
        Computes the propellant's inertia, in [kg.m^2].

        :param t: time, in [s]
        :return: propellant inertia, in [kg.m^2]
        """
        # Internal grain radius (stays constant)
        r_i = 0.005
        # External grain radius
        r_e = self.diameter / 2

        # Ix inertia : inertia along yaw/pitch axis.
        # We call it "longitudinal" but it is a misuse of the term.
        i_l_grain = self.get_propellant_mass(t) * (self.length ** 2 / 12 + (r_e ** 2 + r_i ** 2) / 4)
        return i_l_grain

    def get_casing_inertia(self) -> float:
        """
        Computes the casing's inertia, in [kg.m^2].

        :return: casing inertia, in [kg.m^2]
        """
        # External grain radius
        r_e = self.diameter / 2
        # We consider an infinitesimal thickness for the casing.
        # Hence r_e == r_i and (r_e**2 + r_i**2)/4 becomes r_e**2/2.
        i_l_casing = self.casing_mass * (self.length ** 2 / 12 + r_e ** 2 / 2)
        return i_l_casing

    def get_motor_inertia(self, t: float, d: float) -> float:
        """
        Computes the motor's inertia with respect to a point at a distance d [m] (e.g. rocket center of mass),
        in [kg.m^2].

        :param t: time, in [s]
        :param d: distance, in [m]
        :return: motor inertia from the distant point, in [kg.m^2]
        """
        return self.get_total_mass(t) * d

    def get_total_inertia(self, t: float, d: float) -> float:
        """
        Computes the total motor inertia with respect to a point at a distance d [m] (e.g. rocket center of mass),
        in [kg.m^2].

        :param t: time, in [s]
        :param d: distance, in [m]
        :return: motor total inertia, in [kg.m^2]
        """
        return self.get_propellant_inertia(t) + self.get_casing_inertia() + self.get_motor_inertia(t, d)

    def get_thrust_time(self):
        return self.thrust_time

    def get_thrust_force(self):
        return self.thrust_force


if __name__ == '__main__':
    # Location of current motor test file
    CS_M1800 = Motor('../Motors/AT_L850.eng')
    # get_thrust method test
    print(CS_M1800.get_thrust(4.5))
    # get_mass method tests
    print(CS_M1800.get_total_mass(CS_M1800.burn_time))
    print(CS_M1800.get_total_mass(10))
    print(CS_M1800.total_impulse)
    print(CS_M1800.thrust_to_mass ** -1 * CS_M1800.get_burnt_propellant_mass(CS_M1800.burn_time))
    print(CS_M1800.get_burnt_propellant_mass(CS_M1800.burn_time))
    print(CS_M1800.propellant_mass)
