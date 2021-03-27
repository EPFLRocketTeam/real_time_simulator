# US standard atmosphere class file
# Author : Jules Triomphe
# Date : 30 March 2019
# EPFL Rocket Team, 1015 Lausanne, Switzerland

from math import exp, sqrt, sin, cos
from scipy.interpolate import interp1d
import numpy as np


class stdAtmosUS:
    """
    US Standard Atmosphere object
    ==========


    Attributes
    ----------

    R : float
        Real gas constant of air, in [J.kg^-1.K^-1].

    GAMMA : float
        Specific heat coefficient of air, unitless.

    P0 : float
        Mean sea level pressure, in [Pa].

    RHO0 : float
        Mean sea level air density, in [kg.m^-3].

    T0 : float
        Mean sea level temperature (25°C), in [K].

    A0 : float
        Mean sea level speed of sound, in [m.s^-1].

    G0 : float
        Mean sea level gravitational constant, in [m.s^-2].

    VISCOSITY : list of float
        Table of temperatures with their corresponding VISCOSITY, in [K] and [Pa].

    ground_altitude : float
        Ground altitude AMSL, in [m].

    ground_temperature : float
        Ground temperature, in [K].

    ground_pressure : float
        Ground pressure, in [Pa].

    ground_humidity : float
        Ground humidity, in arbitrary units (0 to 1).

    dTdH : float
        Variation of temperature as a function of altitude, in [K.m^-1].

    temperature_nu : float
        Temperatures for the corresponding viscosities, in [K].

    viscosity_nu : float
        Viscosities at the corresponding temperatures, in [Pa].

    viscosity_function : interp1d function
        Handle to calculate the viscosity of air at a given temperature T in [K]. Should be used as
        viscosity_function(T) or viscosity_function(self.get_temperature(altitude)) where altitude is in [m].

    p_ws : float
        Water saturation pressure, in [Pa].

    saturation_vapor_ratio : float
        Saturation vapor ratio, dimensionless.

    Constructor
    -----------

    __init__(ground_altitude, ground_temperature, ground_pressure, ground_humidity)
        Initializes the atmosphere based on ground altitude, temperature, pressure and humidity.

    Methods
    -------

    get_temperature(altitude)
         Returns the temperature at a given altitude AMSL in [m], in [K].

    get_pressure(altitude)
         Returns the pressure at a given altitude AMSL in [m], in [Pa].

    get_density(altitude)
         Returns the air density at a given altitude AMSL in [m], in [kg.m^-3].

    get_speed_of_sound(altitude)
        Returns the speed of sound at a given altitude AMSL in [m], in [m.s^-1].

    get_viscosity(altitude)
        Returns the air viscosity at a given altitude AMSL in [m], in [Pa].

    get_turb(altitude)
        Returns the turbulence TODO : complete get_turb(altitude)

    get_turb_model()
        Returns the model of turbulence, either 'None', 'Gaussian', 'VonKarman', 'Logarithmic'

    """

    # --------------------
    # CLASS ATTRIBUTES
    # --------------------

    R = 287.04
    GAMMA = 1.4
    P0 = 101325
    RHO0 = 1.225
    T0 = 288.15
    A0 = 340.294
    G0 = 9.80665
    Turb_model = 'None'
    V_inf = 10
    Rail_Angle = 0
    Rail_Length = 7
    V_Azimuth = 45
    Rail_Azimuth = 181
    V_dir = np.array([1, 0, 0.0])



    VISCOSITY = ([200, 7.5400e-06],
                 [250, 1.1370e-05],
                 [260, 1.2410e-05],
                 [270, 1.2990e-05],
                 [280, 1.3850e-05],
                 [290, 1.3850e-05],
                 [300, 1.5780e-05],
                 [310, 1.6590e-05],
                 [320, 1.7540e-05],
                 [330, 1.8510e-05],
                 [340, 1.9510e-05],
                 [350, 2.0730e-05])

    # --------------------
    # CONSTRUCTOR
    # --------------------

    def __init__(self, ground_altitude: float, ground_temperature: float, ground_pressure: float,
                 ground_humidity: float):
        self.ground_altitude = ground_altitude
        self.ground_temperature = ground_temperature
        self.ground_pressure = ground_pressure
        self.ground_humidity = ground_humidity

        self.dTdH = -9.5

        self.temperature_nu = [float(line[0]) for line in self.VISCOSITY]
        self.viscosity_nu = [float(line[1]) for line in self.VISCOSITY]
        self.viscosity_function = interp1d(self.temperature_nu, self.viscosity_nu)

        self.p_ws = exp(77.345 + 0.0057 * self.ground_temperature \
                        - 7235 / self.ground_temperature) / self.ground_temperature ** 8.2
        self.saturation_vapor_ratio = 0.62198 * self.p_ws / (self.ground_pressure - self.p_ws)

        


    # --------------------
    # METHODS
    # --------------------

    def get_temperature(self, altitude: float) -> float:
        """
        Computes the temperature at a given altitude AMSL in [m], in [K].

        :param altitude: altitude, in [m]
        :return: temperature at altitude, in [K]
        """
        return self.ground_temperature + self.dTdH * (altitude - self.ground_altitude) / 1000

    def get_pressure(self, altitude: float) -> float:
        """
        Computes the pressure at a given altitude AMSL in [m], in [Pa].

        :param altitude: altitude, in [m]
        :return: pressure at altitude, in [Pa]
        """
        return self.P0 * (1 + self.dTdH / 1000 * altitude / self.T0) ** (-self.G0 / self.R / self.dTdH * 1000)

    def get_density(self, altitude: float) -> float:
        """
        Computes the density at a given altitude AMSL in [m], in [kg.m^-3].

        :param altitude: altitude, in [m]
        :return: density at altitude, in [kg.m^-3]
        """
        x = self.saturation_vapor_ratio * self.ground_humidity
        return self.get_pressure(altitude) / self.R / self.get_temperature(altitude) * (1 + x) / (1 + 1.609 * x)

    def get_speed_of_sound(self, altitude: float) -> float:
        """
        Computes the speed of sound at a given altitude AMSL in [m], in [m.s^-1].

        :param altitude: altitude, in [m]
        :return: speed of sound at altitude, in [m.s^-1]
        """
        return sqrt(self.GAMMA * self.R * self.get_temperature(altitude))

    def get_viscosity(self, altitude: float) -> float:
        """
        Computes the air viscosity at a given altitude AMSL in [m], in [Pa].

        :param altitude: altitude, in [m]
        :return: air viscosity at altitude, in [Pa]
        """
        return self.viscosity_function(self.get_temperature(altitude))

    def get_turb(self, altitude: float):
        return 0  # TODO : complete

    def get_turb_model(self):
        return self.Turb_model

    def get_V_inf(self):
        return self.V_inf  # todo : check v_inf /!\ magic number

    def set_wind(self, v_wind_inf: float, azimuth_wind: float):
        self.V_inf = v_wind_inf
        self.V_Azimuth = azimuth_wind

        self.V_dir = np.array([np.cos(azimuth_wind*np.pi/180), np.sin(azimuth_wind*np.pi/180), 0.0])

if __name__ == '__main__':
    US_Atmos = stdAtmosUS(1382, 308, 86000, 0.15)
    # US_Atmos.dTdH = -6.5
    alt = 1000
    print(US_Atmos.get_temperature(alt))
    print(US_Atmos.get_pressure(alt))
    print(US_Atmos.get_density(alt))
    print(US_Atmos.get_speed_of_sound(alt))
    print(US_Atmos.get_viscosity(alt))
