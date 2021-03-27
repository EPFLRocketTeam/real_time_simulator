# Rocket class file
# Author : Eric Brunner and Jules Triomphe
# Date   : 5 May 2019
# EPFL Rocket Team, 1015 Lausanne, Switzerland

from Rocket.Body import Body
from Rocket.Fins import Fins
from Rocket.Motor import Motor
from Rocket.Parachute import Parachute


class Stage:
    """
         Stage representation class. A rocket is made of many stages, each with a single body, an unlimited number of
         fins and multiple motors.

         Attributes
         ----------

        id : str
            Stage identification name, e.g. 'Upper Stage' or 'Booster Stage'.

        body : Body
            Reference to the body representation object of this stage.

        fins : Fins[]
            List of fin representation objects.

        motors : Motor[]
            List of motor representation objects.

        empty_mass : float
            Stage's empty mass (no motor) in kg.

        empty_cg : float
            Stage's CG position from the upper tip of the stage (no motor) in m.

        empty_inertia : numpy.matrix
            Stage's inertia matrix (no motor) kg.m^2.

        Constructor
        -----------

        __init__(id, body, empty_mass, empty_cg, empty_inertia)
            Initializes a Stage object with it's name, body geometry and mass characteristics.

        Methods
        -------

        add_fins(Fins)
            Adds a fin representation object to the list of fins contained in this stage.

        add_motor(motor)
            Adds a motor representation object to the list of motors contained in this stage.

         """

    # ------------------
    # CONSTRUCTOR
    # ------------------

    def __init__(self, name: str, body: Body, empty_mass: float, empty_cg: float, empty_inertia: float):
        self.name = name
        self.body = body
        self.empty_mass = empty_mass
        self.empty_cg = empty_cg
        self.empty_inertia = empty_inertia

        self.fins = []
        self.motor_paths = []
        self.motors = []
        self.parachutes = []


        # self.diameters = self.body.diameters
        # self.diameters_position = self.body.diameters_position

    # ------------------
    # METHODS
    # ------------------

    def add_fins(self, fin_set_data: dict):
        """
        Adds a fin representation object to the stage.

        :param: fin_set: fin set data to be added to this stage
        :return: None
        """
        self.fins.append(Fins(**fin_set_data))

    def add_motor(self, motor_path: Motor):
        """
        Adds a motor representation object to the stage.

        :param: motor: motor path to be added to this stage
        :return: None
        """
        self.motor_paths.append(motor_path)
        self.motors.append(Motor(motor_path))

    def add_parachute(self, parachute_parameters: list):
        self.parachutes.append(Parachute(parachute_parameters[0], parachute_parameters[1], parachute_parameters[2]))


    def set_motor_fac(self, motor_fac: float):
        for motor in self.motors:
            motor.set_motor_fac(motor_fac)

    def add_airbrakes(self, ab_data:list):
        self.ab_x = ab_data[0]
        self.ab_n = ab_data[1]
        self.ab_phi = ab_data[2]

    def get_empty_mass(self):
        tmp_mass = 0
        if self.fins is not []:
            tmp_mass += sum([fin_set.total_mass for fin_set in self.fins])
        return self.empty_mass + tmp_mass

    def get_mass(self, t: float):
        tmp_mass = 0
        if self.fins is not []:
            tmp_mass += sum([fin_set.total_mass for fin_set in self.fins])
        if self.motors is not []:
            tmp_mass += sum([motor.get_total_mass(t) for motor in self.motors])
        return self.empty_mass + tmp_mass

    def get_dmass_dt(self, t: float):
        return sum([motor.get_dmass_dt(t) for motor in self.motors])

    def get_thrust(self, t: float):
        return sum([motor.get_thrust(t) for motor in self.motors])

    @property
    def get_burn_time(self):
        return sum([motor.burn_time for motor in self.motors])

    def get_thrust_time(self):
        return self.motors[0].get_thrust_time()

    def get_thrust_force(self):
        return self.motors[0].get_thrust_force()

    def get_motor_mass(self):
        return self.motors[0].total_mass

    def get_thrust_to_mass(self):
        return self.motors[0].thrust_to_mass

    def get_propel_mass(self):
        return self.motors[0].propellant_mass

    def get_motor_length(self):
        return self.motors[0].length

    def get_motor_dia(self):
        return self.motors[0].diameter

    def get_motor_casing_mass(self):
        return self.motors[0].casing_mass

    def get_para_main_event(self):
        for parachute in self.parachutes:
            if parachute.main:
                return parachute.event

    def get_para_main_SCD(self):
        for parachute in self.parachutes:
            if parachute.main:
                return parachute.SCD

    def get_para_drogue_SCD(self):
        for parachute in self.parachutes:
            if not parachute.main:
                return parachute.SCD

    def __str__(self):
        return self.name

    def __repr__(self):
        return self.name
