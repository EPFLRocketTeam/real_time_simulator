# Rocket class file
# Author : Jules Triomphe
# Date   : 5 May 2019
# EPFL Rocket Team, 1015 Lausanne, Switzerland

from Rocket.Stage import Stage
from scipy.interpolate import interp1d
import numpy as np


class Rocket:
    """
    Rocket object
    =============

    Rocket representation class. Stores all geometrical and inertial information as well as the staging sequence.

    Attributes
    ----------

    stages : Stage[]
        an ordered list of stages constituting the rocket

    Constructor
    -----------

    __init__(*stages)
        Initializes a Rocket object with the Stages included in the stage list.

    Methods
    -------

    add_stage(stage)
         Add a stage representation object to the rocket.

    """

    # ------------------
    # CONSTRUCTOR
    # ------------------

    def __init__(self, *stages: "list of Stage"):
        """
        Rocket constructor, takes an ordered list of stages constituting the rocket.
        Example: rocket = Rocket()

        :type: stages: Stage[]
        """
        self.stages = []
        if np.any(stages):
            self.stages = [stage for stage in stages]
            print(self.stages)

        self.diameters = []

        if np.any(stages):
            self.diameters = [diameter for diameter in [stage.body.diameters for stage in self.stages]]

        self.diameters_position = []
        if np.any(stages):
            self.diameters_position = [stage.body.diameters_position for stage in self.stages]

        # TODO: Implement or modify the expression of these parameters
        self.cone_mode = 'on'
        self.ab_n = 3  # TODO: Create an airbrake section in Body or find another way to implement them
        self.ab_x = 1.390
        self.lug_n = 2
        self.lug_S = 1.32e-4
        self.cp_fac = 1
        self.CNa_fac = 1
        self.CD_fac = 1
        self.isHybrid = 0
        # Arbitrary, based on M3 launch lugs in Cernier 23/03/2019; 1.61e-4 for M2
        # TODO: Implement this parameter in the definition of the body and then for the rocket as a whole

        self.IMU_acc = np.zeros(3)
        self.IMU_gyro = np.zeros(3)
        self.baro_height = 0

        self.aero_force = np.zeros(3)
        self.aero_torque = np.zeros(3)

    # ------------------
    # METHODS
    # ------------------

    def add_stage(self, stage: Stage):
        """
        Add a stage representation object to the rocket.

        :param stage: stage representation object
        :return: None
        """
        self.stages.append(stage)

        if np.any(self.diameters):
            self.diameters.extend(stage.body.diameters)
        else:
            self.diameters = stage.body.diameters

        if np.any(self.diameters_position):
            corrected_diameter_position = [diameter_position + self.diameters_position[-1] for
                                           diameter_position in stage.body.diameters_position]
            self.diameters_position.extend(corrected_diameter_position)
        else:
            self.diameters_position = stage.body.diameters_position

        self.L = self.diameters_position[-1]

    def set_sensor_data(self, v_dot, w, z, R):
        self.IMU_acc = v_dot.dot(R)
        self.IMU_gyro = w.dot(R)
        self.baro_height = z

    def get_sensor_data(self):
        return [self.IMU_acc, self.IMU_gyro, self.baro_height]
        
    def set_aero(self, aero_force, aero_torque):
        self.aero_force = aero_force
        self.aero_torque = aero_torque

    def get_aero(self):
        return [self.aero_force, self.aero_torque]


    def add_lugs(self, lugs: list):
        self.lug_n = lugs[0]
        self.lug_S = lugs[1]

    def add_cg_empty_rocket(self, cg: float):
        self.cg = cg

    def set_propellant_mass(self, propellant_mass:float):
        self.propellant_mass = propellant_mass

    def set_propellant_CG(self, propellant_CG:float):
        self.propellant_CG = propellant_CG

    # TODO : check method
    def get_cg(self, t: float):

        sum1 = sum([stage.empty_cg * stage.empty_mass for stage in self.stages])
        sum2 = 0
        for stage in self.stages:
            if stage.motors is not []:
                sum2 += sum([stage.get_mass(t)*motor.get_cg for motor in stage.motors])
        return (sum1 + sum2)/self.get_mass(t)

    def get_motor_fac(self):
        for stage in self.stages:
            if stage.motors:
                return stage.motors[0].get_motor_fac()

    def set_motor_fac(self, motor_fac: float):
        for stage in self.stages:
            if stage.motors:
                for motor in stage.motors:
                    motor.set_motor_fac(motor_fac)

    def set_payload_mass(self, pl_mass: float):
        self.pl_mass = pl_mass

    def get_payload_mass(self):
        return self.pl_mass

    def get_propellant_mass(self):
        return self.propellant_mass

    def get_propellant_cg(self):
        return self.propellant_CG

    def set_rocket_inertia(self, inertia):
        self.rocket_I = inertia

    def get_rocket_inertia(self):
        return self.rocket_I

    def set_motor_Isp(self, Isp):
        self.Isp = Isp

    def get_motor_Isp(self):
        return self.Isp

    def get_dry_cg(self):
        return  self.cg

    # TODO : update method
    def get_long_inertia(self, t: float):
        return t

    # TODO : update method
    def get_rot_inertia(self, t: float):
        return t

    @property
    def get_nb_stages(self):
        return len(self.diameters)

    @property
    def get_burn_time(self):
        return sum([stage.get_burn_time for stage in self.stages])

    @property
    def get_length(self):
        return max(self.diameters_position)

    def get_empty_mass(self):
        return sum([stage.get_empty_mass() for stage in self.stages])

    def get_mass(self, t: float):
        return sum([stage.get_mass(t) for stage in self.stages])

    def get_dmass_dt(self, t: float):
        return sum([stage.get_dmass_dt(t) for stage in self.stages])

    def get_thrust(self, t: float):
        return sum([stage.get_thrust(t) for stage in self.stages])

    @property
    def get_max_diameter(self):
        return max([stage.body.max_diameter for stage in self.stages])

    @property
    def get_max_cross_section_surface(self):
        return max([stage.body.max_cross_section_surface for stage in self.stages])

    @property
    def get_fin_xt(self):
        for stage in self.stages:
            if stage.fins:
                return stage.fins[0].body_top_offset

    @property
    def get_fin_cr(self):
        for stage in self.stages:
            if stage.fins:
                return stage.fins[0].root_chord

    @property
    def get_fin_ct(self):
        for stage in self.stages:
            if stage.fins:
                return stage.fins[0].tip_chord

    @property
    def get_fin_xs(self):
        for stage in self.stages:
            if stage.fins:
                return stage.fins[0].sweep

    @property
    def get_fin_chord(self):
        for stage in self.stages:
            if stage.fins:
                return (stage.fins[0].root_chord + stage.fins[0].tip_chord) / 2
            # TODO: Implement a way to have the fin chord for each fin set. Modify drag function in accordance.

    @property
    def get_fin_span(self):
        for stage in self.stages:
            if stage.fins:
                return stage.fins[0].span

    @property
    def get_fin_exposed_planform_area(self):
        for stage in self.stages:
            if stage.fins:
                return (stage.fins[0].root_chord + stage.fins[0].tip_chord) / 2 * stage.fins[0].span

    @property
    def get_mid_fin_diameter(self):
        for stage in self.stages:
            if stage.fins:
                diameter_at_position_function = interp1d(self.diameters_position, self.diameters)
                return diameter_at_position_function(stage.fins[0].body_top_offset + stage.fins[0].root_chord / 2)
            # TODO: Find a way to organize this when multiple fin sets per body are involved or when the diameter...
            #  is changing and fins are offsetted (e.g. Hydra)

    @property
    def get_fin_virtual_planform_area(self):
        for stage in self.stages:
            if stage.fins:
                return self.get_fin_exposed_planform_area + 0.5 * self.get_mid_fin_diameter * stage.fins[0].root_chord
        # TODO: implement the alternative and define whether 0 or exception

    @property
    def get_fin_thickness(self):
        for stage in self.stages:
            if stage.fins:
                return stage.fins[0].thickness

    @property
    def get_fin_number(self):
        for stage in self.stages:
            if stage.fins:
                return stage.fins[0].number

    @property
    def set_cp_fac(self, cp_fac:float):
        self.cp_fac = cp_fac

    @property
    def set_CNa_fac(self, CNa_fac: float):
        self.CNa_fac = CNa_fac

    @property
    def set_CD_fac(self, CD_fac: float):
        self.CD_fac = CD_fac

    def get_thrust_time(self):
        for stage in self.stages:

            if len(stage.motors) > 0:
                tt = stage.get_thrust_time()
                return tt

    def get_thrust_force(self):
        for stage in self.stages:
            if len(stage.motors) > 0:
                tf = stage.get_thrust_force()
                return tf

    def get_motor_mass(self):
        for stage in self.stages:
            if len(stage.motors) > 0:
                mm = stage.get_motor_mass()
                return mm

    def get_thrust_to_mass(self):
        for stage in self.stages:
            if len(stage.motors) > 0:
                t2m = stage.get_thrust_to_mass()
                return t2m

    def get_propel_mass(self):
        for stage in self.stages:
            if len(stage.motors) > 0:
                pm = stage.get_propel_mass()
                return pm

    def get_motor_length(self):
        for stage in self.stages:
            if len(stage.motors) > 0:
                motor_length = stage.get_motor_length()
                return motor_length

    def get_motor_dia(self):
        for stage in self.stages:
            if len(stage.motors) > 0:
                motor_dia = stage.get_motor_dia()
                return motor_dia

    def get_motor_casing_mass(self):
        for stage in self.stages:
            if len(stage.motors) > 0:
                casing_mass = stage.get_motor_casing_mass()
                return casing_mass

    def get_burn_time(self):
        return sum([stage.get_burn_time for stage in self.stages])

    def set_hybrid(self, isHybrid):
        self.isHybrid = isHybrid

    def get_para_main_event(self):
        for stage in self.stages:
            if len(stage.parachutes) > 0:
                para_main_event = stage.get_para_main_event()
                return para_main_event

    def get_para_main_SCD(self):
        for stage in self.stages:
            if len(stage.parachutes) > 0:
                para_main_scd = stage.get_para_main_SCD()
                return para_main_scd

    def get_para_drogue_SCD(self):
        for stage in self.stages:
            if len(stage.parachutes) > 0:
                para_drogue_scd = stage.get_para_drogue_SCD()
                return para_drogue_scd

    def __str__(self):
        return self.stages.__str__()
