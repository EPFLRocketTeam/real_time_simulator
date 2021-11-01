# Author : Paul Nadal
# Last update : 16 October 2020
# EPFL Rocket Team, 1015 Lausanne, Switzerland

from math import *

from scipy.interpolate import interp1d

from Functions.Utilities.motor2RocketReader import motor2RocketReader


def rocketReader(Rocket, rocketFilePath):
    # -------------------------------------------------------------------------
    # Read Rocket
    # -------------------------------------------------------------------------

    with open(rocketFilePath, "r") as file:
        lines = file.readlines()

    Rocket.isHybrid = 0

    for i in lines:
        data = i.split()

        # integer number indicating how many stages (diameter changes) the
        # rocket has. Typically, a straight rocket will have only 3 stages:
        # tip, cone base and tail. A rocket with a boattail has one
        # additional stage.
        if data[0] == "stages":
            Rocket.stages = int(data[1])

        # list containing as many numbers as defined by the 'stages'
        # parameter. Each number indicates the diameter at that stage
        elif data[0] == "diameters":
            Rocket.diameters = [float(i) for i in data[1:Rocket.stages + 1]]

        # list containing as many numbers as defined by the 'stages'
        # parameter. Each number indicates the position from the rocket's
        # tip of the diameter change
        elif data[0] == "stage_z":
            Rocket.stage_z = [float(i) for i in data[1:Rocket.stages + 1]]

        # indicates if the aerodynamics are computed with or without the
        # cone. 'cone_mode' = 'on' indicates the cone is on the rocket,
        # 'cone_mode = off' indicates the cone is removed from the rocket
        elif data[0] == "cone_mode":
            Rocket.cone_mode = data[1]

        # integer referring to the number of fins
        elif data[0] == "fin_n":
            Rocket.fin_n = float(data[1])

        # distance of the fin's leading edge root from the rocket's tip
        elif data[0] == "fin_xt":
            Rocket.fin_xt = float(data[1])

        # fin span
        elif data[0] == "fin_s":
            Rocket.fin_s = float(data[1])

        # fin root chord
        elif data[0] == "fin_cr":
            Rocket.fin_cr = float(data[1])

        # fin tip chord
        elif data[0] == "fin_ct":
            Rocket.fin_ct = float(data[1])

        # fin thickness
        elif data[0] == "fin_t":
            Rocket.fin_t = float(data[1])

        # axial distance between the fin's leading edge root and tip
        elif data[0] == "fin_xs":
            Rocket.fin_xs = float(data[1])

        # number of lugs
        elif data[0] == "lug_n":
            Rocket.lug_n = float(data[1])

        # exposed lug surface
        elif data[0] == "lug_S":
            Rocket.lug_S = float(data[1])

        # rocket empty mass
        elif data[0] == "rocket_m":
            Rocket.rocket_m = float(data[1])

        # rocket empty inertia
        elif data[0] == "rocket_I":
            Rocket.rocket_I = float(data[1])

        # rocket center of mass for empty rocket (PL + rocket without
        # motor)
        elif data[0] == "rocket_cm":
            Rocket.rocket_cm = float(data[1])

        # position of airbrakes from rocket's tip
        elif data[0] == "ab_x":
            Rocket.ab_x = float(data[1])

        # number of airbrake fins
        elif data[0] == "ab_n":
            Rocket.ab_n = float(data[1])

        # airbrake openeing angle
        elif data[0] == "ab_phi":
            Rocket.ab_phi = data[1]

        # motor file name (with extension)
        # in case of a hybrid motor, is the propergol bloc
        # so the closest to the end of the rocket
        elif data[0] == "motor":
            Rocket.motor_ID = data[1]

        # is the tank fuel part of a hybrid motor, so
        # the furthest to the end of the rocket
        # Second parameter is
        # the distance of the valve between the
        # propergol bloc and the tank fuel
        elif data[0] == "hybr":
            Rocket.fuel_ID = data[1]
            Rocket.intermotor_d = float(data[2])
            Rocket.isHybrid = 1

        # motor thrust multiplication factor
        elif data[0] == "motor_fac":
            Rocket.motor_fac = float(data[1])

        # payload mass
        elif data[0] == "pl_mass":
            Rocket.pl_mass = float(data[1])

        # main parachute S*CD (area times drag coefficient)
        elif data[0] == "para_main_SCD":
            Rocket.para_main_SCD = float(data[1])

        # drogue parachute S*CD (area times drag coefficient)
        elif data[0] == "para_drogue_SCD":
            Rocket.para_drogue_SCD = float(data[1])

        # main parachute deployment event altitude
        elif data[0] == "para_main_event":
            Rocket.para_main_event = float(data[1])

        # error factor on center of pressure position
        elif data[0] == "cp_fac":
            Rocket.cp_fac = float(data[1])

        # error factor on normal lift coefficient derivative
        elif data[0] == "CNa_fac":
            Rocket.CNa_fac = float(data[1])

        # error factor on drag coefficient
        elif data[0] == "CD_fac":
            Rocket.CD_fac = float(data[1])

        else:
            print("ERROR: In rocket definition, unknown line identifier:" + data[0])

    # -------------------------------------------------------------------------
    # Read Motor
    # -------------------------------------------------------------------------

    motor2RocketReader(Rocket.motor_ID, Rocket)

    # -------------------------------------------------------------------------
    # Checks
    # -------------------------------------------------------------------------

    if checkStages(Rocket):
        raise Exception("ERROR: Reading rocket definition file.")
    if not (Rocket.cone_mode == "on" or Rocket.cone_mode == "off"):
        raise Exception("ERROR: Cone mode parameter " + Rocket.cone_mode + " unknown.")

    # -------------------------------------------------------------------------
    # Intrinsic parameters
    # -------------------------------------------------------------------------

    # maximum body diameter
    Rocket.dm = max(Rocket.diameters)
    # fin cord
    Rocket.fin_c = (Rocket.fin_cr + Rocket.fin_ct) / 2
    # maximum cross-sectional body area
    Rocket.Sm = pi * pow(Rocket.dm, 2) / 4
    # exposed planform fin area
    Rocket.fin_SE = (Rocket.fin_cr + Rocket.fin_ct) / 2 * Rocket.fin_s
    # body diameter at middle of fin station
    Rocket.fin_df = interp1d(Rocket.stage_z, Rocket.diameters)(Rocket.fin_xt + Rocket.fin_cr / 2)
    # virtual fin planform area
    Rocket.fin_SF = Rocket.fin_SE + 1 / 2 * Rocket.fin_df * Rocket.fin_cr
    # rocket Length
    Rocket.L = Rocket.stage_z[-1]


# -------------------------------------------------------------------------
# Sub-routines
# -------------------------------------------------------------------------

def checkStages(Rocket):
    if not (len(Rocket.diameters) == Rocket.stages and len(Rocket.stage_z) == Rocket.stages):
        flag = 1
        print(
            "ERROR: In rocket definition, rocket diameters and/or stage_z are not equal in length to the "
            "announced stages.")
    elif not (Rocket.diameters[0] == 0 and Rocket.stage_z[0] == 0):
        flag = 1
        print(
            "ERROR: In rocket definition, rocket must start with a point (diameters(1) = 0, stage_z(1) = 0)")
    else:
        flag = 0
    return flag
