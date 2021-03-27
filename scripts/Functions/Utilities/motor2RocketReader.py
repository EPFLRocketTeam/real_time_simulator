# Author : Paul Nadal
# Last update : 16 October 2020
# EPFL Rocket Team, 1015 Lausanne, Switzerland

import numpy as np

from Functions.Utilities.motorReader import motorReader


def motor2RocketReader(motorFilePath, Rocket):
    # MOTOR2ROCKETREADER reads the information contained in the RASP formatted
    # motor text file named 'motorFilePath' into the 'Rocket' structure.

    if Rocket.isHybrid == 0:
        [time, Thrust, Info] = motorReader(motorFilePath)

        # motor info
        Rocket.motor_dia = float(Info[1]) / 1000
        Rocket.motor_length = float(Info[2]) / 1000
        Rocket.motor_delay = Info[3][0]
        Rocket.propel_mass = float(Info[4])
        Rocket.motor_mass = float(Info[5])
        Rocket.casing_mass = Rocket.motor_mass - Rocket.propel_mass

        # thrust lookup table
        if time[0] > 0:
            time.insert(0, 0)
            Thrust.insert(0, 0)
        elif time[0] < 0:
            raise Exception("ERROR: in motor2RocketReader, thrust curve only allows positive time values")
        Rocket.Thrust_Time = time
        Rocket.Thrust_Force = Thrust

        # burn time
        Rocket.Burn_Time = time[-1]

        # mass variation coefficient
        A_T = np.trapz(time, Thrust)
        Rocket.Thrust2dMass_Ratio = Rocket.propel_mass / A_T

    else:
        [time, ThrustP, InfoP] = motorReader(motorFilePath)
        [timeF, ThrustF, InfoF] = motorReader(Rocket.fuel_ID)

        # prop bloc info
        Rocket.motor_diaP = float(InfoP[1]) / 1000
        Rocket.motor_lengthP = float(InfoP[2]) / 1000
        Rocket.motor_delayP = InfoP[3][0]
        Rocket.propel_massP = float(InfoP[4])
        Rocket.motor_massP = float(InfoP[5])
        Rocket.casing_massP = Rocket.motor_massP - Rocket.propel_massP

        # fuel bloc info
        Rocket.motor_diaF = float(InfoF[1]) / 1000
        Rocket.motor_lengthF = float(InfoF[2]) / 1000
        Rocket.motor_delayF = InfoF[3][0]
        Rocket.propel_massF = float(InfoF[4])
        Rocket.motor_massF = float(InfoF[5])
        Rocket.casing_massF = Rocket.motor_massF - Rocket.propel_massF

        # global info
        Rocket.motor_dia = max(Rocket.motor_diaP, Rocket.motor_diaF)
        Rocket.motor_length = Rocket.motor_lengthP + Rocket.motor_lengthF + Rocket.intermotor_d
        Rocket.motor_delay = int(InfoP[3][0])
        Rocket.propel_mass = Rocket.propel_massP + Rocket.propel_massF
        Rocket.motor_mass = Rocket.motor_massP + Rocket.motor_massF
        Rocket.casing_mass = Rocket.casing_massP - Rocket.casing_massF

        # thrust lookup table
        if time[0] > 0:
            time.insert(0, 0)
            ThrustP.insert(0, 0)
        elif time[0] < 0:
            raise Exception("ERROR: in motor2RocketReader, thrust curve only allows positive time values")
        Rocket.Thrust_Time = time
        Rocket.Thrust_Force = ThrustP

        # burn time
        Rocket.Burn_Time = time[-1]

        # mass variation coefficient
        A_T = np.trapz(time, ThrustP)
        Rocket.Thrust2dMass_Ratio = Rocket.propel_mass / A_T
