# Author : Paul Nadal
# Last update : 16 October 2020
# EPFL Rocket Team, 1015 Lausanne, Switzerland

def motorReader(motorFilePath):
    # MOTORREADER extracts the raw motor data from RASP formated text file
    # named 'motorFilePath'.

    # -------------------------------------------------------------------------
    # Read Motor
    # -------------------------------------------------------------------------

    with open(motorFilePath, "r") as file:

        # read informations
        line = file.readline()  # Read one line
        Info = line.split()

        # read thrust informations
        t = []  # initialization
        T = []

        while line:  # test end file
            line = file.readline()  # read one line
            tmp = line.split()
            if tmp:
                t.append(float(tmp[0]))
                T.append(float(tmp[1]))

    return [t, T, Info]
