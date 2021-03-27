# Author : Paul Nadal
# Last update : 16 October 2020
# EPFL Rocket Team, 1015 Lausanne, Switzerland

from Functions.Utilities.rocketReader import rocketReader


class RocketFiled:

    def __init__(self, rocketFilePath):
        rocketReader(self, rocketFilePath)
