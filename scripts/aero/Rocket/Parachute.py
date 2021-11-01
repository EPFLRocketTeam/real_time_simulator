# Rocket class file
# Author : Sayid Derder
# Date   : 07.12.2020
# EPFL Rocket Team, 1015 Lausanne, Switzerland


class Parachute:
    """
    Main: Boolean

    SCD: Float

    event: Altitude of event


    """

    def __init__(self, main: bool, SCD: float, event: float):

        self.main = main
        self.SCD = SCD
        self.event = event

    @property
    def get_main(self):
        return self.main

    @property
    def set_main(self, main: bool):
        self.main = main

    @property
    def get_SCD(self):
        return self.SCD

    @property
    def set_SCD(self, SCD: float):
        self.SCD = SCD

    @property
    def get_event(self):
        return self.event

    @property
    def set_event(self, event: float):
        self.event = event

