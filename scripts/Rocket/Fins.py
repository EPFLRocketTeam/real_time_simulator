# Rocket class file
# Author : Eric Brunner and Jules Triomphe
# Date   : May 5 2019
# EPFL Rocket Team, 1015 Lausanne, Switzerland


class Fins:
    """
    Fins object
    ===========

    Fin set representation object. It is limited to flat trapezoidal fins.

     Attributes
     ----------

    number : int
        Number of fins per set.

    root_chord : float
        Fin root chord length in m.

    tip_chord : float
        Fin tip chord length in m.

    span : float
        Fin span in m.

    sweep : float
        Fin sweep in m.

    thickness : float
        Fin thickness in m.

    phase : float
        Fin angular phase around stage body in degrees.

    bottom_offset : float
        Distance between bottom of stage and bottom of fin root chord.

    Constructor
    -----------

    __init__(number, root_chord, ti_chord, span, sweep, thickness, phase, bottom_offset, mass)
        Initializes a Stage object with it's name, body geometry and mass characteristics.

    Methods
    -------

    get_fin_area()
        Returns the planar area of a fin

    """

    # --------------------
    # CONSTRUCTORS
    # --------------------

    def __init__(self,
                 number: int, root_chord: float, tip_chord: float, span: float,
                 sweep: float, thickness: float, phase: float, body_top_offset: float, total_mass: float):
        self.number = number
        self.root_chord = root_chord
        self.tip_chord = tip_chord
        self.span = span
        self.sweep = sweep
        self.thickness = thickness
        self.phase = phase
        self.body_top_offset = body_top_offset
        self.total_mass = total_mass

    # --------------------
    # METHODS
    # --------------------

    @property
    def get_fin_area(self):
        """
        Computes and returns the planar area of a single fin in the set.

        :return: area of a single fin in the set given in m^2
        """
        return self.span * (self.root_chord + self.tip_chord) / 2

    def get_cg(self):
        """
        Computes the x component of the cg position for trapezoidal fins from the front extremity of the root-chord.
        The formula is x = [sum from 0 to i-1](x_i+x_{i+1})*(x_i*y_{i+1}-y_i*x_{i+1}) /
            [3*[sum from 0 to i-1](x_i*y_{i+1)-y_i*x_{i+1}})]

        The 4 points' coordinates (counter-clockwise) are (0,0), (self.root_chord,0), (self.sweep + self.tip_chord,
        self.span) and (self.sweep,self.span).

        :return:
        """
        i1 = self.root_chord * self.span
        i2 = self.tip_chord * self.span
        return ((self.root_chord + self.sweep + self.tip_chord) * i1 + self.tip_chord * i2) / (3 * (i1 + i2))
