# Body class file
# Author : Jules Triomphe
# Date : 5 May 2019
# EPFL Rocket Team, 1015 Lausanne, Switzerland

import numpy as np

class Body:
    """
        Body object
        ==========


        Attributes
        ----------

        body_type : str
            "cylinder","tangent ogive"
            TODO : Implement "conic", "spherically blunted conic", "bi-conic", "spherically blunted tangent ogive",
                "secant ogive", "elliptical", "parabola", "3/4 parabola", "1/2 parabola", "1/2 power", "3/4 power",
                "LV-Haack", "Von Karman" or "LD-Haack", "tangent Haack", "aerospike"

        diameters : list of float
            Diameters at beginning and end of section changes.
            Diameters in m.

        diameters_position : list of float
            Position of the self.diameters in m.
            Measured from the top of the body.

    """

    # --------------------
    # CONSTRUCTOR
    # --------------------

    def __init__(self, body_type: str, diameters: "np array", diameters_position: "np array"):
        self.cone_type = body_type
        self.diameters = diameters
        self.diameters_position = diameters_position
        self.max_diameter = max(self.diameters)
        self.max_cross_section_surface = np.pi / 4 * max(self.diameters) ** 2
