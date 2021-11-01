# Author : Michaël Tasev
# Last update : 16 October 2020
# EPFL Rocket Team, 1015 Lausanne, Switzerland

import numpy as np


def normalize_vector(v):
    norm = np.linalg.norm(v)
    if norm == 0:
        return v
    else:
        return v / norm
